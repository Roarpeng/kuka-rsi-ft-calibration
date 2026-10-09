# -*- coding: utf-8 -*-
"""一次性补丁：Chart.js -> uPlot。用后即删。"""

# 1) index.html
with open("web_static/index.html", encoding="utf-8") as f:
    t = f.read()
t = t.replace(
    '  <link rel="stylesheet" href="/static/style.css">',
    '  <link rel="stylesheet" href="/static/style.css">\n  <link rel="stylesheet" href="/static/vendor/uplot.min.css">',
)
for chart_id in ("chart-force", "chart-torque", "chart-ovpro", "chart-history"):
    t = t.replace(f'<canvas id="{chart_id}"></canvas>', f'<div id="{chart_id}" class="uplot-fill"></div>')
t = t.replace(
    '<script src="/static/vendor/chart.umd.min.js"></script>',
    '<script src="/static/vendor/uplot.iife.min.js"></script>\n  <script src="/static/uplot_charts.js"></script>',
)
with open("web_static/index.html", "w", encoding="utf-8") as f:
    f.write(t)
print("index.html OK")

# 2) debug.html
with open("web_static/debug.html", encoding="utf-8") as f:
    t = f.read()
t = t.replace(
    '  <link rel="stylesheet" href="/static/style.css">',
    '  <link rel="stylesheet" href="/static/style.css">\n  <link rel="stylesheet" href="/static/vendor/uplot.min.css">',
)
for chart_id in ("chart-pos", "chart-att"):
    t = t.replace(f'<canvas id="{chart_id}"></canvas>', f'<div id="{chart_id}" class="uplot-fill"></div>')
t = t.replace(
    '<script src="/static/vendor/chart.umd.min.js"></script>',
    '<script src="/static/vendor/uplot.iife.min.js"></script>\n  <script src="/static/uplot_charts.js"></script>',
)
with open("web_static/debug.html", "w", encoding="utf-8") as f:
    f.write(t)
print("debug.html OK")

# 3) app.js: 移除 Chart.js 封装，改用 uplot_charts
with open("web_static/app.js", encoding="utf-8") as f:
    t = f.read()

start = t.index("/* ---------------- 实时曲线（Chart.js，示波器配色） ---------------- */")
end = t.index("/* ---------------- SSE 实时帧 ---------------- */")
t = t[:start] + '''/* ---------------- 实时曲线（uPlot，示波器配色） ---------------- */

const chartForce = makeLiveChart("chart-force", ["Fx", "Fy", "Fz"],
  ["Fx", "Fy", "Fz"].map(k => CHANNELS[k]));
const chartTorque = makeLiveChart("chart-torque", ["Mx", "My", "Mz"],
  ["Mx", "My", "Mz"].map(k => CHANNELS[k]));
const chartOvpro = makeLiveChart("chart-ovpro", ["OV_PRO"], ["#E9EEF3"]);

''' + t[end:]

# SSE 推送改 push
old = '''    const label = (frame.timestamp || "").split(" ")[1] || "";
    pushLivePoint(chartForce, label, frame.tcp.slice(0, 3));
    pushLivePoint(chartTorque, label, frame.tcp.slice(3, 6));
    pushLivePoint(chartOvpro, label, [frame.ov_pro]);
    trimLiveChart(chartForce);
    trimLiveChart(chartTorque);
    trimLiveChart(chartOvpro);
    chartForce.update("none");
    chartTorque.update("none");
    chartOvpro.update("none");'''
new = '''    const tSec = frameTimeSec(frame.timestamp);
    chartForce.push(tSec, frame.tcp.slice(0, 3));
    chartTorque.push(tSec, frame.tcp.slice(3, 6));
    chartOvpro.push(tSec, [frame.ov_pro]);'''
assert old in t, "SSE block"
t = t.replace(old, new)

# 历史回放改 uPlot
old_start = t.index("let chartHistory = null;")
old_end = t.index("/* ---------------- 文件管理 ---------------- */")
t = t[:old_start] + '''let chartHistory = null;

async function loadHistory() {
  const select = document.getElementById("history-file");
  const name = select.value;
  if (!name) { alert("暂无数据文件，无法回放"); return; }
  const params = new URLSearchParams();
  const startVal = document.getElementById("history-start").value;
  const endVal = document.getElementById("history-end").value;
  // datetime-local "YYYY-MM-DDTHH:MM:SS" → "YYYY-MM-DD HH:MM:SS"
  if (startVal) params.set("start", startVal.replace("T", " "));
  if (endVal) params.set("end", endVal.replace("T", " "));
  params.set("points", "2000");

  const info = document.getElementById("history-info");
  info.textContent = "加载中…";
  try {
    const data = await fetchJSON(
      `/api/files/${encodeURIComponent(name)}/series?` + params.toString()
    );
    const series = data.series || {};
    const defs = [
      ["tcp_Fx_N", "Fx (N)"], ["tcp_Fy_N", "Fy (N)"], ["tcp_Fz_N", "Fz (N)"],
      ["tcp_Mx_Nm", "Mx (N·m)"], ["tcp_My_Nm", "My (N·m)"], ["tcp_Mz_Nm", "Mz (N·m)"],
    ];
    const labels = [];
    const colors = [];
    const arrays = [];
    for (const [col, label] of defs) {
      if (!Array.isArray(series[col])) continue;
      labels.push(label);
      colors.push(CHANNELS[label.slice(0, 2)]);
      arrays.push(series[col]);
    }
    const times = data.timestamps.map(ts => frameTimeSec(ts));
    chartHistory = renderHistoryChart("chart-history", labels, colors, times, arrays);
    info.textContent = `已加载 ${times.length} 点`;
  } catch (err) {
    info.textContent = "加载失败：" + err.message;
  }
}

''' + t[old_end:]

with open("web_static/app.js", "w", encoding="utf-8") as f:
    f.write(t)
print("app.js OK")

# 4) debug.js: 同样替换
with open("web_static/debug.js", encoding="utf-8") as f:
    t = f.read()

start = t.index("/* ---------------- 图表（与主监控台同款主题） ---------------- */")
end = t.index("/* ---------------- SSE：位姿回读 ---------------- */")
t = t[:start] + '''/* ---------------- 图表（uPlot，与主监控台同款主题） ---------------- */

const chartPos = makeLiveChart("chart-pos", ["Act_X", "Act_Y", "Act_Z"], CHANNELS_POS);
const chartAtt = makeLiveChart("chart-att", ["Act_A", "Act_B", "Act_C"], CHANNELS_ATT);

''' + t[end:]

old = '''    const label = (frame.timestamp || "").split(" ")[1] || "";
    pushLivePoint(chartPos, label, act.slice(0, 3));
    pushLivePoint(chartAtt, label, act.slice(3, 6));
    trimLiveChart(chartPos);
    trimLiveChart(chartAtt);
    chartPos.update("none");
    chartAtt.update("none");'''
new = '''    const tSec = frameTimeSec(frame.timestamp);
    chartPos.push(tSec, act.slice(0, 3));
    chartAtt.push(tSec, act.slice(3, 6));'''
assert old in t, "debug SSE"
t = t.replace(old, new)

with open("web_static/debug.js", "w", encoding="utf-8") as f:
    f.write(t)
print("debug.js OK")

# 5) style.css: uPlot 容器样式 + 移除旧 canvas 强制尺寸规则
with open("web_static/style.css", encoding="utf-8") as f:
    t = f.read()
old = """.chart-box canvas { width: 100% !important; height: calc(100% - 22px) !important; }
.chart-box.replay canvas { height: 100% !important; }"""
assert old in t
t = t.replace(old, """/* uPlot 容器：占满图表框，图例由 uPlot 自绘 */
.uplot-fill { width: 100%; height: 100%; position: relative; }

.uplot-empty {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--ink-3);
  font-size: 13px;
  pointer-events: none;
}

.uplot-fill .uplot { font-size: 11px; }
.uplot-fill .u-legend { font-size: 11px; color: var(--ink-2); }
.uplot-fill .u-legend .u-marker { border-radius: 2px; }""")
with open("web_static/style.css", "w", encoding="utf-8") as f:
    f.write(t)
print("style.css OK")
