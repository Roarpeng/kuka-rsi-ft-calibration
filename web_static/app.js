/* KUKA RSI 力控监控台前端：仪表组 + 状态灯（机床条）+ 示波曲线（SSE）
 * + 凿击参数设定 + 事件流 + 历史回放 + 文件管理。
 * 只依赖 vendored Chart.js；后端 API 见 web_monitor.py。 */
"use strict";

const POLL_MS = 2000;          // 状态/事件/文件轮询周期
const LIVE_WINDOW_MS = 60000;  // 实时曲线滑动窗口

const MODE_NAMES = {
  record_only: "仅记录",
  calibration_collect: "标定采集",
  calibrated_runtime: "补偿运行",
};

/* 示波器通道色（与 style.css --ch-* 一致） */
const CHANNELS = {
  Fx: "#FFD75E", Fy: "#46D6E8", Fz: "#B48CFF",
  Mx: "#7BE0A2", My: "#FF7EB5", Mz: "#8FA9FF",
};

const REDUCED_MOTION = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

/* ---------------- 通用工具 ---------------- */

function fmt(v, digits = 2) {
  return typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
}

function fmtSigned(v, digits = 2) {
  return typeof v === "number" && Number.isFinite(v)
    ? (v >= 0 ? "+" : "") + v.toFixed(digits)
    : "—";
}

function fmtMB(bytes) {
  return (bytes / 1024 / 1024).toFixed(1) + " MB";
}

function fmtGB(bytes) {
  return (bytes / 1024 / 1024 / 1024).toFixed(1) + " GB";
}

async function fetchJSON(url, options) {
  const resp = await fetch(url, options);
  const data = await resp.json().catch(() => ({}));
  if (!resp.ok) throw new Error(data.error || `HTTP ${resp.status}`);
  return data;
}

/* ---------------- 仪表组：六轴带符号微条规 ---------------- */

const GAUGE_DEFS = [
  { container: "gauge-force", keys: ["Fx", "Fy", "Fz"], scale: 160 },
  { container: "gauge-torque", keys: ["Mx", "My", "Mz"], scale: 8 },
];

const gaugeCells = {};  // key -> { val, fill }

function buildGauges() {
  for (const def of GAUGE_DEFS) {
    const host = document.getElementById(def.container);
    for (const key of def.keys) {
      const row = document.createElement("div");
      row.className = "axis";
      const name = document.createElement("span");
      name.className = "axis-name";
      name.textContent = key;
      const val = document.createElement("span");
      val.className = "axis-val";
      val.textContent = "—";
      const track = document.createElement("div");
      track.className = "axis-track";
      const zero = document.createElement("i");
      zero.className = "axis-zero";
      const fill = document.createElement("i");
      fill.className = "axis-fill";
      fill.style.background = CHANNELS[key];
      track.append(zero, fill);
      row.append(name, val, track);
      host.appendChild(row);
      gaugeCells[key] = { val, fill, scale: def.scale };
    }
  }
}

function updateGauges(tcp) {
  const keys = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"];
  for (let i = 0; i < keys.length; i++) {
    const cell = gaugeCells[keys[i]];
    if (!cell) continue;
    const v = tcp[i];
    cell.val.textContent = fmtSigned(v, i < 3 ? 1 : 2);
    if (typeof v === "number" && Number.isFinite(v)) {
      const half = Math.min(Math.abs(v) / cell.scale, 1) * 50;
      if (v >= 0) {
        cell.fill.style.left = "50%";
        cell.fill.style.width = half + "%";
      } else {
        cell.fill.style.left = (50 - half) + "%";
        cell.fill.style.width = half + "%";
      }
    } else {
      cell.fill.style.width = "0%";
    }
  }
}

/* ---------------- 机床条：状态灯 / 周期脉冲 / 表盘 ---------------- */

function setLamp(id, state, text) {
  const lamp = document.getElementById(id);
  if (!lamp) return;
  lamp.className = "lamp" + (state ? " on-" + state : "");
  lamp.querySelector("em").textContent = text;
}

const cycleDot = document.getElementById("cycle-dot");
let cycleTimer = null;

function pulseCycle() {
  if (REDUCED_MOTION) {
    cycleDot.classList.add("lit");
    return;
  }
  cycleDot.classList.add("lit");
  clearTimeout(cycleTimer);
  cycleTimer = setTimeout(() => cycleDot.classList.remove("lit"), 120);
}

function setDial(id, text) {
  document.getElementById(id).textContent = text;
}

function renderStatus(s) {
  // 模式灯
  const modeName = MODE_NAMES[s.mode] || s.mode || "—";
  setLamp("lamp-mode",
    s.mode === "calibrated_runtime" ? "run" : (s.mode === "calibration_collect" ? "warn" : null),
    s.force_mode ? modeName + " + 力控" : modeName);

  // 力控灯：接触/让位/锁存
  if (s.force) {
    let state = null;
    const parts = [s.force.in_contact ? "接触中" : "未接触"];
    if (s.force.chisel_lateral_active) { parts.push("横向让位中"); state = "kuka"; }
    if (s.force.tripped) { parts.push("超力锁存"); state = "trip"; }
    if (!state && s.force.in_contact) state = "run";
    setLamp("lamp-force", state, parts.join(" · "));
  } else {
    setLamp("lamp-force", null, "未启用");
  }

  // 链路灯
  const conn = s.connection || {};
  const disconnected = conn.last_rx_age_s === null || conn.last_rx_age_s > 2;
  setLamp("lamp-link", disconnected ? "trip" : "run",
    disconnected ? "断线" : `正常 ${fmt(conn.last_rx_age_s, 1)}s`);

  // 表盘
  setDial("dial-ov", s.ov_pro !== null && s.ov_pro !== undefined ? fmt(s.ov_pro, 1) : "—");
  setDial("dial-corr",
    s.force && s.force.corr_cumulative_mm
      ? s.force.corr_cumulative_mm.map(v => fmt(v, 2)).join(", ")
      : "—");

  const c = s.calibration || {};
  setDial("dial-calib", `${c.samples ?? "—"} / ${c.min_samples ?? "—"}`);
  document.getElementById("dial-calib-sub").textContent =
    c.has_result ? "已有标定结果" : "等待标定";

  setDial("dial-disk", s.data_dir
    ? `${fmtMB(s.data_dir.used_bytes)} / ${fmtGB(s.data_dir.cap_bytes)}` : "—");
  document.getElementById("dial-csv").textContent = s.active_csv || "—";

  // 报警条：断线 / 超力锁存
  const banner = document.getElementById("alert-banner");
  const alerts = [];
  if (disconnected) alerts.push("RSI 链路断线：超过 2s 未收到机器人数据包");
  if (s.force && s.force.tripped) alerts.push("力控超力保护已锁存，请检查传感器与工件");
  if (alerts.length) {
    banner.textContent = "⚠ " + alerts.join("；");
    banner.classList.remove("hidden");
  } else {
    banner.classList.add("hidden");
  }
}

async function pollStatus() {
  try {
    renderStatus(await fetchJSON("/api/status"));
  } catch {
    setLamp("lamp-link", "trip", "监控服务不可达");
  }
}

/* ---------------- 实时曲线（Chart.js，示波器配色） ---------------- */

const CHART_GRID = "rgba(148, 163, 178, 0.10)";
const CHART_TICK = "#7E8C9A";

/* 空态提示：无数据点时在图心画一行说明（无机器人/未连接时的引导） */
const emptyStatePlugin = {
  id: "emptyState",
  afterDraw(chart) {
    const hasData = (chart.data.datasets || []).some(ds => ds.data && ds.data.length > 0);
    if (hasData) return;
    const { ctx, chartArea } = chart;
    if (!chartArea) return;
    ctx.save();
    ctx.fillStyle = CHART_TICK;
    ctx.font = "13px 'Segoe UI', 'Microsoft YaHei', sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("等待 RSI 数据…", (chartArea.left + chartArea.right) / 2,
      (chartArea.top + chartArea.bottom) / 2);
    ctx.restore();
  },
};
Chart.register(emptyStatePlugin);

function themedChartOptions(extra = {}) {
  return {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    interaction: { intersect: false },
    plugins: {
      legend: {
        labels: {
          color: CHART_TICK,
          boxWidth: 8,
          boxHeight: 8,
          usePointStyle: false,
          font: { size: 11 },
        },
      },
    },
    scales: {
      x: { ticks: { color: CHART_TICK, maxTicksLimit: 8, font: { size: 10 } }, grid: { color: CHART_GRID } },
      y: { ticks: { color: CHART_TICK, font: { size: 10 } }, grid: { color: CHART_GRID } },
    },
    ...extra,
  };
}

function makeLiveChart(canvasId, labels, colors) {
  return new Chart(document.getElementById(canvasId), {
    type: "line",
    data: {
      labels: [],
      datasets: labels.map((label, i) => ({
        label,
        data: [],
        borderColor: colors[i % colors.length],
        backgroundColor: colors[i % colors.length],
        borderWidth: 1.5,
        pointRadius: 0,
        tension: 0.15,
      })),
    },
    options: themedChartOptions(),
  });
}

const chartForce = makeLiveChart("chart-force", ["Fx", "Fy", "Fz"],
  ["Fx", "Fy", "Fz"].map(k => CHANNELS[k]));
const chartTorque = makeLiveChart("chart-torque", ["Mx", "My", "Mz"],
  ["Mx", "My", "Mz"].map(k => CHANNELS[k]));
const chartOvpro = makeLiveChart("chart-ovpro", ["OV_PRO"], ["#E9EEF3"]);

function pushLivePoint(chart, timeLabel, values) {
  chart.data.labels.push(timeLabel);
  chart.data.datasets.forEach((ds, i) => ds.data.push(values[i]));
}

function trimLiveChart(chart) {
  const cutoff = Date.now() - LIVE_WINDOW_MS;
  const labels = chart.data.labels;
  let drop = 0;
  // labels 是 "HH:MM:SS" 字符串；用点数粗裁 + 当日时间比较
  while (drop < labels.length) {
    const parts = labels[drop].split(":");
    if (parts.length < 3) { drop++; continue; }
    const t = new Date();
    t.setHours(+parts[0], +parts[1], +parts[2].split(".")[0], 0);
    if (t.getTime() < cutoff) drop++; else break;
  }
  if (drop > 0) {
    chart.data.labels.splice(0, drop);
    chart.data.datasets.forEach(ds => ds.data.splice(0, drop));
  }
}

/* ---------------- SSE 实时帧 ---------------- */

let lastRobotStatus = null;

function startStream() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    let frame;
    try { frame = JSON.parse(ev.data); } catch { return; }
    pulseCycle();
    updateGauges(frame.tcp);
    if (typeof frame.robot_status === "boolean" && frame.robot_status !== lastRobotStatus) {
      lastRobotStatus = frame.robot_status;
      setLamp("lamp-chisel", frame.robot_status ? "kuka" : null,
        frame.robot_status ? "凿击" : "钻孔");
    }
    const label = (frame.timestamp || "").split(" ")[1] || "";
    pushLivePoint(chartForce, label, frame.tcp.slice(0, 3));
    pushLivePoint(chartTorque, label, frame.tcp.slice(3, 6));
    pushLivePoint(chartOvpro, label, [frame.ov_pro]);
    trimLiveChart(chartForce);
    trimLiveChart(chartTorque);
    trimLiveChart(chartOvpro);
    chartForce.update("none");
    chartTorque.update("none");
    chartOvpro.update("none");
  };
  es.onerror = () => {
    // EventSource 自动重连；断线由 /api/status 轮询的链路灯提示
  };
}

/* ---------------- 事件流 ---------------- */

async function pollEvents() {
  try {
    const data = await fetchJSON("/api/events?limit=100");
    const ul = document.getElementById("event-list");
    ul.innerHTML = "";
    for (const ev of data.events.slice().reverse()) {  // 最新在前
      const li = document.createElement("li");
      li.className = "event-" + ev.level;
      const time = document.createElement("time");
      time.textContent = ev.time;
      const msg = document.createElement("span");
      msg.textContent = ev.message;
      li.append(time, msg);
      ul.appendChild(li);
    }
  } catch { /* 轮询失败静默，下轮重试 */ }
}

/* ---------------- 历史回放 ---------------- */

let chartHistory = null;

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
    const datasets = [];
    for (const [col, label] of defs) {
      if (!Array.isArray(series[col])) continue;
      const key = label.slice(0, 2);
      datasets.push({
        label,
        data: series[col],
        borderColor: CHANNELS[key],
        backgroundColor: CHANNELS[key],
        borderWidth: 1.5,
        pointRadius: 0,
        tension: 0.15,
      });
    }
    const labels = data.timestamps.map(t => (t.split(" ")[1] || t));
    if (chartHistory) chartHistory.destroy();
    chartHistory = new Chart(document.getElementById("chart-history"), {
      type: "line",
      data: { labels, datasets },
      options: themedChartOptions({ scales: { x: { ticks: { color: CHART_TICK, maxTicksLimit: 12 }, grid: { color: CHART_GRID } }, y: { ticks: { color: CHART_TICK }, grid: { color: CHART_GRID } } } }),
    });
    info.textContent = `已加载 ${labels.length} 点`;
  } catch (err) {
    info.textContent = "加载失败：" + err.message;
  }
}

/* ---------------- 文件管理 ---------------- */

async function pollFiles() {
  try {
    const data = await fetchJSON("/api/files");
    renderFileTable(data.files);
    renderFileSelect(data.files);
  } catch { /* 静默，下轮重试 */ }
}

function renderFileTable(files) {
  const tbody = document.getElementById("file-tbody");
  tbody.innerHTML = "";
  if (!files.length) {
    const tr = document.createElement("tr");
    const td = document.createElement("td");
    td.colSpan = 5;
    td.className = "empty";
    td.textContent = "暂无数据文件";
    tr.appendChild(td);
    tbody.appendChild(tr);
    return;
  }
  for (const f of files) {
    const tr = document.createElement("tr");

    const tdName = document.createElement("td");
    tdName.textContent = f.name;
    tr.appendChild(tdName);

    const tdSize = document.createElement("td");
    tdSize.textContent = f.size_mb + " MB";
    tr.appendChild(tdSize);

    const tdMtime = document.createElement("td");
    tdMtime.textContent = new Date(f.mtime * 1000).toLocaleString();
    tr.appendChild(tdMtime);

    const tdState = document.createElement("td");
    const states = [];
    if (f.active) states.push("活动中");
    if (f.locked) states.push("已锁定");
    tdState.textContent = states.join(" / ") || "—";
    tr.appendChild(tdState);

    const tdOps = document.createElement("td");

    const btnDl = document.createElement("a");
    btnDl.textContent = "下载";
    btnDl.className = "btn";
    btnDl.href = "/api/files/" + encodeURIComponent(f.name);
    btnDl.setAttribute("download", f.name);
    tdOps.appendChild(btnDl);

    const btnLock = document.createElement("button");
    btnLock.textContent = f.locked ? "解锁" : "锁定";
    btnLock.onclick = async () => {
      try {
        await fetchJSON(
          `/api/files/${encodeURIComponent(f.name)}/${f.locked ? "unlock" : "lock"}`,
          { method: "POST" }
        );
        pollFiles();
      } catch (err) { alert("操作失败：" + err.message); }
    };
    tdOps.appendChild(btnLock);

    const btnDel = document.createElement("button");
    btnDel.textContent = "删除";
    btnDel.className = "btn-danger";
    btnDel.disabled = f.active;
    if (f.active) btnDel.title = "活动文件不可删除";
    btnDel.onclick = async () => {
      if (!confirm(`确认删除 ${f.name}？该操作不可恢复。`)) return;
      try {
        await fetchJSON(`/api/files/${encodeURIComponent(f.name)}/delete`, { method: "POST" });
        pollFiles();
      } catch (err) { alert("删除失败：" + err.message); }
    };
    tdOps.appendChild(btnDel);

    tr.appendChild(tdOps);
    tbody.appendChild(tr);
  }
}

function renderFileSelect(files) {
  const select = document.getElementById("history-file");
  const current = select.value;
  select.innerHTML = "";
  if (!files.length) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "暂无数据文件";
    select.appendChild(opt);
    return;
  }
  for (const f of files) {
    const opt = document.createElement("option");
    opt.value = f.name;
    opt.textContent = f.name;
    select.appendChild(opt);
  }
  // 保留原选择（仍在列表中则恢复）
  if (files.some(f => f.name === current)) select.value = current;
}

/* ---------------- 凿击让位参数设定 ---------------- */

const FORCE_CONFIG_KEYS = [
  "default_target_force_n",
  "chisel_lateral_deadband_n",
  "chisel_lateral_max_mm",
  "chisel_lateral_trip_n",
  "chisel_lateral_gain_mm_per_s_per_n",
  "chisel_lateral_sign",
];

function applyForceConfig(fc) {
  for (const key of FORCE_CONFIG_KEYS) {
    if (!(key in fc)) continue;
    const el = document.getElementById("fc-" + key);
    if (el) el.value = fc[key];
  }
}

async function loadForceConfig() {
  try {
    const data = await fetchJSON("/api/force_config");
    applyForceConfig(data.force_control || {});
  } catch { /* 力控未启用或服务不可达时静默 */ }
}

async function saveForceConfig() {
  const info = document.getElementById("force-config-info");
  const payload = {};
  for (const key of FORCE_CONFIG_KEYS) {
    const el = document.getElementById("fc-" + key);
    if (!el) continue;
    const raw = key === "chisel_lateral_sign" ? parseInt(el.value, 10) : parseFloat(el.value);
    if (!Number.isFinite(raw)) {
      info.textContent = "参数无效：" + key;
      return;
    }
    payload[key] = raw;
  }
  info.textContent = "保存中…";
  try {
    const data = await fetchJSON("/api/force_config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    applyForceConfig(data.force_control || {});
    info.textContent = "已保存（写入配置文件，重启仍生效）";
  } catch (err) {
    info.textContent = "保存失败：" + err.message;
  }
}

/* ---------------- 启动 ---------------- */

buildGauges();
document.getElementById("history-load").onclick = loadHistory;
document.getElementById("force-config-save").onclick = saveForceConfig;
pollStatus();
pollEvents();
pollFiles();
loadForceConfig();
setInterval(pollStatus, POLL_MS);
setInterval(pollEvents, POLL_MS);
setInterval(pollFiles, POLL_MS);
startStream();
