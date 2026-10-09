/* uPlot 图表封装（主监控台与调试台共用）：实时滑窗曲线 + 历史回放。
 * 依赖 /static/vendor/uplot.iife.min.js（本地化，现场离线）。 */
"use strict";

const UPLOT_TICK = "#7E8C9A";
const UPLOT_GRID = "rgba(148, 163, 178, 0.10)";

function _uplotBaseOpts(labels, colors) {
  return {
    class: "uplot-dark",
    width: 320,
    height: 200,
    cursor: { drag: { x: false, y: false, setScale: false } },
    legend: { show: true, live: false },
    scales: { x: { time: false } },
    axes: [
      {
        stroke: UPLOT_TICK,
        grid: { stroke: UPLOT_GRID },
        values: (u, vals) => vals.map(v => new Date(v * 1000).toTimeString().slice(0, 8)),
      },
      { stroke: UPLOT_TICK, grid: { stroke: UPLOT_GRID } },
    ],
    series: [
      {},
      ...labels.map((label, i) => ({
        label,
        stroke: colors[i % colors.length],
        width: 1.5,
        spanGaps: true,
      })),
    ],
  };
}

function _uplotHookResize(u, host) {
  if (typeof ResizeObserver === "undefined") return;
  const ro = new ResizeObserver(() => {
    const w = Math.max(80, host.clientWidth);
    const h = Math.max(60, host.clientHeight);
    u.setSize({ width: w, height: h });
  });
  ro.observe(host);
}

/* 实时滑窗曲线：push(时间秒, 各系列值)，内部裁剪窗口并整表刷新。 */
function makeLiveChart(containerId, labels, colors, windowMs = 60000) {
  const host = document.getElementById(containerId);
  const empty = document.createElement("div");
  empty.className = "uplot-empty";
  empty.textContent = "等待 RSI 数据…";
  host.appendChild(empty);

  const opts = _uplotBaseOpts(labels, colors);
  const u = new uPlot(opts, [new Float64Array(0)], host);
  _uplotHookResize(u, host);

  const data = [[]].concat(labels.map(() => []));
  const cutoffMs = windowMs / 1000;

  return {
    push(timeSec, values) {
      if (typeof timeSec !== "number" || !Number.isFinite(timeSec)) return;
      data[0].push(timeSec);
      for (let i = 0; i < data.length - 1; i++) {
        data[i + 1].push(values[i] === undefined || values[i] === null ? null : values[i]);
      }
      const cutoff = timeSec - cutoffMs;
      let drop = 0;
      while (drop < data[0].length && data[0][drop] < cutoff) drop++;
      if (drop > 0) data.forEach(arr => arr.splice(0, drop));
      empty.style.display = data[0].length ? "none" : "";
      u.setData(data);
    },
    chart: u,
  };
}

/* 历史曲线：一次性渲染（timesSec 为秒级时间戳数组，seriesArrays 与 labels 对齐）。 */
function renderHistoryChart(containerId, labels, colors, timesSec, seriesArrays) {
  const host = document.getElementById(containerId);
  host.innerHTML = "";
  const opts = _uplotBaseOpts(labels, colors);
  const u = new uPlot(opts, [timesSec, ...seriesArrays], host);
  _uplotHookResize(u, host);
  return u;
}

/* "YYYY-MM-DD HH:MM:SS.mmm" -> 秒级时间戳；解析失败回退当前时刻。 */
function frameTimeSec(timestampText) {
  if (timestampText) {
    const t = new Date(timestampText.trim().replace(" ", "T")).getTime();
    if (Number.isFinite(t)) return t / 1000;
  }
  return Date.now() / 1000;
}
