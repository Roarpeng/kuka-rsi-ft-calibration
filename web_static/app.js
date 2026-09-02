/* KUKA RSI 监控台前端逻辑：状态轮询、SSE 实时曲线、事件流、历史回放、文件管理 */
"use strict";

const POLL_MS = 2000;          // 状态/事件/文件轮询周期
const LIVE_WINDOW_MS = 60000;  // 实时曲线滑动窗口 60s

const MODE_NAMES = {
  record_only: "仅记录",
  calibration_collect: "标定采集",
  calibrated_runtime: "补偿运行",
};

/* ---------------- 通用工具 ---------------- */

function fmt(v, digits = 2) {
  return typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
}

function fmtMB(bytes) {
  return (bytes / 1024 / 1024).toFixed(1) + " MB";
}

async function fetchJSON(url, options) {
  const resp = await fetch(url, options);
  const data = await resp.json().catch(() => ({}));
  if (!resp.ok) throw new Error(data.error || `HTTP ${resp.status}`);
  return data;
}

/* ---------------- 实时曲线（Chart.js） ---------------- */

const SERIES_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4"];

function makeLiveChart(canvasId, labels) {
  const ctx = document.getElementById(canvasId);
  return new Chart(ctx, {
    type: "line",
    data: {
      labels: [],
      datasets: labels.map((label, i) => ({
        label,
        data: [],
        borderColor: SERIES_COLORS[i % SERIES_COLORS.length],
        borderWidth: 1.5,
        pointRadius: 0,
        tension: 0.1,
      })),
    },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      interaction: { intersect: false },
      scales: { x: { ticks: { maxTicksLimit: 8 } } },
    },
  });
}

const chartForce = makeLiveChart("chart-force", ["Fx", "Fy", "Fz"]);
const chartTorque = makeLiveChart("chart-torque", ["Mx", "My", "Mz"]);
const chartOvpro = makeLiveChart("chart-ovpro", ["OV_PRO"]);

function pushLivePoint(chart, timeLabel, values) {
  chart.data.labels.push(timeLabel);
  chart.data.datasets.forEach((ds, i) => ds.data.push(values[i]));
}

function trimLiveChart(chart) {
  const cutoff = Date.now() - LIVE_WINDOW_MS;
  const labels = chart.data.labels;
  let drop = 0;
  // labels 是 "HH:MM:SS" 字符串，无法直接比较日期；用点数粗裁 + 时间比较
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

function startStream() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    let frame;
    try { frame = JSON.parse(ev.data); } catch { return; }
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
    // EventSource 自动重连；断开时无需处理
  };
}

/* ---------------- 状态栏与告警横幅 ---------------- */

async function pollStatus() {
  try {
    const s = await fetchJSON("/api/status");
    document.getElementById("st-mode").textContent =
      (MODE_NAMES[s.mode] || s.mode || "—") + (s.force_mode ? " + 力控" : "");

    const conn = s.connection || {};
    const connEl = document.getElementById("st-conn");
    const disconnected = conn.last_rx_age_s === null || conn.last_rx_age_s > 2;
    connEl.textContent = disconnected
      ? "断线"
      : `正常（${fmt(conn.last_rx_age_s, 1)}s 前收包）`;
    connEl.className = disconnected ? "bad" : "good";

    document.getElementById("st-packets").textContent =
      `收 ${conn.rx_count} / 发 ${conn.tx_count} / 解析 ${conn.parse_ok_count} / 累计 ${conn.packet_count}`;

    const c = s.calibration || {};
    document.getElementById("st-calib").textContent =
      `样本 ${c.samples}/${c.min_samples ?? "—"}` + (c.has_result ? "（已有标定结果）" : "");

    const forceEl = document.getElementById("st-force");
    if (s.force) {
      const parts = [];
      parts.push(s.force.in_contact ? "接触中" : "未接触");
      if (s.force.tripped) parts.push("超力锁存");
      forceEl.textContent = parts.join(" / ");
      forceEl.className = s.force.tripped ? "bad" : "";
    } else {
      forceEl.textContent = "未启用";
      forceEl.className = "";
    }

    document.getElementById("st-ovpro").textContent =
      s.ov_pro !== null && s.ov_pro !== undefined ? fmt(s.ov_pro, 1) + " %" : "—";
    document.getElementById("st-corr").textContent =
      s.force && s.force.corr_cumulative_mm
        ? s.force.corr_cumulative_mm.map(v => fmt(v, 2)).join(", ")
        : "—";
    document.getElementById("st-csv").textContent = s.active_csv || "—";
    document.getElementById("st-usage").textContent =
      s.data_dir ? `${fmtMB(s.data_dir.used_bytes)} / ${fmtMB(s.data_dir.cap_bytes)}` : "—";

    if (s.tcp_wrench) {
      document.getElementById("st-tcp-f").textContent =
        s.tcp_wrench.slice(0, 3).map(v => fmt(v, 2)).join(", ");
      document.getElementById("st-tcp-m").textContent =
        s.tcp_wrench.slice(3, 6).map(v => fmt(v, 3)).join(", ");
    }

    // 告警横幅：断线或超力锁存
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
  } catch (err) {
    const connEl = document.getElementById("st-conn");
    connEl.textContent = "监控服务不可达";
    connEl.className = "bad";
  }
}

/* ---------------- 事件流 ---------------- */

async function pollEvents() {
  try {
    const data = await fetchJSON("/api/events?limit=100");
    const ul = document.getElementById("event-list");
    ul.innerHTML = "";
    // 最新在前
    for (const ev of data.events.slice().reverse()) {
      const li = document.createElement("li");
      li.className = "event-" + ev.level;
      li.textContent = `[${ev.time}] [${ev.level}] ${ev.message}`;
      ul.appendChild(li);
    }
  } catch { /* 轮询失败静默，下轮重试 */ }
}

/* ---------------- 历史回放 ---------------- */

let chartHistory = null;

async function loadHistory() {
  const select = document.getElementById("history-file");
  const name = select.value;
  if (!name) { alert("请先选择数据文件"); return; }
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
      if (Array.isArray(series[col])) {
        datasets.push({
          label,
          data: series[col],
          borderColor: SERIES_COLORS[datasets.length % SERIES_COLORS.length],
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.1,
        });
      }
    }
    const labels = data.timestamps.map(t => (t.split(" ")[1] || t));
    if (chartHistory) chartHistory.destroy();
    chartHistory = new Chart(document.getElementById("chart-history"), {
      type: "line",
      data: { labels, datasets },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: { intersect: false },
        scales: { x: { ticks: { maxTicksLimit: 12 } } },
      },
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
  for (const f of files) {
    const opt = document.createElement("option");
    opt.value = f.name;
    opt.textContent = f.name;
    select.appendChild(opt);
  }
  // 保留原选择（仍在列表中则恢复）
  if (files.some(f => f.name === current)) select.value = current;
}

/* ---------------- 启动 ---------------- */

document.getElementById("history-load").onclick = loadHistory;
pollStatus();
pollEvents();
pollFiles();
setInterval(pollStatus, POLL_MS);
setInterval(pollEvents, POLL_MS);
setInterval(pollFiles, POLL_MS);
startStream();
