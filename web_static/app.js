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
  // 模式灯：服务模式（力控/监控）优先展示，标定采集时覆盖提示
  let modeState = null;
  let modeText;
  if (s.mode === "calibration_collect") {
    modeText = "标定采集";
    modeState = "warn";
  } else if (s.force_mode) {
    modeText = "力控";
    modeState = "run";
  } else {
    modeText = "监控";
  }
  setLamp("lamp-mode", modeState, modeText);

  // 力控灯：接触/让位/对中/锁存
  if (s.force) {
    let state = null;
    const parts = [s.force.in_contact ? "接触中" : "未接触"];
    if (s.force.chisel_lateral_active) { parts.push("横向让位中"); state = "kuka"; }
    if (s.force.align_active) { parts.push("轴线对中中"); if (state !== "trip") state = "kuka"; }
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

/* ---------------- 实时曲线（uPlot，示波器配色） ---------------- */

const chartForce = makeLiveChart("chart-force", ["Fx", "Fy", "Fz"],
  ["Fx", "Fy", "Fz"].map(k => CHANNELS[k]));
const chartTorque = makeLiveChart("chart-torque", ["Mx", "My", "Mz"],
  ["Mx", "My", "Mz"].map(k => CHANNELS[k]));
const chartOvpro = makeLiveChart("chart-ovpro", ["OV_PRO"], ["#E9EEF3"]);

/* ---------------- SSE 实时帧 ---------------- */

let lastRobotStatus = null;

function startStream() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    let frame;
    try { frame = JSON.parse(ev.data); } catch { return; }
    pulseCycle();
    updateGauges(frame.tcp);
    if (frame.robot_status !== undefined && frame.robot_status !== null
        && frame.robot_status !== lastRobotStatus) {
      lastRobotStatus = frame.robot_status;
      if (frame.robot_status === 1) setLamp("lamp-chisel", "warn", "标定");
      else if (frame.robot_status === 3) setLamp("lamp-chisel", "kuka", "凿击");
      else setLamp("lamp-chisel", null, "钻孔");
    }
    const tSec = frameTimeSec(frame.timestamp);
    chartForce.push(tSec, frame.tcp.slice(0, 3));
    chartTorque.push(tSec, frame.tcp.slice(3, 6));
    chartOvpro.push(tSec, [frame.ov_pro]);
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

/* 分组定义：顺序即显示顺序；desc 来自 GET /api/force_config 的 fields 元数据 */
const FORCE_CONFIG_GROUPS = [
  { title: "钻孔调速（力-速度跟随）", keys: [
    "default_target_force_n", "contact_threshold_n", "deadband_n",
    "default_ov_pro", "ov_pro_slew_pct", "max_force_n",
    "trip_clear_s", "contact_lost_s",
  ]},
  { title: "凿击横向让位", keys: [
    "chisel_lateral_deadband_n", "chisel_lateral_gain_mm_per_s_per_n",
    "chisel_lateral_max_mm", "chisel_lateral_trip_n", "chisel_lateral_sign",
    "chisel_lateral_median_window", "chisel_lateral_lpf_hz",
  ]},
  { title: "轴线对中", keys: [
    "align_drill_enable", "align_chisel_enable", "align_deadband_nm",
    "align_gain_deg_per_s_per_nm", "align_per_cycle_max_deg", "align_max_deg",
    "align_trip_nm", "align_sign", "align_median_window", "align_lpf_hz",
  ]},
  { title: "信号滤波", keys: [
    "tcp_filter_enable", "tcp_filter_median_window", "tcp_filter_lpf_hz",
    "filter_median_window", "filter_protect_window", "filter_lpf_hz",
  ]},
  { title: "轴与符号（结构参数，改动须重验方向）", keys: [
    "axis", "press_sign", "press_motion_sign",
  ]},
  { title: "叠加测试 / 限幅（高级）", keys: [
    "per_cycle_max_mm", "cumulative_max_mm", "kp_mm_per_s_per_n",
    "ki_mm_per_s2_per_n", "integral_limit_n_s", "advance_speed_mm_s",
    "search_speed_mm_s", "search_before_contact", "path_feed_mm_s", "path_feed_hold_s",
  ]},
];

let forceFieldsMeta = {};   // key -> {desc,type,min,max,options}

function fieldLabel(key) {
  return key
    .replace(/^fc_/, "")
    .replace(/_n$/, " (N)")
    .replace(/_nm$/, " (N·m)")
    .replace(/_mm$/, " (mm)")
    .replace(/_deg$/, " (°)")
    .replace(/_hz$/, " (Hz)")
    .replace(/_s$/, " (s)")
    .replace(/_pct$/, " (%)")
    .replace(/_n_per_n$/, "")
    .replace(/_/g, " ");
}

function buildForceConfigForm() {
  const host = document.getElementById("force-config-groups");
  if (!host) return;
  host.innerHTML = "";
  for (const group of FORCE_CONFIG_GROUPS) {
    const section = document.createElement("details");
    section.className = "param-group";
    const summary = document.createElement("summary");
    summary.textContent = group.title;
    section.appendChild(summary);
    const grid = document.createElement("div");
    grid.className = "param-form";
    for (const key of group.keys) {
      const meta = forceFieldsMeta[key];
      const label = document.createElement("label");
      label.className = meta && meta.type === "bool" ? "check" : "";
      const span = document.createElement("span");
      span.textContent = fieldLabel(key);
      label.appendChild(span);
      let input;
      if (meta && meta.type === "bool") {
        input = document.createElement("input");
        input.type = "checkbox";
      } else if (meta && meta.options) {
        input = document.createElement("select");
        for (const opt of meta.options) {
          const o = document.createElement("option");
          o.value = opt;
          o.textContent = opt;
          input.appendChild(o);
        }
      } else {
        input = document.createElement("input");
        input.type = "number";
        if (meta) {
          input.min = meta.min;
          input.max = meta.max;
          input.step = (meta.max - meta.min) > 20 ? 1 : (meta.max - meta.min) > 2 ? 0.1 : 0.005;
        }
      }
      input.id = "fc-" + key;
      if (meta) input.title = meta.desc;   // 悬停显示参数意义
      label.appendChild(input);
      grid.appendChild(label);
    }
    section.appendChild(grid);
    host.appendChild(section);
  }
  // 默认展开前四组（常用）
  host.querySelectorAll("details").forEach((d, i) => { d.open = i < 4; });
}

function applyForceConfig(fc) {
  for (const group of FORCE_CONFIG_GROUPS) {
    for (const key of group.keys) {
      if (!(key in fc)) continue;
      const el = document.getElementById("fc-" + key);
      if (!el) continue;
      if (el.type === "checkbox") el.checked = !!fc[key];
      else el.value = fc[key];
    }
  }
}

async function loadForceConfig() {
  try {
    const data = await fetchJSON("/api/force_config");
    forceFieldsMeta = data.fields || {};
    buildForceConfigForm();
    applyForceConfig(data.force_control || {});
  } catch { /* 力控未启用或服务不可达时静默 */ }
}

async function saveForceConfig() {
  const info = document.getElementById("force-config-info");
  const payload = {};
  for (const group of FORCE_CONFIG_GROUPS) {
    for (const key of group.keys) {
      const el = document.getElementById("fc-" + key);
      if (!el) continue;
      let raw;
      if (el.type === "checkbox") raw = el.checked;
      else if (el.tagName === "SELECT") raw = el.value;
      else raw = parseFloat(el.value);
      if (typeof raw !== "boolean" && typeof raw !== "string" && !Number.isFinite(raw)) {
        info.textContent = "参数无效：" + key;
        return;
      }
      payload[key] = raw;
    }
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

/* ---------------- 工具系手型约定 ---------------- */

function renderFrameConvention(data) {
  const c = data.convention || {};
  document.getElementById("fc-hand").value = c.hand || "right";
  for (const key of ["thumb", "index", "middle"]) {
    const el = document.getElementById("fc-" + key);
    if (el && c[key]) el.value = c[key];
  }
  const derived = data.derived || {};
  const tbody = document.getElementById("frame-tbody");
  tbody.innerHTML = "";
  for (const p of derived.pushes || []) {
    const tr = document.createElement("tr");
    for (const text of [`沿${p.finger}（+${p.axis}）推尖端`, p.expect_force, p.expect_moment]) {
      const td = document.createElement("td");
      td.textContent = text;
      tr.appendChild(td);
    }
    tbody.appendChild(tr);
  }
  const rot = derived.rotations || {};
  document.getElementById("frame-rotations").textContent =
    [rot.B, rot.C].filter(Boolean).join("；") || "";
  document.getElementById("frame-feed").textContent =
    `进给方向 = 工具 +X（${derived.feed_finger || "—"}所指）。此约定用于方向推演与手扳验证，控制律实际符号仍由"让位方向/对中方向"参数决定。`;
}

async function loadFrameConvention() {
  try {
    renderFrameConvention(await fetchJSON("/api/frame_convention"));
  } catch { /* 静默 */ }
}

async function saveFrameConvention() {
  const info = document.getElementById("frame-info");
  const payload = {
    hand: document.getElementById("fc-hand").value,
    thumb: document.getElementById("fc-thumb").value,
    index: document.getElementById("fc-index").value,
    middle: document.getElementById("fc-middle").value,
  };
  info.textContent = "保存中…";
  try {
    const data = await fetchJSON("/api/frame_convention", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    renderFrameConvention(data);
    info.textContent = "已保存";
  } catch (err) {
    info.textContent = "保存失败：" + err.message;
  }
}

/* ---------------- 启动 ---------------- */

buildGauges();
document.getElementById("history-load").onclick = loadHistory;
document.getElementById("force-config-save").onclick = saveForceConfig;
document.getElementById("frame-save").onclick = saveFrameConvention;
pollStatus();
pollEvents();
pollFiles();
loadForceConfig();
loadFrameConvention();
setInterval(pollStatus, POLL_MS);
setInterval(pollEvents, POLL_MS);
setInterval(pollFiles, POLL_MS);
startStream();
