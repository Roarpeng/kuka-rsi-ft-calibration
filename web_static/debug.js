/* KUKA RSI 调试台：固定输出 RKorr 每拍增量 + OV_PRO，监控 Act 位姿回读。
 * 会真实移动机械臂；所有写操作经 POST /api/debug_override（服务端二次限幅）。 */
"use strict";

const POLL_MS = 2000;          // 状态轮询
const DEBUG_POLL_MS = 1000;    // 固定输出状态轮询（较快的启停状态反馈）
const LIVE_WINDOW_MS = 60000;

const CHANNELS_POS = ["#FFD75E", "#46D6E8", "#B48CFF"];
const CHANNELS_ATT = ["#7BE0A2", "#FF7EB5", "#8FA9FF"];

const REDUCED_MOTION = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

let lastRobotStatus = null;

const DEBUG_KEYS = ["x", "y", "z", "a", "b", "c"];
let limits = { trans_max: 0.08, rot_max: 0.05, cycle_s: 0.004 };
let enabled = false;      // 服务端当前固定输出状态
let refAct = null;        // 启用时刻的 Act（基座系六值），用于 Δ 显示

/* ---------------- 通用工具 ---------------- */

function fmt(v, digits = 2) {
  return typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
}

function fmtSigned(v, digits = 2) {
  return typeof v === "number" && Number.isFinite(v)
    ? (v >= 0 ? "+" : "") + v.toFixed(digits)
    : "—";
}

async function fetchJSON(url, options) {
  const resp = await fetch(url, options);
  const data = await resp.json().catch(() => ({}));
  if (!resp.ok) throw new Error(data.error || `HTTP ${resp.status}`);
  return data;
}

/* ---------------- 状态灯 / 周期脉冲 ---------------- */

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

/* ---------------- 图表（uPlot，与主监控台同款主题） ---------------- */

const chartPos = makeLiveChart("chart-pos", ["Act_X", "Act_Y", "Act_Z"], CHANNELS_POS);
const chartAtt = makeLiveChart("chart-att", ["Act_A", "Act_B", "Act_C"], CHANNELS_ATT);

/* ---------------- SSE：位姿回读 ---------------- */

function startStream() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    let frame;
    try { frame = JSON.parse(ev.data); } catch { return; }
    pulseCycle();
    const act = frame.act;
    if (!Array.isArray(act) || act.length < 6) return;
    if (enabled && refAct === null) refAct = act.slice();
    for (const [i, key] of ["x", "y", "z", "a", "b", "c"].entries()) {
      document.getElementById("pose-" + key).textContent = fmt(act[i], i < 3 ? 1 : 2);
      if (enabled && refAct) {
        document.getElementById("delta-" + key).textContent = fmtSigned(act[i] - refAct[i], i < 3 ? 2 : 3);
      }
    }
    if (frame.robot_status !== undefined && frame.robot_status !== null
        && frame.robot_status !== lastRobotStatus) {
      lastRobotStatus = frame.robot_status;
      if (frame.robot_status === 1) setLamp("lamp-service", "warn", "标定");
      else if (frame.robot_status === 3) setLamp("lamp-service", "kuka", "凿击");
      else setLamp("lamp-service", null, "钻孔");
    }
    const tSec = frameTimeSec(frame.timestamp);
    chartPos.push(tSec, act.slice(0, 3));
    chartAtt.push(tSec, act.slice(3, 6));
  };
  es.onerror = () => { /* 自动重连；断线由链路灯提示 */ };
}

/* ---------------- 状态轮询 ---------------- */

async function pollStatus() {
  try {
    const s = await fetchJSON("/api/status");
    const conn = s.connection || {};
    const disconnected = conn.last_rx_age_s === null || conn.last_rx_age_s > 2;
    setLamp("lamp-link", disconnected ? "trip" : "run",
      disconnected ? "断线" : `正常 ${fmt(conn.last_rx_age_s, 1)}s`);
    setLamp("lamp-service", s.force_mode ? "run" : null, s.force_mode ? "力控" : "监控");
    const rk = s.rkorr || {};
    document.getElementById("send-x").textContent = fmt(rk["RKorr.X"], 4);
    document.getElementById("send-y").textContent = fmt(rk["RKorr.Y"], 4);
    document.getElementById("send-z").textContent = fmt(rk["RKorr.Z"], 4);
    document.getElementById("send-ov").textContent =
      s.ov_pro !== null && s.ov_pro !== undefined ? fmt(s.ov_pro, 1) + " %" : "—";
  } catch {
    setLamp("lamp-link", "trip", "监控服务不可达");
  }
}

/* ---------------- 固定输出控制 ---------------- */

let lastSynced = null;   // 最近一次与服务器一致的状态；轮询只在服务端被外部变更时才回写字段

function readInputs() {
  const payload = { enabled: true };
  for (const key of DEBUG_KEYS) {
    const el = document.getElementById("dbg-" + key);
    const raw = parseFloat(el.value);
    if (!Number.isFinite(raw) || raw === 0) { payload[key] = 0; continue; }  // 空值按 0
    const bound = "xyz".includes(key) ? limits.trans_max : limits.rot_max;
    if (Math.abs(raw) > bound + 1e-9) {
      throw new Error(`${key.toUpperCase()} = ${raw} 超出每拍限幅 ±${bound}`);
    }
    payload[key] = raw;
  }
  const ov = parseFloat(document.getElementById("dbg-ov").value);
  if (!Number.isFinite(ov) || ov < 0 || ov > 100) throw new Error("OV_PRO 须在 0–100 之间");
  payload.ov_pro = ov;
  return payload;
}

function writeFields(debug) {
  for (const key of DEBUG_KEYS) document.getElementById("dbg-" + key).value = debug[key];
  document.getElementById("dbg-ov").value = debug.ov_pro;
  refreshHints();
}

function applyDebugState(data) {
  limits = data.limits || limits;
  document.getElementById("lim-trans").textContent = "±" + limits.trans_max;
  document.getElementById("lim-rot").textContent = "±" + limits.rot_max;
  setEnabledUi(data.debug.enabled);
  // 预填优先：轮询不得覆盖用户正在准备、尚未下发的数值。
  // 仅当服务端状态相对上次同步发生变化（首次加载 / RSI 重启自动停用 /
  // 其他页面操作）时，才把字段回写成服务端真值。
  const changed = lastSynced === null
    || JSON.stringify(data.debug) !== JSON.stringify(lastSynced);
  if (changed) {
    writeFields(data.debug);
    lastSynced = { ...data.debug };
  }
}

function setEnabledUi(isEnabled) {
  enabled = isEnabled;
  if (!isEnabled) refAct = null;
  setLamp("lamp-debug", isEnabled ? "kuka" : null, isEnabled ? "已启用" : "已停用");
  document.getElementById("dbg-enable").classList.toggle("hidden", isEnabled);
  document.getElementById("dbg-update").classList.toggle("hidden", !isEnabled);
  document.getElementById("dbg-zero").classList.toggle("hidden", !isEnabled);
  document.getElementById("dbg-stop").classList.toggle("hidden", !isEnabled);
  if (!isEnabled) {
    for (const key of ["x", "y", "z", "a", "b", "c"]) {
      document.getElementById("delta-" + key).textContent = "—";
    }
  }
}

async function pollDebug() {
  try {
    applyDebugState(await fetchJSON("/api/debug_override"));
  } catch { /* 服务不可达时静默 */ }
}

async function postDebug(payload) {
  try {
    applyDebugState(await fetchJSON("/api/debug_override", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    }));
    return true;
  } catch (err) {
    alert("下发失败：" + err.message);
    return false;
  }
}

/* ---------------- 输入区：等效速度提示 ---------------- */

function refreshHints() {
  const per_s = 1.0 / limits.cycle_s;
  for (const el of document.querySelectorAll(".hint[data-ax]")) {
    const key = el.dataset.ax;
    const raw = parseFloat(document.getElementById("dbg-" + key).value);
    if (!Number.isFinite(raw) || raw === 0) { el.textContent = "静止"; continue; }
    const unit = "xyz".includes(key) ? "mm/s" : "°/s";
    el.textContent = "≈ " + fmtSigned(raw * per_s, 1) + " " + unit;
  }
}

function wireControls() {
  for (const key of DEBUG_KEYS) {
    const el = document.getElementById("dbg-" + key);
    el.addEventListener("input", refreshHints);
  }
  document.getElementById("dbg-ov").addEventListener("input", refreshHints);

  document.getElementById("dbg-enable").onclick = async () => {
    let payload;
    try { payload = readInputs(); }
    catch (err) { alert(err.message); return; }
    if (!confirm(`确认启用固定输出？\nRKorr = ${["x", "y", "z", "a", "b", "c"].map(k => k.toUpperCase() + " " + fmtSigned(payload[k], 3)).join(", ")}\nOV_PRO = ${payload.ov_pro}%\n机械臂将按此增量连续移动，请确认现场安全。`)) return;
    await postDebug(payload);
  };

  document.getElementById("dbg-update").onclick = async () => {
    let payload;
    try { payload = readInputs(); }
    catch (err) { alert(err.message); return; }
    await postDebug(payload);
  };

  document.getElementById("dbg-zero").onclick = async () => {
    for (const key of DEBUG_KEYS) document.getElementById("dbg-" + key).value = 0;
    document.getElementById("dbg-ov").value = 100;
    refreshHints();
    await postDebug({ enabled: true, x: 0, y: 0, z: 0, a: 0, b: 0, c: 0, ov_pro: 100 });
  };

  document.getElementById("dbg-stop").onclick = async () => {
    if (!confirm("停止固定输出？将下发零增量（保持当前叠加）并把 OV_PRO 回到 100%。")) return;
    await postDebug({ enabled: false });
  };
}

/* ---------------- 启动 ---------------- */

wireControls();
refreshHints();
pollDebug();
pollStatus();
setInterval(pollDebug, DEBUG_POLL_MS);
setInterval(pollStatus, POLL_MS);
startStream();
