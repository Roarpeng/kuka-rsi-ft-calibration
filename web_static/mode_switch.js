/* 服务模式在线切换（主监控台与调试台共用，免重启）：
 * calibrate = 标定（切标定模式，16 姿态完成后自动回力控）
 * force = 力控（钻孔/凿击由 RobotStatus 决定）
 * 调试台固定输出与模式无关：任何模式下启用即生效并优先于力控。 */
"use strict";

const MODE_NAMES = { calibrate: "标定", force: "力控" };

function renderServiceMode(mode) {
  document.querySelectorAll("[data-service-mode]").forEach(btn => {
    btn.classList.toggle("active", btn.dataset.serviceMode === mode);
  });
}

async function setServiceMode(mode) {
  const hints = {
    force: "切换到力控模式？\n钻孔：按接触力调速 OV_PRO；凿击：恒力 + 横向让位 + 对中。",
    calibrate: "切换到标定模式？\n机器人需配合运行标定程序（16 姿态），完成后自动回力控。",
  };
  if (!confirm(hints[mode] || `切换到 ${mode}？`)) return;
  try {
    const data = await fetchJSON("/api/server_mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode }),
    });
    renderServiceMode(data.mode);
  } catch (err) {
    alert("切换失败：" + err.message);
  }
}

async function pollServiceMode() {
  try {
    const data = await fetchJSON("/api/server_mode");
    renderServiceMode(data.mode);
  } catch { /* 静默，下轮重试 */ }
}

document.querySelectorAll("[data-service-mode]").forEach(btn => {
  btn.addEventListener("click", () => setServiceMode(btn.dataset.serviceMode));
});
pollServiceMode();
setInterval(pollServiceMode, 2000);
