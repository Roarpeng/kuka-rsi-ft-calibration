/* 服务模式在线切换（主监控台与调试台共用，免重启）：
 * monitor = 监控（重力补偿 + 记录）；force = 力控（OV_PRO 恒力等）。
 * 调试台固定输出与模式无关：任何模式下启用即生效并优先于力控。 */
"use strict";

function renderServiceMode(mode) {
  document.querySelectorAll("[data-service-mode]").forEach(btn => {
    btn.classList.toggle("active", btn.dataset.serviceMode === mode);
  });
}

async function setServiceMode(mode) {
  if (mode === "force" && !confirm(
    "切换到力控模式？\n" +
    "机器人进给速度将按接触力自动调节 OV_PRO：空载满速，接近目标力减速，超目标力停止。"
  )) return;
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
    renderServiceMode((await fetchJSON("/api/server_mode")).mode);
  } catch { /* 静默，下轮重试 */ }
}

document.querySelectorAll("[data-service-mode]").forEach(btn => {
  btn.addEventListener("click", () => setServiceMode(btn.dataset.serviceMode));
});
pollServiceMode();
setInterval(pollServiceMode, 2000);
