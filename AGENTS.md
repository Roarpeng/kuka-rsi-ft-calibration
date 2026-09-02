# AGENTS.md — KUKA RSI 六维力传感器标定与恒力钻孔

## 项目概述

基于 KUKA RSI（Robot Sensor Interface，Ethernet/UDP，4ms 周期）的上位机系统，对末端六维力/力矩传感器做**静态多姿态重力补偿标定**，并在运行时输出 TCP 坐标系下的补偿力/力矩，进一步实现**恒力钻孔**（通过回发 `OV_PRO` 控制 `$OV_PRO` 程序倍率）。

- 纯 Python 3.8+，**无第三方依赖**（仅标准库）；`analysis_check*.py` 两个离线诊断脚本额外需要 numpy。
- 机器人侧为 KRL 程序（`*.src`/`*.dat`），上位机与控制器经 UDP 闭环通信。
- 默认 IP：上位机（本机）`192.168.2.250`，机器人 `192.168.2.10`，端口 `59152`（见 `udp_server.py` 的 `RSIConfig` 与 argparse 默认值）。

## 代码组织

| 文件 | 职责 |
|------|------|
| `udp_server.py` | 主程序入口：UDP 收发、RSI XML 解析/回包生成、CSV 记录、四种模式（`--calibrate`/`--run`/`--force`/默认仅记录）、`--test-link` 链路自检 |
| `calibration_models.py` | 全部 dataclass 数据结构：配置（`CalibrationConfig`/`ForceControlConfig` 等）、样本、标定结果 |
| `calibration_io.py` | `ft_calibration_config.json`、标定结果、样本文件的读写 |
| `calibration_math.py` | 纯数学：欧拉角→旋转矩阵、变换合成、最小二乘（高斯消元）、重力模型拟合 `fit_gravity_model`、补偿 `compensate_wrench` |
| `calibration_runner.py` | 采样状态机：`data_collection` TRUE→FALSE 分段取均值成样，满 `min_samples` 自动求解并切到运行时补偿 |
| `force_controller.py` | 恒力控制器 `ForceController`：每 RSI 周期调用一次 `update()`，输出本拍 RKorr 增量与 `ov_pro_pct` |
| `test_force_control.py` | 力控闭环仿真测试（21 项断言） |
| `test_force_e2e.py` | UDP 回环端到端测试（127.0.0.1:59353，真实 `RSIServer` 全链路） |
| `test_auto_calibrate.py` | 自动重标定测试（不起 socket，直驱 `process_frame`）：`data_collection` 上升沿自动进标定、力控挂起/恢复、`--force` 无标定待激活 |
| `rerun_calibration.py` | 用现有样本离线重跑标定求解，与旧结果对比 |
| `data_manager.py` | CSV 数据文件管理：白名单校验、列举、锁定/解锁、删除、容量控制 `enforce_capacity`、历史回放 `read_series`（供 Web 层调用） |
| `web_monitor.py` | Web 监控层（纯标准库 `http.server`）：`start_web_server(server, data_dir, data_cap_mb, port)` 启动 HTTP + SSE 推送，只读监控与文件管理，**不参与控制** |
| `web_static/` | 无构建单页监控台（`index.html`/`app.js`/`style.css`），图表用本地化 `vendor/chart.umd.min.js`（Chart.js 4.4.1，现场无外网，勿删） |
| `test_web_monitor.py` | Web 层测试：data_manager 单测 + Web API 集成（dummy server + http.client） |
| `deploy/` | Ubuntu 生产部署：`kuka-rsi.service`（systemd，开机自启 + `Restart=always`）、`install.sh`（装到 `/opt/kuka-rsi`） |
| `analysis_check.py` / `analysis_check2.py` | 一次性离线诊断脚本（需 numpy），用于历史 bug 排查，非正式测试 |
| `FT_Calibration.src` | KRL：自动走 16 个分散姿态并触发 `RECORD_SAMPLE()` 采样 |
| `FT_Drilling.src` / `FT_Drilling.dat` | KRL：恒力钻孔程序与全局变量（`RobotStatus`、`target_force`） |
| `RSI_Control.src` | KRL：RSI 容器开关，**必须 `RSI_ON(#RELATIVE)`** |
| `RSIEthernet.snippet.xml` | 与现场一致的 RSI SEND/RECEIVE 元素定义片段 |
| `KST_RSI_40_zh.pdf` | KUKA RSI 4.0 中文手册（参考资料） |

## 运行与测试命令

```bash
python udp_server.py                  # 仅记录（已有标定文件时自动启用补偿）
python udp_server.py --calibrate      # 标定模式（机器人侧跑 FT_Calibration.src）
python udp_server.py --run            # 运行模式（实时重力补偿）
python udp_server.py --force          # 力控模式（无 ft_calibration.json 也不退出：力控待激活，自动标定完成后恢复）
python udp_server.py --test-link      # 链路自检：XML 自检 + 回环 + 等真实 RSI 包
python udp_server.py --ip 192.168.2.10 --host-ip 192.168.2.250 --port 59152
```

Web 监控默认随主程序启动（`web_monitor.py`，端口 `--web-port 8080`，`--no-web` 关闭，`--data-dir`/`--data-cap-mb` 控制数据目录与容量上限）。`web_static/vendor/chart.umd.min.js` 是本地化的 Chart.js，属源码需提交。

测试（**改动 `force_controller.py`、`udp_server.py` 或 Web 层后四个都必须通过**）：

```bash
python test_force_control.py   # 闭环仿真，打印 "全部自检通过"
python test_force_e2e.py       # UDP 回环端到端（127.0.0.1:59353），退出码 0
python test_web_monitor.py     # data_manager 单测 + Web API 集成，退出码 0
python test_auto_calibrate.py  # 自动重标定（直驱 process_frame，标定文件写临时目录），退出码 0
```

Ubuntu 生产部署：`sudo bash deploy/install.sh`（装到 `/opt/kuka-rsi`，systemd 服务 `kuka-rsi` 开机自启 + 崩溃自动重启，模式改 unit 里的 `RSI_MODE`）。

无 pytest/unittest 框架；测试是带 `assert` 的可执行脚本，直接 `python` 运行。Windows 控制台输出中文可能是 GBK 乱码，属正常，以退出码和 "OK/通过" 判断。

## 运行时产物（已 gitignore，勿提交）

`ft_calibration.json`（标定结果）、`ft_calibration_samples.json`（样本）、`ft_calibration_fixed.json`、`rsi_data_*.csv`（含 `data/` 目录及其 `.lock` sidecar）、`pktmon_*.txt`。`__pycache__/`、`.graphflow-cache/`、`graphflow-out/` 也已忽略。

## 代码约定

- 中文注释与文档；日志/打印均为中文，带 `[标定]`、`[力控]` 等前缀。
- 全文件 `from __future__ import annotations`；dataclass 建模；类型标注齐全。
- 纯标准库，手写线性代数（`calibration_math.py`），**不引入 numpy 到运行时路径**。
- 单位约定：位置 mm（RSI）/ m（内部标定），力 N，力矩 N·m，角度 deg；姿态欧拉角顺序 `ZYX`（`rsi_rotation_order` 配置项）。
- 原始力值换算物理量默认 `force_scales = torque_scales = 0.001`。

## RSI 协议要点（改 SEND/RECEIVE 时必须同步三处）

`udp_server.py` 的 `SEND_ELEMENTS`/`RECEIVE_ELEMENTS`、机器人侧 `RSIEthernet.xml`（见 `RSIEthernet.snippet.xml`）、README 的字段表，三者字段、顺序、类型必须一致。

- SEND（机器人→上位机）：`Fx_raw~Mz_raw`(LONG 1–6)、`Act_X~C`(DOUBLE 7–12)、`data_collection`(BOOL 13)、`RobotStatus`(BOOL 14，**必须是 BOOL 不要用 INT**)。
- RECEIVE（上位机→机器人）：`RKorr.X~C`(DOUBLE 1–6，HOLDON=0)、`OV_PRO`(DOUBLE 7，Ethernet Out7→Map2OV_PRO，量程 0–100%)。
- 回包 `IPOC` 必须与收包一致，否则包无效；`RKorr.X` 写成属性形式 `<RKorr X="..." />`。

## 安全关键约束（力控，源自抖动事故复盘，勿随意改）

- **RKorr 是每拍增量**（KRL 必须 `RSI_ON(#RELATIVE)`；默认 `#ABSOLUTE` 会把增量当绝对偏移）。发 0 = 保持当前叠加，不是归零。
- PosCorr 的 Lower/UpperLim 限制**总修正**（须 ±80mm 行程），单拍由 `per_cycle_max_mm=0.08` 钳；`PosCorrMon.MaxTrans=80` 监总半径；`cumulative_max_mm` 必须 ≤ 机器人侧限值。
- **RefCorrSys=Tool**：RKorr 增量就是工具系毫米，禁止旋到基座（按基座发会把退刀变加压，正反馈）。
- 方向约定：进给 = 工具 +X（`press_motion_sign=+1`），压紧时传感器该轴读数为负（`press_sign=-1`）。
- `contact_threshold_n=20` 高于空载残差（5~12N 漂移）才不误判接触。
- 收包中断 >1s 判定 RSI 重启，`udp_server.py` 自动 `ForceController.reset()`（机器人叠加归零，PC 累积须同步清零）。
- 关 RSI 前必须先退刀让 `OV_PRO` 回到 100%，避免 `$OV_PRO` 停在 0；`RobotStatus=TRUE` 是凿击占位，不是"结束钻孔"。
- 超力保护 `max_force_n=150` 用原始力锁存，过零不立即解锁（须连续卸荷 `trip_clear_s`）。

## 标定流程与模型

1. 确认 `ft_calibration_config.json` 外参（`sensor_to_flange`、`flange_to_tcp`）与缩放正确。
2. `--calibrate` + 机器人跑 `FT_Calibration.src`（16 姿态）；`data_collection` TRUE→FALSE 一段生成一条样本（最短 `min_dwell_seconds=0.4s`，姿态间隔 ≥ `min_pose_separation_deg=12°`）。
3. 满 `min_samples=16` 自动求解保存 `ft_calibration.json` 并切到补偿模式。
   - **自动重标定（换工具）**：任何模式下 `data_collection` 的 FALSE→TRUE 上升沿都会自动进入标定（清旧样本、切 `calibration_collect`，见 `udp_server.py` `_check_auto_calibration`）；力控中则挂起力控（`reset()`、RKorr=0、`OV_PRO`=100%）。求解完成后若服务带 `--force` 自动恢复力控（`_check_force_restore`）。注意必须用**上升沿**触发，否则求解完成当拍残留的 TRUE 会立刻重新触发一轮。
   - `--force` 无 `ft_calibration.json` 不再退出：`force_suspended=True` 待激活（RKorr=0、`OV_PRO`=100%），等自动标定完成后激活。
4. 求解模型（`calibration_math.py` `fit_gravity_model`）：物理模型仅用于估计 `mass_kg`；**实际补偿用自由 3x3 `gravity_matrix_n`**（吸收安装偏角与增益误差，不依赖质量估计）。力矩拟合 `tau = bias + r_com × (S·d)`。
5. 验证：空载无接触时补偿后 TCP 力/力矩应接近 0（残差 5~12N 属正常漂移）。

## 已知未完成项

- `RobotStatus=TRUE`（凿击）位移控制是占位：只回默认倍率 + RKorr=0。
- `analysis_check*.py` 为一次性历史诊断脚本，不是回归测试。
