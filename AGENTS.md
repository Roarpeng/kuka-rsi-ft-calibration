# AGENTS.md — KUKA RSI 六维力传感器标定与恒力钻孔

## 项目概述

基于 KUKA RSI（Robot Sensor Interface，Ethernet/UDP，4ms 周期）的上位机系统，对末端六维力/力矩传感器做**静态多姿态重力补偿标定**，并在运行时输出 TCP 坐标系下的补偿力/力矩，进一步实现**恒力钻孔**（通过回发 `OV_PRO` 控制 `$OV_PRO` 程序倍率）。

- 纯 Python 3.8+，**无第三方依赖**（仅标准库）；`analysis_check*.py` 两个离线诊断脚本额外需要 numpy。
- 机器人侧为 KRL 程序（`*.src`/`*.dat`），上位机与控制器经 UDP 闭环通信。
- 默认 IP：上位机（本机）`192.168.2.250`，机器人 `192.168.2.10`，端口 `59152`（见 `udp_server.py` 的 `RSIConfig` 与 argparse 默认值）。

## 代码组织

| 文件 | 职责 |
|------|------|
| `udp_server.py` | 主程序入口：UDP 收发、RSI XML 解析/回包生成、CSV 记录、四种启动模式（`--calibrate`/`--run`/`--force`/默认仅记录）、`--test-link` 链路自检、调试固定输出 `debug_override`（优先于力控）、运行时模式切换 `set_service_mode`（monitor/force，Web 在线切免重启） |
| `calibration_models.py` | 全部 dataclass 数据结构：配置（`CalibrationConfig`/`ForceControlConfig` 等）、样本、标定结果 |
| `calibration_io.py` | `ft_calibration_config.json`、标定结果、样本文件的读写 |
| `calibration_math.py` | 纯数学：欧拉角→旋转矩阵、变换合成、最小二乘（高斯消元）、重力模型拟合 `fit_gravity_model`、补偿 `compensate_wrench` |
| `calibration_runner.py` | 采样状态机：`data_collection` TRUE→FALSE 分段取均值成样，满 `min_samples` 自动求解并切到运行时补偿 |
| `force_controller.py` | 恒力控制器 `ForceController`：每 RSI 周期调用一次 `update()`，输出本拍 RKorr 增量与 `ov_pro_pct`；钻孔（drill）OV_PRO 恒力、凿击（chisel）Y/Z 横向零力让位 + X 恒力冻结、overlay 为 RKorr 位置外环测试路径 |
| `test_force_control.py` | 力控闭环仿真测试（28 项断言，含凿击横向让位 7 项） |
| `test_force_e2e.py` | UDP 回环端到端测试（127.0.0.1:59353，真实 `RSIServer` 全链路） |
| `test_auto_calibrate.py` | 自动重标定测试（不起 socket，直驱 `process_frame`）：`data_collection` 上升沿自动进标定、力控挂起/恢复、`--force` 无标定待激活 |
| `rerun_calibration.py` | 用现有样本离线重跑标定求解，与旧结果对比 |
| `data_manager.py` | CSV 数据文件管理：白名单校验、列举、锁定/解锁、删除、容量控制 `enforce_capacity`、历史回放 `read_series`（供 Web 层调用） |
| `web_monitor.py` | Web 监控层（纯标准库 `http.server`）：`start_web_server(server, data_dir, data_cap_mb, port, force_config_path)` 启动 HTTP + SSE 推送，只读监控与文件管理 + `/api/force_config` 力控参数在线设定（白名单+范围校验、实时生效并写回 `ft_calibration_config.json`）+ `/api/debug_override` 调试固定输出设定 + `/api/server_mode` 服务模式在线切换，**不直接参与控制环** |
| `web_static/` | 无构建单页监控台（`index.html`/`app.js`/`style.css` + 调试台 `debug.html`/`debug.js`），图表用本地化 `vendor/chart.umd.min.js`（Chart.js 4.4.1，现场无外网，勿删） |
| `test_web_monitor.py` | Web 层测试：data_manager 单测 + Web API 集成（dummy server + http.client） |
| `deploy/` | Ubuntu 生产部署：`kuka-rsi.service`（systemd，开机自启 + `Restart=always`）、`install.sh`（装到 `/opt/kuka-rsi`） |
| `analysis_check.py` / `analysis_check2.py` | 一次性离线诊断脚本（需 numpy），用于历史 bug 排查，非正式测试 |
| `KUKA_SRC/` | KRL 程序集（2026-10-09 重示教点位）：FT_Calibration/FT_Drilling/FT_Chisel 的 SRC/DAT 成对文件 |

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

命令行参数只决定**启动默认**；监控/力控可在 Web 页面顶栏的"监控/力控"分段开关**运行时切换**（`POST /api/server_mode`），无需重启服务。切换时会停用调试固定输出、复位力控器、RKorr 清零、OV_PRO 回 100%。

Web 监控默认随主程序启动（`web_monitor.py`，端口 `--web-port 8080`，`--no-web` 关闭，`--data-dir`/`--data-cap-mb` 控制数据目录与容量上限）。`web_static/vendor/chart.umd.min.js` 是本地化的 Chart.js，属源码需提交。监控台"力控参数设定"卡片（`GET/POST /api/force_config`）可在线改凿击横向让位阈值/行程上限/卡滞阈值/方向符号、轴线对中开关与参数、默认目标力，实时生效并持久化。`/debug.html` 调试台（`GET/POST /api/debug_override`）可固定下发 RKorr 每拍增量 + OV_PRO 并监控 Act 位姿，用于实机验证方向约定（如 `chisel_lateral_sign`/`align_sign` 手推标定前的方向确认）。

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
- 方向约定：进给 = 钻轴。**2026-10-09 TCP 重标定后钻轴 = 工具 Z**（`force_control.axis="Z"`，轴向推力实测落 Fz、压紧读负）；压紧时该轴读数为负（`press_sign=-1`）。横向让位/对中通道**自动跟随钻轴推导**（axis=Z 时横向走 X/Y、对中 Mx→A/My→B）。旧 KRL 点位按"TCP=法兰"示教，重标定后**必须重示教**（已重示教完毕）。
- **工具系手型约定（Web 可配置：`frame_convention` 段 + `/api/frame_convention`，默认右手、拇指X/食指Y/中指Z=钻轴）**：服务端校验排列与手性自洽（右手须 拇指×食指=中指）。监控台据当前约定+当前钻轴自动生成"手扳验证参考表"；**此约定只做方向推演，控制律符号由 chisel_lateral_sign / align_sign 独立参数决定**。**用户示教器/口头标签 = 法兰系**（TCP 重标定前旧轴），映射：用户 X = 钻轴 = 工具 Z 通道，用户 Y ≈ 工具 X，用户 Z ≈ 工具 Y——沟通方向时先对齐标签再谈符号。
- **本机传感器符号（2026-10-08 手扳标定 + 2026-10-09 三方向 40/50/30N 推力复核）**：法兰系下 Y/Z 通道**力与力矩整体反号**（+Y 扳动实测 Fy=−78/Mz=−6.8，+Z 扳动实测 Fz=−287/My=+19.6，等效传感器绕 X 转 180°；X 通道不受影响，故 X 轴力控一直正常，重力矩阵已吸收此旋转）。因此 **`chisel_lateral_sign=-1`、`align_sign=-1`**（已写入配置）。勿改 `sensor_to_flange.rotation_deg` 来"纠正"——现有标定是在当前配置下拟合的，改变换须重标定。另：空载 My 残差实测 +2.4 N·m，**对中死区须高于该残差**（暂设 ≥3 N·m 或先排查残差）再开对中。
- `contact_threshold_n=20` 高于空载残差（5~12N 漂移）才不误判接触。
- 收包中断 >1s 判定 RSI 重启，`udp_server.py` 自动 `ForceController.reset()`（机器人叠加归零，PC 累积须同步清零）。
- 关 RSI 前必须先退刀让 `OV_PRO` 回到 100%，避免 `$OV_PRO` 停在 0；`RobotStatus=TRUE` 是凿击模式，不是"结束钻孔"。
- 超力保护 `max_force_n=150` 用滤波通道锁存（EMI 尖峰 1~3 帧可达 ±150N 级，原始值会连续误触发、实测一轮 62/18 次），过零不立即解锁（须连续卸荷 `trip_clear_s`）。调速与停止判断同样用滤波通道（中值+低通，实测均值 30N 时原始值判据 OV 有 98% 时间为 0）。
- **凿击（`RobotStatus=TRUE`）横向让位**：Y/Z 对持续横向力做零力让位（慢通道滤波 + 死区 + 比例漂移）。让位方向 = `chisel_lateral_sign` × 读数方向（读数=工件对工具作用力，同号让位即背离障碍物卸载）；**符号位必须实机手推批头实测后才能改**，搞反即横向正反馈（顶墙）。死区 `chisel_lateral_deadband_n=15N` 必须高于空载残差 5~12N，否则空载慢漂。单轴让位上限 `chisel_lateral_max_mm=20mm`（Web 可设），内部仍受 80mm PosCorrMon 总限。横向原始力超 `chisel_lateral_trip_n=100N` 判卡滞：沿 -X 全速退刀并闩锁（复用超力解锁规则）。横向让位期间 X 轴恒力（OV_PRO）冻结、横向撤销后恢复；切回钻孔时横向叠加按每拍限幅缓撤到 0。
- **调试台固定输出（`/debug.html` → `server.debug_override`）**：会真实移动机械臂，仅限联调。平移限幅 = `per_cycle_max_mm`（±0.08mm/拍）、旋转 ±0.05°/拍、OV_PRO 0–100，Web 层与服务端双重校验。优先于力控；**RSI 收包中断 >1s 或 `data_collection` 上升沿（自动标定）会自动停用**（RKorr=0、OV_PRO=100%、力控器复位）。停止后发 0 只保持叠加不撤销偏移（#RELATIVE 语义）。
- **轴线零力矩对中（`align_*` 参数，通道跟随钻轴：axis=Z 时 Mx→A、My→B）**：凿击叠加在平移让位之上（`align_chisel_enable`），钻孔单独可开（`align_drill_enable`）。几何依据（实测 13:46 B 轴验证）：绕 TCP 旋转不平移 TCP，与平移让位正交；旋转与 OV_PRO 速度环也正交。**实测（2026-10-09）：A/B 双轴 ±5° 全幅执行 ✓、力矩被压降 ✓；参数迭代结论——死区按作业力矩实测×余量设（当前 3 N·m）、保护 30 N·m、累计 ±5°。旋转每拍限幅不能低于 ~0.005°（A 通道实测 0.005°/拍时机器人侧几乎不执行，Act 仅动 0.08°；0.02°/拍全幅执行），推荐 0.01~0.02°/拍 + 低增益控速**。长臂（TCP 距传感器 0.66m）下 5°/s 端部线速度 57mm/s 会激振，增益已降至 0.05。关闭/退出时旋转叠加按每拍限幅缓撤到 0。

## 标定流程与模型

1. 确认 `ft_calibration_config.json` 外参（`sensor_to_flange`、`flange_to_tcp`）与缩放正确。
2. `--calibrate` + 机器人跑 `FT_Calibration.src`（16 姿态）；`data_collection` TRUE→FALSE 一段生成一条样本（最短 `min_dwell_seconds=0.4s`，姿态间隔 ≥ `min_pose_separation_deg=12°`）。
3. 满 `min_samples=16` 自动求解保存 `ft_calibration.json` 并切到补偿模式。
   - **自动重标定（换工具）**：任何模式下 `data_collection` 的 FALSE→TRUE 上升沿都会自动进入标定（清旧样本、切 `calibration_collect`，见 `udp_server.py` `_check_auto_calibration`）；力控中则挂起力控（`reset()`、RKorr=0、`OV_PRO`=100%）。求解完成后若服务带 `--force` 自动恢复力控（`_check_force_restore`）。注意必须用**上升沿**触发，否则求解完成当拍残留的 TRUE 会立刻重新触发一轮。
   - `--force` 无 `ft_calibration.json` 不再退出：`force_suspended=True` 待激活（RKorr=0、`OV_PRO`=100%），等自动标定完成后激活。
4. 求解模型（`calibration_math.py` `fit_gravity_model`）：物理模型仅用于估计 `mass_kg`；**实际补偿用自由 3x3 `gravity_matrix_n`**（吸收安装偏角与增益误差，不依赖质量估计）。力矩拟合 `tau = bias + r_com × (S·d)`。
5. 验证：空载无接触时补偿后 TCP 力/力矩应接近 0（残差 5~12N 属正常漂移）。

## 已知未完成项

- 凿击（`FT_Chisel.src`，KUKA_SRC/ 目录）待实机联调；横向让位增益当前 0.01（现场调低，默认 0.04）。
- 对中 A 通道每拍限幅需回调 0.01~0.015° 再测（0.005 时机器人不执行）。
- 力矩重力矩阵后 RMS 地板 ~3.5 N·m（角度读数精度限制）；作业姿态偏置实测 <0.5。
- `analysis_check*.py` 为一次性历史诊断脚本，不是回归测试。
- `record_debug_session.py` 为实机联调录制器（临时工具，200ms 轮询 /api/status）。
