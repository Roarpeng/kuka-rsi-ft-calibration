# KUKA RSI 六维力传感器重力补偿标定

基于 KUKA RSI UDP 数据流，对末端工具装载后的六维力/力矩做**静态多姿态重力补偿标定**，并在运行时输出 TCP 坐标系下的补偿结果。

## 功能

- 接收 RSI XML 数据包（位姿 + 六维力原始值 + `data_collection` 采样触发）
- 原始力/力矩按比例换算为物理量（默认 `/1000`）
- 由 `data_collection` 控制采样段，对区间取均值生成标定样本
- 运行时重力补偿，输出 TCP 坐标系 6 维力/力矩
- CSV 持续记录（原始值、换算值、补偿值、采样状态）

## 环境要求

- Python 3.8+
- 标准库即可运行（无第三方依赖）
- 与 KUKA 控制器处于同一网段，可接收 RSI UDP 数据

## 快速开始

```bash
# 标定模式：FT_Calibration.src 自动走 16 个分散静止姿态后自动求解
python3 udp_server.py --calibrate

# 运行模式：加载标定结果做实时补偿
python3 udp_server.py --run

# 力控模式（恒力钻孔）：补偿 + 恒力控制
# RobotStatus=FALSE 钻孔：回发 OV_PRO 改 $OV_PRO；TRUE 凿击：X 恒力 + Y/Z 横向让位。
# 机器人侧运行 FT_Drilling.src；RobotStatus 由全局变量下发
# 无 ft_calibration.json 时不再退出：力控挂起待激活，标定完成后自动恢复
python3 udp_server.py --force

# 仅记录模式（默认）；若已有标定文件则自动启用补偿
python3 udp_server.py

# 指定网口
python3 udp_server.py --calibrate --ip 192.168.2.10 --port 59152
```

## Web 监控

主程序启动时默认附带 Web 监控（局域网开放，无登录，只读监控不参与控制）：

```bash
python3 udp_server.py --force --web-port 8080      # 默认 8080 端口
python3 udp_server.py --force --no-web             # 关闭 Web
python3 udp_server.py --data-dir data --data-cap-mb 2048   # 数据目录与容量上限
```

浏览器打开 `http://<本机IP>:8080`：

- **实时**：TCP 六轴力/力矩曲线、OV_PRO、连接状态、标定进度、力控状态（SSE 推送）
- **模式在线切换**：顶栏"监控 / 力控"分段开关运行时切换服务模式，无需重启（监控=重力补偿+记录；力控=OV_PRO 恒力调节，需已有标定结果）
- **告警**：断线、超力保护触发等在页面顶部横幅提示，事件流记录全部事件
- **历史回放**：任选 CSV 查看曲线，支持时间段缩放查询
- **数据管理**：CSV 下载/删除/锁定；超出 `--data-cap-mb` 自动删除最旧的未锁定文件，锁定文件永不自动清理
- **力控参数设定**：在线改凿击横向让位阈值/行程上限/卡滞阈值/方向符号、轴线对中（B←My/C←Mz）开关与参数、默认目标力（白名单校验，实时生效并写回 `ft_calibration_config.json`）
- **工具系手型约定**：左右手与三指 XYZ 分配可配置，自动生成手扳验证参考表（推哪根手指方向 → F/M 预期符号、B/C 正向偏转方向）
- **调试台**（`/debug.html`）：固定下发 RKorr 每拍增量（XYZ 平移 ±0.08mm/拍、ABC 旋转 ±0.05°/拍）与 OV_PRO，实时监控 Act 位姿回读与自启用以来位移，用于实机验证方向约定与链路；链路中断 >1s 或触发标定信号会自动停止固定输出

## Ubuntu 生产部署

```bash
sudo bash deploy/install.sh
```

安装到 `/opt/kuka-rsi` 并注册 systemd 服务 `kuka-rsi`（开机自启、崩溃 3s 自动重启）。
切换运行模式（标定/运行/力控）：编辑 `/etc/systemd/system/kuka-rsi.service` 中的
`RSI_MODE` 后 `systemctl daemon-reload && systemctl restart kuka-rsi`。

```bash
systemctl status kuka-rsi        # 状态
journalctl -u kuka-rsi -f        # 日志
```

注意：本机网口需配置静态 IP `192.168.2.250`（机器人 RSI XML 的 `IP_NUMBER`）。

## 项目结构

| 文件 | 说明 |
|------|------|
| `udp_server.py` | UDP 主程序：接收、解析、CSV、模式入口 |
| `calibration_runner.py` | 采样触发、均值样本、求解、运行时补偿 |
| `calibration_math.py` | 旋转/变换、重力模型拟合、补偿计算 |
| `force_controller.py` | 恒力控制：钻孔用 `$OV_PRO` 控进给；凿击 X 恒力 + Y/Z 横向零力让位（滑坑/卡滞卸载） |
| `FT_Drilling.src/.dat` | 恒力钻孔程序（`RobotStatus`、`target_force` 全局变量） |
| `RSIEthernet.snippet.xml` | 与现场一致的 SEND `RobotStatus` / RECEIVE `OV_PRO` 片段 |
| `calibration_models.py` | 配置与数据结构 |
| `calibration_io.py` | 配置/结果/样本读写 |
| `data_manager.py` | CSV 数据管理：锁定、删除、容量滚动清理、历史回放抽稀 |
| `web_monitor.py` | Web 监控服务（纯标准库 HTTP + SSE），只读监控 + 力控参数在线设定 |
| `web_static/` | 监控单页前端（Chart.js 已本地化，离线可用） |
| `deploy/` | Ubuntu systemd 部署（`kuka-rsi.service`、`install.sh`） |
| `ft_calibration_config.json` | 标定与外参配置 |

运行后生成（已加入 `.gitignore`）：

- `ft_calibration.json` — 标定结果
- `ft_calibration_samples.json` — 采样样本
- `data/rsi_data_*.csv` — 原始记录（`.lock` sidecar 表示锁定）

## 标定流程

1. 确认 `ft_calibration_config.json` 中传感器外参与缩放正确  
   - `sensor_to_flange`、`flange_to_tcp`  
   - `force_scales` / `torque_scales`（默认 `0.001`）
2. 启动：`python3 udp_server.py --calibrate`
3. 遥控到 **6–9 个分散姿态**（不够可加到 12）  
   - 到位后将 RSI 的 `data_collection` 置 `TRUE` 约 0.5s，再置 `FALSE`  
   - 每个 TRUE→FALSE 周期对位姿与力取均值，生成一条样本
4. 达到最少样本数（默认 6）后自动求解并保存 `ft_calibration.json`，切换到 TCP 补偿
5. 用 `--run` 验证：空载无接触时力/力矩应接近 0

### 换工具重新标定（自动，服务无需重启/改模式）

服务常驻 `--force` 运行时，换工具后只需在示教器运行 `FT_Calibration.src`：

1. 上位机收到 `data_collection` 的 FALSE→TRUE 上升沿即自动进入标定（任何模式下）：
   清空旧样本、切换到采集模式；若在力控中则自动挂起力控（RKorr=0、`OV_PRO`=100%）。
2. 标定程序走完分散姿态、样本数达标后自动求解并保存 `ft_calibration.json`。
3. 求解完成后自动恢复力控（无力控器则新建）；非 `--force` 启动则只完成标定。

因此 `--force` 启动时即使没有 `ft_calibration.json` 也不会退出，力控处于
"待激活"状态（RKorr=0、`OV_PRO`=100%），等自动标定完成后自动激活。

## 配置要点

`ft_calibration_config.json` 关键项：

| 项 | 含义 |
|----|------|
| `rsi_rotation_order` | 姿态欧拉角顺序，当前为 `ZYX` |
| `sensor_to_flange` | 传感器相对法兰的平移(m)与旋转(deg) |
| `flange_to_tcp` | 法兰相对 TCP 的平移(m)与旋转(deg) |
| `scale` | 原始值缩放与力矩单位（`N_m`） |
| `static_detection` | 最短采集时长、姿态间隔、最少/最多样本数等 |

位姿约定：RSI 的 `Act_X/Y/Z` 为 TCP 位置（mm），`Act_A/B/C` 为 TCP 姿态角（度）。

## KUKA RSI 侧要求

### 数据顺序（须与 Python 端一致）

| 序号 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 1–6 | Fx_raw ~ Mz_raw | LONG | 六维力原始值 |
| 7–9 | Act_X ~ Act_Z | DOUBLE | TCP 位置 (mm) |
| 10–12 | Act_A ~ Act_C | DOUBLE | TCP 姿态 (deg) |
| 13 | data_collection | BOOL | 标定采样触发 |
| 14 | RobotStatus | BOOL | FALSE=钻孔（控倍率）；TRUE=凿击（X 恒力 + Y/Z 横向让位） |

RECEIVE（上位机 → 机器人）：

| 序号 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 1–6 | RKorr.X ~ C | DOUBLE | 位置修正（#RELATIVE 每拍增量）；HOLDON=0 |
| 7 | OV_PRO | DOUBLE | `$OV_PRO` 0–100%，Ethernet Out7 → Map2OV_PRO |

### data_collection 信号

```xml
<data_collection>
  <Tag>your_data_collection_signal</Tag>
  <Type>BOOL</Type>
  <Index>13</Index>
</data_collection>
```

- `data_collection = TRUE`：开始累积当前姿态的位姿与力数据  
- `data_collection = FALSE`：结束本段，对区间取均值生成一条标定样本  

建议每姿态保持 TRUE 约 0.5s；过短（默认 < 0.4s）的段会被丢弃。

### RobotStatus 与 $OV_PRO

`Map2OV_PRO` 改的是程序倍率（0–100%），不是 mm/s。实际路径速度 = `$VEL.CP × OV_PRO/100`。

- `RobotStatus = FALSE`（钻孔）：按接触力映射倍率（空载接近 100%；到目标力或超力发 0，LIN 真正停住）。RKorr 为 0。
- `RobotStatus = TRUE`（凿击）：X 仍按 OV_PRO 恒力压紧；Y/Z 横向力超过阈值（默认 15N）时零力让位卸载（滑坑/卡滞防护，最多让位 20mm，横向卡滞 >100N 沿 -X 全速退刀）。横向让位期间 X 恒力冻结，横向撤销后恢复；切回钻孔时横向叠加自动缓撤到 0。参数可在 Web 监控台在线设定。
- RSIVisual：Ethernet **Out7** → Map2OV_PRO，输入量程必须是 **0～100**。
- 关 RSI 前先退刀让接触力下降，倍率回到 100%，避免 `$OV_PRO` 停在 0。不要用 TRUE 当作“结束钻孔”（TRUE 表示凿击）。
- 现场 XML 片段见 `RSIEthernet.snippet.xml`。
- RSIEthernet SEND 中 `RobotStatus` 必须是 **BOOL**（不要再用 INT）。

## 验证建议

1. 空载、无接触：补偿后 TCP 力/力矩接近 0  
2. 接触作业：检查 TCP 力方向与幅值是否符合预期  
3. 若偏差大：检查外参、缩放、欧拉角顺序，或增加更分散的标定姿态  

## 恒力钻孔力控要点（2026-08-31 抖动事故复盘后修正）

曾出现钻孔时疯狂抖动，根因有二（数据实锤，见 `rsi_data_20260831_131835.csv`）：

1. **RKorr 必须是每拍增量**：KRL 使用 `RSI_ON(#RELATIVE)`。POSCORR 把每包
   `RKorr` 叠加到当前修正上；发 `0` 表示保持。**PosCorr 的 Lower/UpperLim 限制的是
   该对象上的总修正**（超了报 xmax），不是单拍；单拍由 PC `per_cycle_max_mm` 钳。
   PosCorrMon.MaxTrans 再监笛卡尔总半径。目标力 <=0 时按每拍限幅反向撤除叠加。
2. **压紧运动方向**：PosCorr 为工具系。进给 = 工具 **+X**（正增量），退刀 = 工具 **−X**（负增量）。`press_motion_sign=+1`。压紧时传感器该轴读数为负（`press_sign=-1`），与运动方向独立。

其他要点：

- RSI 重启后机器人叠加归零：`udp_server.py` 检测收包中断 >1s 自动 `reset()`
- 空载重力补偿残差约 5~12N 且有漂移，`contact_threshold_n` 取 20N 才不误判接触
- `per_cycle_max_mm=0.08` 是 PC 单拍帽；PosCorr X 须为 **±80**（总行程），PosCorrMon.MaxTrans=80
- 不要用默认 `RSI_ON()`（`#ABSOLUTE`）：那会把 0.2 mm 增量当成 0.2 mm 总偏移
- 验证手段：`test_force_control.py`（闭环仿真）、`test_force_e2e.py`（UDP 回环
  端到端）；改动力控后两者都必须通过

## License

按项目需要自行补充。
