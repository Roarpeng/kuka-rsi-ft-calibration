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
# 标定模式：采集 6–12 个分散静止姿态后自动求解
python3 udp_server.py --calibrate

# 运行模式：加载标定结果做实时补偿
python3 udp_server.py --run

# 仅记录模式（默认）；若已有标定文件则自动启用补偿
python3 udp_server.py

# 指定网口
python3 udp_server.py --calibrate --ip 192.168.2.10 --port 59152
```

## 项目结构

| 文件 | 说明 |
|------|------|
| `udp_server.py` | UDP 主程序：接收、解析、CSV、模式入口 |
| `calibration_runner.py` | 采样触发、均值样本、求解、运行时补偿 |
| `calibration_math.py` | 旋转/变换、重力模型拟合、补偿计算 |
| `calibration_models.py` | 配置与数据结构 |
| `calibration_io.py` | 配置/结果/样本读写 |
| `ft_calibration_config.json` | 标定与外参配置 |

运行后生成（已加入 `.gitignore`）：

- `ft_calibration.json` — 标定结果
- `ft_calibration_samples.json` — 采样样本
- `rsi_data_*.csv` — 原始记录

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

## 验证建议

1. 空载、无接触：补偿后 TCP 力/力矩接近 0  
2. 接触作业：检查 TCP 力方向与幅值是否符合预期  
3. 若偏差大：检查外参、缩放、欧拉角顺序，或增加更分散的标定姿态  

## License

按项目需要自行补充。
