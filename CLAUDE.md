# 当前工作总结

## 项目目标

当前项目的目标是接收 KUKA RSI 通过 UDP 发送的机器人位姿和六维力传感器数据，并实现末端工具装载后的静态重力补偿标定。

标定后的目标是：
- 在机器人装有末端工具但未接触工件时，输出力和力矩尽量接近 0。
- 在实际作业接触时，输出补偿后的 TCP 坐标系下的 6 维力和力矩。

## 已确认的工程条件

### 数据含义
- RSI 发来的 `Act_X Act_Y Act_Z Act_A Act_B Act_C` 表示 TCP 位姿。
- 姿态角按 `ZYX` 顺序解释。
- 当前姿态数据按标准欧拉角使用。

### 采集与标定方式
- 标定采用多个静止姿态，不采用连续运动标定。
- 每个姿态通过 RSI 布尔信号 `data_collection` 触发采样。
- `data_collection=TRUE` 时累积帧；变为 `FALSE` 时对本段取均值生成一条样本。
- 建议每姿态保持 TRUE 约 0.5s（最短采集时长默认 0.4s）。
- 姿态数量默认最少 6、最多 12；不够稳可增加到 12。
- 输出坐标系为 TCP；暂不做运动惯性补偿。

### 力传感器条件
- 力传感器输出当前仍是原始值。
- 原始值除以 1000 后得到标准物理量。
- 力和力矩通道当前都按 `1/1000` 比例缩放处理。
- 力矩单位按 `N·m` 处理。
- 传感器安装关系当前已知到 `sensor -> flange`。
- 运行时输出目标坐标系是 TCP 坐标系。

## 已完成的代码改动

### 1. 扩展 UDP 主程序
已修改文件：`udp_server.py`

当前改动包括：
- 保留原有 UDP 接收与 XML 解析流程。
- 增加标定配置加载。
- 增加标定结果加载。
- 增加每帧数据统一处理入口。
- 增加换算后的传感器力数据输出。
- 增加补偿后的 TCP 力数据输出。
- 增加 `sample_status` 标记当前采样状态。
- 扩展 CSV 输出字段，保存原始值、换算值、补偿值和状态。

### 2. 新增标定数据模型
已新增文件：`calibration_models.py`

当前文件负责：
- 定义标定配置结构。
- 定义静止检测配置。
- 定义标定样本结构。
- 定义标定结果结构。
- 定义补偿后的 wrench 数据结构。

### 3. 新增标定数学模块
已新增文件：`calibration_math.py`

当前文件负责：
- 原始值按比例系数换算成物理量。
- 欧拉角转旋转矩阵。
- 组合外参变换。
- 静态重力模型拟合。
- 最小二乘求解。
- 运行时重力补偿。
- 将补偿结果转换到 TCP 坐标系。

### 4. 新增配置与结果读写模块
已新增文件：`calibration_io.py`

当前文件负责：
- 读取本地标定配置。
- 读取本地标定结果。
- 保存标定结果。
- 保存采样样本。

### 5. 新增采样与标定执行模块
已新增文件：`calibration_runner.py`

当前文件负责：
- 连续接收每帧数据。
- 按 `data_collection` 边沿累积/结束采样段。
- 对采样段取均值生成样本。
- 判断姿态是否重复。
- 达到最少样本数后自动求解标定。
- 保存样本和标定结果。
- 在运行模式下做实时 TCP 补偿。

## 已创建的配置文件

已新增文件：`ft_calibration_config.json`

当前配置中已经明确：
- `rsi_rotation_order = "ZYX"`
- `force_scales = [0.001, 0.001, 0.001]`
- `torque_scales = [0.001, 0.001, 0.001]`
- 力矩单位为 `N_m`
- `sensor_to_flange`: 平移 [0.0, 0.0, 0.035] 米，旋转 [0, 0, 0] 度
- `flange_to_tcp`: 平移 [0.0, 0.150, 0.230] 米，旋转 [0, 0, 0] 度

## 命令行用法

程序现在支持三种运行模式：

### 标定模式
```bash
python3 udp_server.py --calibrate
```
- 由 `data_collection` 触发采样（TRUE 采集，FALSE 结束并取均值）
- 达到最少样本数（默认 6 个）后自动求解标定参数
- 标定结果保存到 `ft_calibration.json`，并切换到 TCP 补偿

### 运行模式
```bash
python3 udp_server.py --run
```
- 使用已有标定结果进行实时重力补偿
- 输出补偿后的 TCP 坐标系 6 维力/力矩

### 仅记录模式（默认）
```bash
python3 udp_server.py
```
- 记录原始数据，不做标定或补偿
- 如果检测到已有标定结果，自动启用补偿

### 其他参数
```bash
python3 udp_server.py --calibrate --ip 192.168.2.10 --port 59152
```
- `--ip`: RSI 数据源 IP 地址（默认：192.168.2.10）
- `--port`: UDP 监听端口（默认：59152）

## KUKA 侧 RSI 配置

### 采样触发信号 `data_collection`

需要在 KUKA RSI 配置中添加 `data_collection` 布尔信号，用于手动/程序触发标定采样。

**RSI 对象配置示例：**
```xml
<data_collection>
  <Tag>your_data_collection_signal</Tag>
  <Type>BOOL</Type>
  <Index>13</Index>
</data_collection>
```

**KUKA 程序逻辑：**
- 机器人运动到目标姿态后，将 `data_collection` 置为 `TRUE` 约 0.5s
- 采集结束后置回 `FALSE`，再运动到下一姿态

**数据处理逻辑：**
- `data_collection = TRUE`：累积位姿与力数据
- `data_collection` 由 TRUE→FALSE：对本段取均值，生成一条标定样本

### 数据发送顺序

RSI 发送的数据元素顺序必须与 Python 端一致：

| 序号 | 字段名 | 类型 | 说明 |
|------|--------|------|------|
| 1-6 | Fx_raw ~ Mz_raw | LONG | 六维力传感器原始值 |
| 7-9 | Act_X ~ Act_Z | DOUBLE | TCP 位置 (mm) |
| 10-12 | Act_A ~ Act_C | DOUBLE | TCP 姿态角 (度) |
| 13 | data_collection | BOOL | 标定采样触发 |

## 当前程序能力边界

当前程序已经完成基础可运行框架，并支持命令行模式选择与 `data_collection` 采样交互。

当前已具备：
- UDP 实时接收。
- RSI XML 解析（含 BOOL `data_collection`）。
- CSV 持续记录。
- 原始值缩放。
- `data_collection` 边沿采样与均值。
- 标定结果求解。
- 运行时 TCP 补偿。
- 命令行模式选择（--calibrate / --run / 默认）。

当前还需要现场确认：
- RSI XML 标签名确为 `data_collection`（与 Python 端一致）。
- 用在线数据做验证。
- 检查补偿后在空载无接触时是否接近 0。
- 检查接触时 TCP 坐标系输出方向是否符合预期。

## 已完成检查

当前所有 Python 文件已经通过 `py_compile` 语法检查。

## 下一步建议

推荐按下面顺序继续：

1. 启动标定模式：`python3 udp_server.py --calibrate`
2. 遥控到 6–9 个不同姿态；每姿态 `data_collection` TRUE 约 0.5s 再 FALSE。
3. 标定完成后自动切换到运行模式。
4. 检查空载补偿结果是否接近 0。
5. 检查受力时 TCP 输出方向和大小是否合理。
