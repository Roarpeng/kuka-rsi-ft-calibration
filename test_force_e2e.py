# 端到端仿真：真实 RSIServer(--force) 在 127.0.0.1 回环上跑，验证
# 收包 -> 补偿 -> 力控 -> RKorr 回包 的完整链路
import socket
import threading
import time

from calibration_io import load_config, load_calibration_result
from calibration_models import CalibratedWrench
from force_controller import ForceController
from udp_server import RSIConfig, RSIServer

LOOPBACK = "127.0.0.1"
PORT = 59353  # 用别的端口避免冲突；59101~59200 可能被 Windows 端口排除范围占用

calibration_config = load_config("ft_calibration_config.json")
calibration_config.mode = "calibrated_runtime"
# 现场标定值可能已写进配置（符号 -1 / 对中开）；端到端测试按未标定默认值跑
calibration_config.force_control.chisel_lateral_sign = 1
calibration_config.force_control.align_sign = 1
calibration_config.force_control.align_chisel_enable = False
calibration_config.force_control.align_drill_enable = False
calibration_config.force_control.align_deadband_nm = 0.5
# 端到端合成数据按"钻轴=X + 零 TCP 变换"典范帧构造（现场 TCP 为 Z 轴形态，不影响控制律验证）
from calibration_models import EulerTransform
calibration_config.flange_to_tcp = EulerTransform()
calibration_config.force_control.axis = "X"
result = load_calibration_result("ft_calibration.json")
# 标定结果里烘焙了现场 TCP 变换；端到端合成数据按典范系（恒等变换）构造，一并归一化
result.sensor_to_tcp_rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
result.sensor_to_tcp_translation_m = [0.0, 0.0, 0.0]
assert result is not None

config = RSIConfig(IP_NUMBER=LOOPBACK, BIND_IP=LOOPBACK, PORT=PORT, ROBOT_IP=LOOPBACK)
server = RSIServer(config=config, csv_filename="rsi_force_e2e.csv")
server.calibration_config = calibration_config
from calibration_runner import CalibrationRunner
server.calibration_runner = CalibrationRunner(calibration_config)
server.calibration_runner.calibration_result = result
server.force_controller = ForceController(
    calibration_config.force_control,
    rsi_rotation_order=calibration_config.rsi_rotation_order,
)
server.force_mode = True

thread = threading.Thread(target=server.run, daemon=True)
thread.start()
time.sleep(0.5)

# 客户端：模拟机器人发包。RobotStatus=FALSE 钻孔控倍率。
bias = result.force_bias_n
S = result.gravity_matrix_n
# 基准姿态(A-170,B0,C0)下 d=[0,0,-1]，重力项 = S·d = -S 的第 3 列
g_sensor = [-S[0][2], -S[1][2], -S[2][2]]
free_n = [b + g for b, g in zip(bias, g_sensor)]  # 无接触真实读数（传感器系）
press_n = [free_n[0] - 60.0, free_n[1], free_n[2]]  # 加压 60N（反力 -X）

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.settimeout(2.0)

def send_frames(force_n, count, ipoc_start, target_n=50, robot_status="FALSE", my_raw=88000):
    out = []
    fx, fy, fz = [int(v * 1000) for v in force_n]
    for i in range(count):
        xml = (
            f'<Rob Type="KUKA"><Fx_raw>{fx}</Fx_raw><Fy_raw>{fy}</Fy_raw><Fz_raw>{fz}</Fz_raw>'
            f'<Mx_raw>-106500</Mx_raw><My_raw>{my_raw}</My_raw><Mz_raw>18300</Mz_raw>'
            f'<Act_X>1120.000</Act_X><Act_Y>240.000</Act_Y><Act_Z>-1200.000</Act_Z>'
            f'<Act_A>-170.000</Act_A><Act_B>0.000</Act_B><Act_C>0.000</Act_C>'
            f'<data_collection>FALSE</data_collection><target_force>{target_n}</target_force>'
            f'<RobotStatus>{robot_status}</RobotStatus>'
            f'<IPOC>{ipoc_start + i}</IPOC></Rob>'
        )
        client.sendto(xml.encode("utf-8"), (LOOPBACK, PORT))
        data, _ = client.recvfrom(4096)
        out.append(data.decode("utf-8"))
        time.sleep(0.004)
    return out

import re
def rkorr_of(reply):
    m = re.search(r'<RKorr X="([-\d.]+)" Y="([-\d.]+)" Z="([-\d.]+)"', reply)
    return tuple(float(v) for v in m.groups())

def ov_of(reply):
    m = re.search(r'<OV_PRO>([-\d.]+)</OV_PRO>', reply)
    assert m, f"回包缺少 OV_PRO: {reply}"
    return float(m.group(1))

r1 = send_frames(free_n, 100, 1000, robot_status="FALSE")
rx, ry, rz = rkorr_of(r1[-1])
assert abs(rx) < 1e-9 and abs(ry) < 1e-9 and abs(rz) < 1e-9
assert abs(ov_of(r1[-1]) - 100.0) < 0.1
print("阶段1 钻孔未接触：OV_PRO=100、RKorr=0，OK")

r2 = send_frames(press_n, 80, 2000, robot_status="FALSE")
rx, ry, rz = rkorr_of(r2[-1])
assert abs(rx) < 1e-9 and abs(ry) < 1e-9 and abs(rz) < 1e-9
assert ov_of(r2[-1]) <= 1e-6
assert all(abs(c) < 1e-9 for c in server.force_controller.corr_cumulative_mm)
assert "<IPOC>2079</IPOC>" in r2[-1]
print("阶段2 钻孔受压60N：OV_PRO=0、不叠位移，OK")

server.force_controller.reset()
r3 = send_frames(press_n, 20, 3000, robot_status="TRUE")
rx, ry, rz = rkorr_of(r3[-1])
assert abs(rx) < 1e-9 and abs(ry) < 1e-9 and abs(rz) < 1e-9
ov3 = ov_of(r3[-1])
# 凿击：X 仍恒力 -> 60N 超目标 50N，倍率应持续下降（无横向力，RKorr=0）
assert ov3 < 100.0 - 1e-6, f"凿击 X 恒力应开始降倍率: {ov3}"
print(f"阶段3 凿击（TRUE）：X 恒力降倍率（OV={ov3:.0f}%）、RKorr=0，OK")

# 阶段4 调试台固定输出：优先于力控，逐拍写入回包
server.debug_override = {"enabled": True, "x": 0.02, "y": -0.01, "z": 0.0,
                         "a": 0.005, "b": 0.0, "c": -0.005, "ov_pro": 77.0}
r4 = send_frames(free_n, 3, 4000, robot_status="FALSE")
assert 'X="0.0200"' in r4[-1] and 'Y="-0.0100"' in r4[-1], r4[-1]
assert 'A="0.0050"' in r4[-1] and 'C="-0.0050"' in r4[-1], r4[-1]
assert "<OV_PRO>77.0000</OV_PRO>" in r4[-1]
print("阶段4 调试固定输出：优先于力控、六轴+倍率逐拍下发，OK")

# 停止：RKorr/倍率立即回零并复位力控器，之后恢复力控律
server.disable_debug_override("测试")
assert server.debug_override["enabled"] is False
r5 = send_frames(press_n, 2, 5000, robot_status="FALSE")
assert 'X="0.0000"' in r5[-1], r5[-1]
assert ov_of(r5[-1]) < 100.0, "停止后力控应重新接管（受压 60N -> 倍率从 100 回落）"
assert all(abs(c) < 1e-9 for c in server.force_controller.corr_cumulative_mm)
print("阶段5 停止固定输出：立即回零、力控恢复接管，OK")

# 阶段6 服务模式在线切换（免重启）：监控 -> 不调倍率；切回力控 -> 恢复调节
server.set_service_mode("monitor")
r6 = send_frames(press_n, 5, 6000, robot_status="FALSE")
assert 'X="0.0000"' in r6[-1], r6[-1]
assert abs(ov_of(r6[-1]) - 100.0) < 0.1, f"监控模式下受压也不应调倍率: {ov_of(r6[-1])}"
server.set_service_mode("force")
r7 = send_frames(press_n, 5, 7000, robot_status="FALSE")
assert 'X="0.0000"' in r7[-1]
assert ov_of(r7[-1]) < 100.0, f"切回力控应恢复调倍率: {ov_of(r7[-1])}"
print("阶段6 服务模式在线切换：监控停调倍率、切回力控恢复，OK")

# 阶段7 轴线对中（钻孔）：My 残差超死区 -> B 旋转输出；平移仍 0；关闭后缓撤到 0
# 力矩 raw 常量与标定强相关（换标定后残差会漂），先运行时测基线与斜率再构造 +2 N·m 残差
server.force_controller.reset()
send_frames(free_n, 20, 8000)   # 探针给足帧数：入口滤波(中值5+LPF)需 ~5 帧收敛
my0 = server.rsi_data_list[-1].tcp_my
send_frames(free_n, 20, 8020, my_raw=98000)
my1 = server.rsi_data_list[-1].tcp_my
slope = (my1 - my0) / 10000.0  # 每 raw count 对应的 tcp_my 变化
assert slope > 0.0005, f"力矩斜率异常: {slope}"
my_target = int(88000 + (2.0 - my0) / slope)  # 构造 +2 N·m 残差
calibration_config.force_control.align_drill_enable = True
r8 = send_frames(free_n, 30, 8100, my_raw=my_target)
m = re.search(r'B="([-\d.]+)"', r8[-1])
assert m and float(m.group(1)) > 0.0, f"B 应按 +My 输出: {r8[-1]}"
assert 'X="0.0000"' in r8[-1] and 'Y="0.0000"' in r8[-1] and 'Z="0.0000"' in r8[-1]
calibration_config.force_control.align_drill_enable = False
r9 = send_frames(free_n, 60, 9000)
m = re.search(r'B="([-\d.]+)"', r9[-1])
assert m and float(m.group(1)) == 0.0, f"关闭对中后 B 应缓撤到 0: {r9[-1]}"
print("阶段7 钻孔轴线对中：B←My 输出、关闭后缓撤到 0，OK")

print("\n端到端力控链路验证通过")
