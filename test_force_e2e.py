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
result = load_calibration_result("ft_calibration.json")
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

def send_frames(force_n, count, ipoc_start, target_n=50, robot_status="FALSE"):
    out = []
    fx, fy, fz = [int(v * 1000) for v in force_n]
    for i in range(count):
        xml = (
            f'<Rob Type="KUKA"><Fx_raw>{fx}</Fx_raw><Fy_raw>{fy}</Fy_raw><Fz_raw>{fz}</Fz_raw>'
            f'<Mx_raw>-106500</Mx_raw><My_raw>88000</My_raw><Mz_raw>18300</Mz_raw>'
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

print("\n端到端力控链路验证通过")
