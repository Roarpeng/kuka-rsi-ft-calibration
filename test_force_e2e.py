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
PORT = 59153  # 用别的端口避免冲突

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

# 客户端：模拟机器人发包。钻孔模式 search_before_contact=false：
# 阶段1（未接触，补偿后≈0）应输出零修正；阶段2（TOOL X 受压 60N > 目标 50N）应退刀
bias = result.force_bias_n
S = result.gravity_matrix_n
# 基准姿态(A-170,B0,C0)下 d=[0,0,-1]，重力项 = S·d = -S 的第 3 列
g_sensor = [-S[0][2], -S[1][2], -S[2][2]]
free_n = [b + g for b, g in zip(bias, g_sensor)]  # 无接触真实读数（传感器系）
press_n = [free_n[0] - 60.0, free_n[1], free_n[2]]  # 加压 60N（反力 -X）

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.settimeout(2.0)

def send_frames(force_n, count, ipoc_start):
    out = []
    fx, fy, fz = [int(v * 1000) for v in force_n]
    for i in range(count):
        xml = (
            f'<Rob Type="KUKA"><Fx_raw>{fx}</Fx_raw><Fy_raw>{fy}</Fy_raw><Fz_raw>{fz}</Fz_raw>'
            f'<Mx_raw>-106500</Mx_raw><My_raw>88000</My_raw><Mz_raw>18300</Mz_raw>'
            f'<Act_X>1120.000</Act_X><Act_Y>240.000</Act_Y><Act_Z>-1200.000</Act_Z>'
            f'<Act_A>-170.000</Act_A><Act_B>0.000</Act_B><Act_C>0.000</Act_C>'
            f'<data_collection>FALSE</data_collection><target_force>50</target_force>'
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

r1 = send_frames(free_n, 100, 1000)
rx, ry, rz = rkorr_of(r1[-1])
assert abs(rx) < 1e-9 and abs(ry) < 1e-9 and abs(rz) < 1e-9, f"钻孔模式未接触应为零修正: {rx},{ry},{rz}"
print("阶段1 未接触：零修正（钻孔模式不前推），OK")

r2 = send_frames(press_n, 300, 2000)
rx, ry, rz = rkorr_of(r2[-1])
print(f"阶段2 受压60N(目标50N)：RKorr X={rx}, Y={ry}, Z={rz}")
# 超目标 10N -> 退刀；含积分积累，幅值随时间增大，只验证方向和限幅
assert 0.005 < rx <= 0.2, f"应输出退刀修正(+X 且在限幅内): {rx}"
assert ry > 0 and abs(rz) < 1e-6
assert "<IPOC>2299</IPOC>" in r2[-1]
print("方向与幅值符合退刀预期，IPOC 回传一致")

print("\n端到端力控链路验证通过")
