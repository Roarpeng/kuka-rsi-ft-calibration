# 恒力钻孔闭环仿真：编程进给 10 mm/s（行程 60mm）+ 材料切削模型
# 切削模型：接触且推进力 > 5N 时，孔底以 k_cut * 力 的速度后退（材料被切除）
# 现场几何：进给 = 工具 +X（press_motion_sign=+1），退刀 = 工具 −X；压紧时传感器读数为 −X
# RKorr 语义：#RELATIVE 每拍增量；仿真里 pos_corr 自行累加得到总偏移
from calibration_io import load_config, load_calibration_result
from calibration_math import euler_to_matrix
from force_controller import ForceController
import udp_server  # 语法/导入检查

cfg = load_config("ft_calibration_config.json")
fc = cfg.force_control
print("配置：轴 TOOL", fc.axis, "目标力", fc.default_target_force_n,
      "N  Kp", fc.kp_mm_per_s_per_n, " Ki", fc.ki_mm_per_s2_per_n,
      " 加压限速", fc.advance_speed_mm_s, "mm/s")
assert load_calibration_result("ft_calibration.json") is not None

ANGLES = [-170.0, 0.0, 0.0]
R = euler_to_matrix(ANGLES, "ZYX")
tool_x_base = [R[0][0], R[1][0], R[2][0]]
START = [1120.0, 240.0, -1200.0]


def tcp_mm(pos_along_tool_x: float) -> list[float]:
    return [START[i] + pos_along_tool_x * tool_x_base[i] for i in range(3)]

K_SURFACE = 200.0       # 接触刚度 N/mm
K_CUT = 0.05            # 切削率 (mm/s)/N —— 50N 时 2.5 mm/s
FEED_MM_S = 10.0        # 编程进给速度
PATH_LEN = 60.0         # 编程行程 mm
target = 50.0

ctrl = ForceController(fc)
gap = 2.0
pos_prog = 0.0          # 编程位置（沿工具 +X 进给）
pos_corr = 0.0          # 工具系 X 总偏移（仿真累加每拍增量）；负值 = 退刀
prev_corr = 0.0
hole_depth = 0.0        # 已切削深度
max_step = 0.0
peak_force = 0.0
history = []
steps = int(30.0 / fc.cycle_s)
for i in range(steps):
    feed = FEED_MM_S if pos_prog < PATH_LEN else 0.0
    pos_prog += feed * fc.cycle_s

    pos_actual = pos_prog + pos_corr
    penetration = pos_actual - gap - hole_depth   # 工件在工具 +X 方向
    f_x = -K_SURFACE * penetration if penetration > 0 else 0.0
    press = fc.press_sign * f_x
    if press > 5.0:
        hole_depth += K_CUT * press * fc.cycle_s   # 材料被切除，孔底后退

    rk = ctrl.update([f_x, 0.0, 0.0], ANGLES, target, tcp_position_mm=tcp_mm(pos_actual))
    max_step = max(max_step, abs(rk["RKorr.X"]), abs(rk["RKorr.Y"]), abs(rk["RKorr.Z"]))
    pos_corr += rk["RKorr.X"]
    prev_corr = pos_corr
    peak_force = max(peak_force, press)
    if i % int(3.0 / fc.cycle_s) == 0 or i == steps - 1:
        history.append((i * fc.cycle_s, press, hole_depth, pos_corr, ctrl.tripped))

print("\n时间(s)  推进力(N)  孔深(mm)  修正(mm)  保护")
for t, f, d, c, tr in history:
    print("  %5.1f  %8.1f  %7.2f  %8.2f   %s" % (t, f, d, c, "触发" if tr else "-"))

print("\n=== 断言检查 ===")
assert max_step <= fc.per_cycle_max_mm + 1e-9, f"每周期修正速度限幅被突破: {max_step}"
print(f"1. 每周期修正变化限幅 OK (max={max_step:.4f} mm <= {fc.per_cycle_max_mm})")
print(f"2. 峰值推进力 {peak_force:.1f} N（保护阈值 {fc.max_force_n} N）")
assert peak_force < fc.max_force_n, "正常钻进不应触发保护"
assert abs(press - target) < 5.0, f"未收敛: {press}"
print(f"3. 末期推进力 {press:.1f} N ≈ 目标 {target} N，OK")
assert abs(ctrl.corr_cumulative_mm[0]) <= fc.cumulative_max_mm + 1e-9
print(f"4. 累积限幅 OK ({ctrl.corr_cumulative_mm[0]:.2f} mm <= {fc.cumulative_max_mm})")

# 钻穿：快通道立刻退出接触，此后发 0 增量（叠加保持，不因低通滞后继续加压）
cum0 = ctrl.corr_cumulative_mm[0]
ctrl.update([0.0, 0.0, 0.0], ANGLES, target)
for _ in range(20):
    ctrl.update([0.0, 0.0, 0.0], ANGLES, target)
rk1 = ctrl.update([0.0, 0.0, 0.0], ANGLES, target)
for _ in range(100):
    rk2 = ctrl.update([0.0, 0.0, 0.0], ANGLES, target)
assert rk1 == rk2, f"钻穿后增量应保持不变: {rk1} -> {rk2}"
assert all(abs(v) < 1e-9 for v in rk1.values()), f"钻穿后应发 0 增量: {rk1}"
lurch = abs(ctrl.corr_cumulative_mm[0] - cum0)
max_lurch = fc.filter_protect_window * fc.advance_speed_mm_s * fc.cycle_s + 1e-9
assert lurch <= max_lurch, f"钻穿前冲过大: {lurch}"
print(f"5. 钻穿后发 0 增量、叠加保持（快通道退出，前冲 {lurch:.3f} mm），OK")

# 目标力 0：按每拍限幅发反向增量，把叠加撤到 0
for _ in range(2000):
    rk0 = ctrl.update([0.0, 0.0, 0.0], ANGLES, 0.0)
    assert all(abs(v) <= fc.per_cycle_max_mm + 1e-9 for v in rk0.values()), f"撤除增量超限: {rk0}"
assert all(abs(v) < 1e-9 for v in rk0.values()), f"末拍增量未到 0: {rk0}"
assert all(abs(c) < 1e-9 for c in ctrl.corr_cumulative_mm), f"叠加未撤除到 0: {ctrl.corr_cumulative_mm}"
print("6. 目标力 <=0 时按限幅撤除叠加到 0，OK")
ctrl.reset()
rk = ctrl.update([-200.0, 0.0, 0.0], ANGLES, target)
assert rk["RKorr.X"] < -0.5 * fc.per_cycle_max_mm and abs(rk["RKorr.Y"]) < 1e-9, f"超力保护应沿工具 -X 退刀: {rk}"
print("7. 超力保护触发并向工具 -X 全速退刀，OK")

# 空载残差低于接触阈值：全程零修正，不得超力
ctrl.reset()
idle_fx = -8.0  # press = 8 N < 12 N
idle_peak = 0.0
idle_corr = 0.0
for _ in range(int(6.0 / fc.cycle_s)):  # 60mm / 10mm/s
    rk = ctrl.update([idle_fx, 0.0, 0.0], ANGLES, target)
    idle_peak = max(idle_peak, abs(idle_fx))
    idle_corr = max(idle_corr, abs(rk["RKorr.X"]), abs(rk["RKorr.Y"]), abs(rk["RKorr.Z"]))
assert not ctrl.tripped and not ctrl.in_contact
assert idle_corr < 1e-9, f"空载不应输出修正: {idle_corr}"
print("8. 空载残差 8N：不接触、零修正、不超力，OK")

# 目标力 0 时碰到表面：立即退刀，而不是等到 150N
ctrl.reset()
rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, 0.0)
assert rk["RKorr.X"] < -0.5 * fc.per_cycle_max_mm, f"目标力0碰到表面应退刀: {rk}"
print("9. 目标力 0 且接触：立即退刀，OK")

# 刚性接触 + 10mm/s 进给：用原始力退刀 + 进给前馈，峰值不得过保护阈值
ctrl.reset()
K_STIFF = 800.0
gap_s = 1.0
pos_prog = 0.0
pos_corr = 0.0
peak_stiff = 0.0
press_s = 0.0
for i in range(int(3.0 / fc.cycle_s)):
    pos_prog += FEED_MM_S * fc.cycle_s
    pos_actual = pos_prog + pos_corr
    penetration = pos_actual - gap_s
    f_x = -K_STIFF * penetration if penetration > 0 else 0.0
    press_s = fc.press_sign * f_x
    peak_stiff = max(peak_stiff, press_s)
    rk = ctrl.update([f_x, 0.0, 0.0], ANGLES, target, tcp_position_mm=tcp_mm(pos_actual))
    pos_corr += rk["RKorr.X"]
assert peak_stiff < fc.max_force_n, f"刚性接触峰值 {peak_stiff:.1f} N 超过保护 {fc.max_force_n}"
assert not ctrl.tripped, "刚性接触不应触发超力保护"
assert abs(press_s - target) < 15.0, f"刚性接触未收敛: {press_s}"
print(f"10. 刚性接触峰值 {peak_stiff:.1f} N < {fc.max_force_n} N，末期 {press_s:.1f} N，OK")

# 滤波滞后：连续低压后连续 3 拍 80N，快通道确认接触后当周期退刀
ctrl.reset()
for _ in range(10):
    ctrl.update([-10.0, 0.0, 0.0], ANGLES, target)
rk_before_x = ctrl.corr_cumulative_mm[0]
rk = {"RKorr.X": 0.0}
for _ in range(3):
    rk = ctrl.update([-80.0, 0.0, 0.0], ANGLES, target)
assert rk["RKorr.X"] < 0.0, f"原始力超目标应立即发负增量退刀: {rk}"
assert ctrl.corr_cumulative_mm[0] < rk_before_x, f"叠加应向 -X 减小: {rk_before_x} -> {ctrl.corr_cumulative_mm[0]}"
print("11. 滤波滞后时原始力超目标立即退刀，OK")

# 单帧脉冲不应判接触、不应输出修正（短中值滤掉）
ctrl.reset()
for _ in range(20):
    ctrl.update([-8.0, 0.0, 0.0], ANGLES, target)
rk_spike = ctrl.update([-80.0, 0.0, 0.0], ANGLES, target)
assert not ctrl.in_contact, "单帧 80N 不应判接触"
assert abs(rk_spike["RKorr.X"]) < 1e-9
print("12. 单帧脉冲被中值滤掉，不误判接触，OK")

# 30Hz 弹跳（幅值不足以超目标）：控制通道低通，修正步长应平稳
ctrl.reset()
import math
amp, freq, bias = 8.0, 30.0, -20.0  # press ≈ 20±8 N，低于目标 50
rkorr_steps = []
for i in range(250):
    t = i * fc.cycle_s
    fx = bias + amp * math.sin(2 * math.pi * freq * t)
    rk = ctrl.update([fx, 0.0, 0.0], ANGLES, target)
    rkorr_steps.append(abs(rk["RKorr.X"]))
steady = rkorr_steps[50:]
assert max(steady) <= fc.per_cycle_max_mm + 1e-9
assert sum(steady) / len(steady) < 0.08, f"30Hz 弹跳下 RKorr 仍在大幅追振: {sum(steady)/len(steady):.4f}"
print(f"13. 30Hz 弹跳：平均修正步长 {sum(steady)/len(steady):.4f} mm/周期，OK")

server = udp_server.RSIServer(udp_server.RSIConfig())
assert server.self_test_xml()
print("14. XML 自检通过，OK")

# 弹跳过零不得退出接触（#RELATIVE 下退出后发 0，叠加冻结，LIN 继续撞墙）
ctrl.reset()
pos_prog = 0.0
for _ in range(20):
    pos_prog += FEED_MM_S * fc.cycle_s
    ctrl.update([-20.0, 0.0, 0.0], ANGLES, target, tcp_position_mm=tcp_mm(pos_prog))
assert ctrl.in_contact
zero_cross_exits = 0
for i in range(30):
    t = i * fc.cycle_s
    fx = -180.0 * math.sin(2 * math.pi * 30.0 * t)
    pos_prog += FEED_MM_S * fc.cycle_s
    ctrl.update([fx, 0.0, 0.0], ANGLES, target, tcp_position_mm=tcp_mm(pos_prog))
    if not ctrl.in_contact:
        zero_cross_exits += 1
assert zero_cross_exits == 0, f"30Hz 弹跳把接触抖掉了 {zero_cross_exits} 次"
print("15. 30Hz ±180N 弹跳：接触闩住，OK")

# 超力过零不得马上解锁
ctrl.reset()
ctrl.update([-200.0, 0.0, 0.0], ANGLES, target)
assert ctrl.tripped
for _ in range(10):
    ctrl.update([-1.0, 0.0, 0.0], ANGLES, target)
    assert ctrl.tripped, "超力过零 40ms 内不得解除"
print("16. 超力保护过零不立即解锁，OK")

# 钻孔：RobotStatus=FALSE 控 $OV_PRO；凿击 TRUE 暂不控位移
ctrl.reset()
xml_drill = udp_server.SAMPLE_ROB_XML  # FALSE=钻孔
parsed_drill = udp_server.RSIServer(udp_server.RSIConfig()).parse_rsi_xml(
    xml_drill.encode("utf-8")
)
assert parsed_drill is not None and parsed_drill.RobotStatus is False
xml_chisel = udp_server.SAMPLE_ROB_XML.replace(
    "<RobotStatus>FALSE</RobotStatus>", "<RobotStatus>TRUE</RobotStatus>"
)
parsed_chisel = udp_server.RSIServer(udp_server.RSIConfig()).parse_rsi_xml(
    xml_chisel.encode("utf-8")
)
assert parsed_chisel is not None and parsed_chisel.RobotStatus is True
idle_ov = 100.0
for _ in range(5):
    rk = ctrl.update([-8.0, 0.0, 0.0], ANGLES, target, mode="drill")
    idle_ov = ctrl.ov_pro_pct
    assert all(abs(v) < 1e-9 for v in rk.values()), f"钻孔空载不应叠 RKorr: {rk}"
assert abs(idle_ov - fc.default_ov_pro) < 1e-6, f"空载应满倍率: {idle_ov}"
print("17. 钻孔空载：OV=100、RKorr=0，OK")

ctrl.reset()
for _ in range(int(0.3 / fc.cycle_s)):
    rk = ctrl.update([-50.0, 0.0, 0.0], ANGLES, target, mode="drill")
assert all(abs(v) < 1e-9 for v in rk.values())
assert ctrl.ov_pro_pct <= 1e-6, f"到位应变 0 停进给: {ctrl.ov_pro_pct}"
print("18. 钻孔力到位：OV=0、RKorr=0，OK")

ctrl.reset()
for _ in range(int(0.3 / fc.cycle_s)):
    ctrl.update([-200.0, 0.0, 0.0], ANGLES, target, mode="drill")
assert ctrl.ov_pro_pct <= 1e-6
assert all(abs(c) < 1e-9 for c in ctrl.corr_cumulative_mm), "钻孔超力应停进给，不得抽刀叠加"
print("19. 钻孔超力：OV=0、不叠位移，OK")

ctrl.reset()
rk = ctrl.update([-80.0, 0.0, 0.0], ANGLES, target, mode="chisel")
assert abs(ctrl.ov_pro_pct - fc.default_ov_pro) < 1e-6
assert all(abs(v) < 1e-9 for v in rk.values())
print("20. 凿击占位：默认倍率、RKorr=0（位移后续），OK")

ctrl.reset()
rk = ctrl.update([-200.0, 0.0, 0.0], ANGLES, target, mode="overlay")
assert rk["RKorr.X"] < -0.5 * fc.per_cycle_max_mm
assert abs(ctrl.ov_pro_pct - fc.default_ov_pro) < 1e-6
print("21. overlay 测试路径超力仍走 RKorr 退刀，OK")

print("\n全部自检通过")
