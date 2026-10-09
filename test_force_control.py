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
# 现场标定会把符号/开关写进配置文件（如实测 lateral/align = -1、对中开）；
# 测试一律按未标定默认值跑，避免依赖现场状态
fc.chisel_lateral_sign = 1
fc.align_sign = 1
fc.align_chisel_enable = False
fc.align_drill_enable = False
fc.align_deadband_nm = 0.5
fc.align_trip_nm = 8.0
fc.align_per_cycle_max_deg = 0.02
fc.align_max_deg = 3.0
fc.axis = "X"  # 本仿真按钻轴=X 的典范帧跑（现场 TCP 重标定后 axis=Z，通道自动推导）
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
assert parsed_drill is not None and parsed_drill.RobotStatus == 2
xml_chisel = udp_server.SAMPLE_ROB_XML.replace(
    "<RobotStatus>2</RobotStatus>", "<RobotStatus>3</RobotStatus>"
)
parsed_chisel = udp_server.RSIServer(udp_server.RSIConfig()).parse_rsi_xml(
    xml_chisel.encode("utf-8")
)
assert parsed_chisel is not None and parsed_chisel.RobotStatus == 3
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

# 凿击（RobotStatus=TRUE）：X 仍按 OV_PRO 恒力压紧；无横向力时 RKorr=0
ctrl.reset()
for _ in range(int(0.3 / fc.cycle_s)):
    rk = ctrl.update([-50.0, 0.0, 0.0], ANGLES, target, mode="chisel")
assert all(abs(v) < 1e-9 for v in rk.values()), f"凿击无横向力不应叠 RKorr: {rk}"
assert ctrl.ov_pro_pct <= 1e-6, f"凿击 X 恒力到位应停进给: {ctrl.ov_pro_pct}"
print("20. 凿击无横向力：X 仍 OV_PRO 恒力（到位 OV→0）、RKorr=0，OK")

ctrl.reset()
rk = ctrl.update([-200.0, 0.0, 0.0], ANGLES, target, mode="overlay")
assert rk["RKorr.X"] < -0.5 * fc.per_cycle_max_mm
assert abs(ctrl.ov_pro_pct - fc.default_ov_pro) < 1e-6
print("21. overlay 测试路径超力仍走 RKorr 退刀，OK")

# ---- 凿击横向让位：Y/Z 零力让位（滑坑/卡滞卸载），方向 = 符号位 * 读数方向 ----
# 读数约定：读数 = 工件对工具作用力（压紧时 X 读负）。批头被 +Y 侧坑壁顶 -> 读数 +Y
# -> 让位 +Y（背离障碍物）。口述"批头+Y受力"指工具对外施力方向，与此是同一动作。

# 22. 让位方向与读数同号；横向调节期间 X 恒力冻结，撤力后恢复
ctrl.reset()
for _ in range(int(0.1 / fc.cycle_s)):
    ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel")
ov_settled = ctrl.ov_pro_pct   # press=30N -> OV 稳在 ~66.7%
assert 0.0 < ov_settled < fc.default_ov_pro, f"预置 OV 应处于中途: {ov_settled}"
steps_y = []
ov_at_activate = None
for _ in range(int(0.5 / fc.cycle_s)):
    # X 同时升到 40N（目标 OV ~33%）：若不冻结，OV 会一路降下去
    rk = ctrl.update([-40.0, 40.0, 0.0], ANGLES, target, mode="chisel")
    steps_y.append(rk["RKorr.Y"])
    assert abs(rk["RKorr.X"]) < 1e-9, f"凿击 X 不用 RKorr: {rk}"
    if ctrl.chisel_lateral_active and ov_at_activate is None:
        ov_at_activate = ctrl.ov_pro_pct
assert ov_at_activate is not None and ov_at_activate > 35.0, \
    f"横向激活时 OV 应尚未降到新目标 ~33%: {ov_at_activate}"
active_steps = [s for s in steps_y if s != 0.0]
assert active_steps, "横向力 40N 超死区后应产生让位增量"
assert all(s > 0.0 for s in active_steps), f"让位方向应与 +Y 读数同号: {active_steps[:5]}"
assert max(abs(s) for s in active_steps) <= fc.per_cycle_max_mm + 1e-9
assert ctrl.corr_cumulative_mm[1] > 1e-9
assert ctrl.ov_pro_pct == ov_at_activate, \
    f"横向调节期间 X 恒力应冻结: {ov_at_activate} -> {ctrl.ov_pro_pct}"
for _ in range(50):
    ctrl.update([-40.0, 0.0, 0.0], ANGLES, target, mode="chisel")
assert not ctrl.chisel_lateral_active, "横向力撤销后应退出让位"
assert ctrl.ov_pro_pct < ov_at_activate, "横向撤销后 X 恒力应恢复调节"
print("22. 横向让位：方向与读数同号、X 恒力冻结/恢复，OK")

# 23. 横向力低于死区不让位（空载残差 5~12N 不得漂移）
ctrl.reset()
for _ in range(int(1.0 / fc.cycle_s)):
    rk = ctrl.update([-30.0, 10.0, 10.0], ANGLES, target, mode="chisel")
assert abs(rk["RKorr.Y"]) < 1e-9 and abs(rk["RKorr.Z"]) < 1e-9, f"低于死区不应让位: {rk}"
assert not ctrl.chisel_lateral_active
assert abs(ctrl.corr_cumulative_mm[1]) < 1e-9 and abs(ctrl.corr_cumulative_mm[2]) < 1e-9
print("23. 横向力 10N < 死区 15N：不让位（空载残差不漂移），OK")

# 24. 方向符号位反号生效（实机手推标定后可改）
ctrl.reset()
fc.chisel_lateral_sign = -1
try:
    steps = []
    for _ in range(int(0.3 / fc.cycle_s)):
        rk = ctrl.update([-30.0, 40.0, 0.0], ANGLES, target, mode="chisel")
        steps.append(rk["RKorr.Y"])
    active = [s for s in steps if s != 0.0]
    assert active and all(s < 0.0 for s in active), f"反号后让位应沿 -Y: {active[:5]}"
finally:
    fc.chisel_lateral_sign = 1
print("24. 让位方向符号位 -1 生效，OK")

# 25. Z 轴同样让位（负读数 -> 负方向）
ctrl.reset()
steps_z = []
for _ in range(int(0.3 / fc.cycle_s)):
    rk = ctrl.update([-30.0, 0.0, -40.0], ANGLES, target, mode="chisel")
    steps_z.append(rk["RKorr.Z"])
active = [s for s in steps_z if s != 0.0]
assert active and all(s < 0.0 for s in active), f"Z 轴让位方向应与读数同号(负): {active[:5]}"
assert ctrl.corr_cumulative_mm[2] < -1e-9
print("25. Z 轴横向让位同号生效，OK")

# 26. 让位行程上限 ±20mm：到限后该轴停止让位
ctrl.reset()
old_gain = fc.chisel_lateral_gain_mm_per_s_per_n
fc.chisel_lateral_gain_mm_per_s_per_n = 5.0  # 40-15=25N 超出 -> 饱和到每拍上限
try:
    for _ in range(int(2.0 / fc.cycle_s)):   # 0.08mm/拍，~250 拍到 20mm
        ctrl.update([-30.0, 40.0, 0.0], ANGLES, target, mode="chisel")
    assert ctrl.corr_cumulative_mm[1] <= fc.chisel_lateral_max_mm + 1e-9, \
        f"横向累计超上限: {ctrl.corr_cumulative_mm[1]}"
    assert abs(ctrl.corr_cumulative_mm[1] - fc.chisel_lateral_max_mm) < 0.5 * fc.per_cycle_max_mm
    rk = ctrl.update([-30.0, 40.0, 0.0], ANGLES, target, mode="chisel")
    assert abs(rk["RKorr.Y"]) < 1e-9, f"到 20mm 上限后应停止让位: {rk}"
finally:
    fc.chisel_lateral_gain_mm_per_s_per_n = old_gain
print(f"26. 让位行程上限 {fc.chisel_lateral_max_mm} mm 到限停止，OK")

# 27. 横向卡滞硬阈值：原始力超限立即 -X 全速退刀，过零不立即解锁
ctrl.reset()
rk = ctrl.update([-30.0, 150.0, 0.0], ANGLES, target, mode="chisel")
assert ctrl.tripped, "横向 150N 应触发卡滞保护"
assert rk["RKorr.X"] < -0.5 * fc.per_cycle_max_mm, f"卡滞保护应沿 -X 全速退刀: {rk}"
assert abs(rk["RKorr.Y"]) < 1e-9, "保护期间不再让位"
for _ in range(10):
    ctrl.update([-1.0, 1.0, 0.0], ANGLES, target, mode="chisel")
    assert ctrl.tripped, "卡滞保护过零 40ms 内不得解除"
for _ in range(int(0.5 / fc.cycle_s)):
    ctrl.update([-1.0, 1.0, 0.0], ANGLES, target, mode="chisel")
assert not ctrl.tripped, "持续卸荷 0.2s 后应解除保护"
print("27. 横向卡滞保护：立即退刀、闩锁、卸荷解除，OK")

# 28. 退出凿击回钻孔：横向叠加按每拍限幅缓撤到 0，X 始终 0
ctrl.reset()
old_gain = fc.chisel_lateral_gain_mm_per_s_per_n
fc.chisel_lateral_gain_mm_per_s_per_n = 5.0
try:
    for _ in range(int(0.6 / fc.cycle_s)):
        ctrl.update([-30.0, 40.0, 0.0], ANGLES, target, mode="chisel")
    assert ctrl.corr_cumulative_mm[1] > 1.0, "应已积累明显横向让位"
finally:
    fc.chisel_lateral_gain_mm_per_s_per_n = old_gain
for _ in range(int(2.0 / fc.cycle_s)):
    rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="drill")
    assert abs(rk["RKorr.X"]) < 1e-9, f"钻孔 X 不用 RKorr: {rk}"
    assert abs(rk["RKorr.Y"]) <= fc.per_cycle_max_mm + 1e-9
assert abs(ctrl.corr_cumulative_mm[1]) < 1e-9, f"钻孔下横向叠加应撤到 0: {ctrl.corr_cumulative_mm}"
assert abs(rk["RKorr.Y"]) < 1e-9
print("28. 凿击->钻孔切换：横向叠加按限幅缓撤到 0，OK")

# ---- 轴线零力矩对中（B←My、C←Mz）：凿击叠加在平移让位上、钻孔单独可开，默认关 ----

# 29. 凿击对中：My 超死区 -> B 与 My 同号；X/Y/Z 平移不受影响，C 不动
ctrl.reset()
fc.align_chisel_enable = True
try:
    steps_b = []
    for _ in range(int(0.5 / fc.cycle_s)):
        rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                         torque_tcp=[0.0, 1.5, 0.0])
        steps_b.append(rk["RKorr.B"])
        assert abs(rk["RKorr.X"]) < 1e-9
        assert abs(rk["RKorr.C"]) < 1e-9, "Mz=0 时 C 不应动"
    active_b = [s for s in steps_b if s != 0.0]
    assert active_b and all(s > 0.0 for s in active_b), f"B 应与 +My 同号: {active_b[:5]}"
    assert max(abs(s) for s in active_b) <= fc.align_per_cycle_max_deg + 1e-9
    assert ctrl.align_active and ctrl.corr_cumulative_deg[1] > 1e-9
finally:
    fc.align_chisel_enable = False
print("29. 凿击轴线对中：B←My 同号、平移不受影响、单轴无串扰，OK")

# 30. Mz 负力矩 -> C 负方向
ctrl.reset()
fc.align_chisel_enable = True
try:
    steps_c = []
    for _ in range(int(0.3 / fc.cycle_s)):
        rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                         torque_tcp=[0.0, 0.0, -1.5])
        steps_c.append(rk["RKorr.C"])
        assert abs(rk["RKorr.B"]) < 1e-9
finally:
    fc.align_chisel_enable = False
active_c = [s for s in steps_c if s != 0.0]
assert active_c and all(s < 0.0 for s in active_c), f"C 应与 -Mz 同号: {active_c[:5]}"
assert ctrl.corr_cumulative_deg[2] < -1e-9
print("30. C←Mz 负力矩同号生效，OK")

# 31. 力矩低于死区不对中（空载力矩残差不漂）
ctrl.reset()
fc.align_chisel_enable = True
try:
    for _ in range(int(1.0 / fc.cycle_s)):
        rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                         torque_tcp=[0.0, 0.3, 0.3])
finally:
    fc.align_chisel_enable = False
assert abs(rk["RKorr.B"]) < 1e-9 and abs(rk["RKorr.C"]) < 1e-9
assert not ctrl.align_active
assert abs(ctrl.corr_cumulative_deg[1]) < 1e-9 and abs(ctrl.corr_cumulative_deg[2]) < 1e-9
print(f"31. 力矩 0.3 N·m < 死区 {fc.align_deadband_nm}：不对中，OK")

# 32. 对中累计限幅 ±align_max_deg：到限后该轴停止
ctrl.reset()
old_agn = fc.align_gain_deg_per_s_per_nm
fc.align_gain_deg_per_s_per_nm = 5.0  # 饱和到每拍上限 0.02°
fc.align_chisel_enable = True
try:
    for _ in range(int(2.0 / fc.cycle_s)):
        ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                    torque_tcp=[0.0, 1.5, 0.0])
    assert ctrl.corr_cumulative_deg[1] <= fc.align_max_deg + 1e-9, \
        f"对中累计超上限: {ctrl.corr_cumulative_deg[1]}"
    assert abs(ctrl.corr_cumulative_deg[1] - fc.align_max_deg) < fc.align_per_cycle_max_deg
    rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                     torque_tcp=[0.0, 1.5, 0.0])
    assert abs(rk["RKorr.B"]) < 1e-9, f"到限后应停止对中: {rk}"
finally:
    fc.align_gain_deg_per_s_per_nm = old_agn
    fc.align_chisel_enable = False
print(f"32. 对中累计限幅 ±{fc.align_max_deg}° 到限停止，OK")

# 33. 力矩硬阈值：滤波力矩持续超限 -> -X 退刀并闩锁；过零不立即解锁
ctrl.reset()
fc.align_chisel_enable = True
try:
    retreated = False
    for _ in range(int(0.2 / fc.cycle_s)):  # 滤波通道需 ~40ms 收敛后触发
        rk = ctrl.update([-30.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                         torque_tcp=[0.0, 10.0, 0.0])
        if ctrl.tripped:
            assert rk["RKorr.X"] < -0.5 * fc.per_cycle_max_mm, f"应沿 -X 全速退刀: {rk}"
            assert abs(rk["RKorr.B"]) < 1e-9 and abs(rk["RKorr.C"]) < 1e-9
            retreated = True
    assert ctrl.tripped, "My=10 N·m 持续 0.2s 应触发力矩卡滞保护"
    assert retreated
    for _ in range(10):
        ctrl.update([-1.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                    torque_tcp=[0.0, 0.5, 0.0])
        assert ctrl.tripped, "力矩保护过零不得立即解锁"
    for _ in range(int(0.5 / fc.cycle_s)):
        ctrl.update([-1.0, 0.0, 0.0], ANGLES, target, mode="chisel",
                    torque_tcp=[0.0, 0.5, 0.0])
    assert not ctrl.tripped, "持续卸荷 0.2s 后应解除保护"
finally:
    fc.align_chisel_enable = False
print("33. 力矩卡滞保护（滤波判据）：触发退刀、闩锁、卸荷解除，OK")

# 34. 钻孔对中：B 输出、平移恒 0，OV 速度律不受影响
ctrl.reset()
fc.align_drill_enable = True
try:
    for _ in range(int(0.3 / fc.cycle_s)):
        rk = ctrl.update([-40.0, 0.0, 0.0], ANGLES, target, mode="drill",
                         torque_tcp=[0.0, 1.5, 0.0])
        assert abs(rk["RKorr.X"]) < 1e-9 and abs(rk["RKorr.Y"]) < 1e-9 and abs(rk["RKorr.Z"]) < 1e-9
        assert rk["RKorr.B"] >= 0.0
    assert ctrl.corr_cumulative_deg[1] > 1e-9, "钻孔对中应产生 B 叠加"
    assert ctrl.ov_pro_pct < 100.0, "OV 律应照常运行（40N 降倍率）"
finally:
    fc.align_drill_enable = False
print("34. 钻孔轴线对中：B 输出、平移恒 0、OV 律不受影响，OK")

# 35. 关闭对中后旋转叠加缓撤到 0
for _ in range(int(0.5 / fc.cycle_s)):
    rk = ctrl.update([-40.0, 0.0, 0.0], ANGLES, target, mode="drill",
                     torque_tcp=[0.0, 0.0, 0.0])
    assert rk["RKorr.B"] <= 0.0
assert abs(ctrl.corr_cumulative_deg[1]) < 1e-9, f"旋转叠加应撤到 0: {ctrl.corr_cumulative_deg}"
assert abs(rk["RKorr.B"]) < 1e-9
print("35. 对中关闭：旋转叠加按限幅缓撤到 0，OK")

# 36. 钻轴=Z 的通道泛化（TCP 重标定后现场形态）：横向走 X/Y、对中 Mx->A
fc.axis = "Z"
fc.align_chisel_enable = True
ctrl_z = ForceController(fc)
ctrl_z.reset()
steps_lat, steps_rot = [], []
for _ in range(int(0.5 / fc.cycle_s)):
    rk = ctrl_z.update([40.0, 0.0, -30.0], ANGLES, target, mode="chisel",
                       torque_tcp=[1.5, 0.0, 0.0])
    steps_lat.append(rk["RKorr.X"])
    steps_rot.append(rk["RKorr.A"])
    assert abs(rk["RKorr.Y"]) < 1e-9 and abs(rk["RKorr.B"]) < 1e-9, f"Z 钻轴下 Y/B 不应动: {rk}"
    assert abs(rk["RKorr.Z"]) < 1e-9, f"钻轴 Z 不做平移让位: {rk}"
act_lat = [s for s in steps_lat if s != 0.0]
act_rot = [s for s in steps_rot if s != 0.0]
assert act_lat and all(s > 0 for s in act_lat), f"+X 横向力应沿 +X 让位: {act_lat[:5]}"
assert act_rot and all(s > 0 for s in act_rot), f"+Mx 应驱动 A 同号旋转: {act_rot[:5]}"
fc.axis = "X"
fc.align_chisel_enable = False
print("36. 钻轴=Z 泛化：横向 X/Y + 对中 Mx->A，OK")

print("\n全部自检通过")
