# 恒力钻孔闭环仿真：编程进给 10 mm/s（行程 60mm）+ 材料切削模型
# 切削模型：接触且推进力 > 5N 时，孔底以 k_cut * 力 的速度后退（材料被切除）
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

K_SURFACE = 200.0       # 接触刚度 N/mm
K_CUT = 0.05            # 切削率 (mm/s)/N —— 50N 时 2.5 mm/s
FEED_MM_S = 10.0        # 编程进给速度
PATH_LEN = 60.0         # 编程行程 mm
target = 50.0

ctrl = ForceController(fc)
gap = 2.0
pos_prog = 0.0          # 编程位置（沿工具 +X）
pos_corr = 0.0          # 修正累积（沿工具 +X）
hole_depth = 0.0        # 已切削深度
max_step = 0.0
peak_force = 0.0
history = []
steps = int(30.0 / fc.cycle_s)
for i in range(steps):
    feed = FEED_MM_S if pos_prog < PATH_LEN else 0.0
    pos_prog += feed * fc.cycle_s

    pos_actual = pos_prog + pos_corr
    penetration = pos_actual - gap - hole_depth
    f_x = -K_SURFACE * penetration if penetration > 0 else 0.0
    press = fc.press_sign * f_x
    if press > 5.0:
        hole_depth += K_CUT * press * fc.cycle_s   # 材料被切除，孔底后退

    rk = ctrl.update([f_x, 0.0, 0.0], ANGLES, target)
    step = [rk["RKorr.X"], rk["RKorr.Y"], rk["RKorr.Z"]]
    max_step = max(max_step, *(abs(v) for v in step))
    pos_corr += sum(s * a for s, a in zip(step, tool_x_base))
    peak_force = max(peak_force, press)
    if i % int(3.0 / fc.cycle_s) == 0 or i == steps - 1:
        history.append((i * fc.cycle_s, press, hole_depth, pos_corr, ctrl.tripped))

print("\n时间(s)  推进力(N)  孔深(mm)  修正(mm)  保护")
for t, f, d, c, tr in history:
    print("  %5.1f  %8.1f  %7.2f  %8.2f   %s" % (t, f, d, c, "触发" if tr else "-"))

print("\n=== 断言检查 ===")
assert max_step <= fc.per_cycle_max_mm + 1e-9, f"每周期限幅被突破: {max_step}"
print(f"1. 每周期修正限幅 OK (max={max_step:.4f} mm <= {fc.per_cycle_max_mm})")
print(f"2. 峰值推进力 {peak_force:.1f} N（保护阈值 {fc.max_force_n} N）")
assert peak_force < fc.max_force_n, "正常钻进不应触发保护"
assert abs(press - target) < 5.0, f"未收敛: {press}"
print(f"3. 末期推进力 {press:.1f} N ≈ 目标 {target} N，OK")
assert abs(ctrl.corr_cumulative_mm[0]) <= fc.cumulative_max_mm + 1e-9
print(f"4. 累积限幅 OK ({ctrl.corr_cumulative_mm[0]:.2f} mm <= {fc.cumulative_max_mm})")

# 钻穿：力突然消失 -> 不得前冲
for _ in range(100):
    rk = ctrl.update([0.0, 0.0, 0.0], ANGLES, target)
assert all(v == 0.0 for v in rk.values()), f"钻穿后仍输出修正: {rk}"
print("5. 钻穿后（力=0）零修正、不前冲，OK")

# 目标力 0 / 超力保护（保护动作 = 全速退刀，退刀方向 +BASE X）
assert all(v == 0.0 for v in ctrl.update([0.0, 0.0, 0.0], ANGLES, 0.0).values())
print("6. 目标力 <=0 时输出零修正，OK")
ctrl.tripped = False
rk = ctrl.update([-200.0, 0.0, 0.0], ANGLES, target)
assert rk["RKorr.X"] > 0.15, f"超力保护应全速退刀(+X): {rk}"
print("7. 超力保护触发并全速退刀，OK")

server = udp_server.RSIServer(udp_server.RSIConfig())
assert server.self_test_xml()
print("8. XML 自检通过，OK")
print("\n全部自检通过")
