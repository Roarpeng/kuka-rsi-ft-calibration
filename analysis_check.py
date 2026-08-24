# 验证：正确模型 vs 现有实现 的残差对比
import json, math
import numpy as np

def rz(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])

def ry(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])

def rx(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])

samples = json.load(open("ft_calibration_samples.json", encoding="utf-8"))["samples"]
angs = [np.array(s["tcp_angles_deg"]) for s in samples]
F = np.array([s["sensor_mean"][:3] for s in samples])
M = np.array([s["sensor_mean"][3:] for s in samples])

# ---------- 正确模型: F = bias + R^T @ g_base ----------
A_rows, b_rows = [], []
for abc, f in zip(angs, F):
    R = rz(abc[0]) @ ry(abc[1]) @ rx(abc[2])   # base<-tcp
    Rt = R.T
    for ax in range(3):
        row = np.zeros(6)
        row[ax] = 1.0
        row[3:] = Rt[ax]      # 每轴用 R^T 的对应行
        A_rows.append(row); b_rows.append(f[ax])
sol, *_ = np.linalg.lstsq(np.array(A_rows), np.array(b_rows), rcond=None)
bias_c, g_c = sol[:3], sol[3:]
res = []
for abc, f in zip(angs, F):
    R = rz(abc[0]) @ ry(abc[1]) @ rx(abc[2])
    pred = bias_c + R.T @ g_c
    res.extend(f - pred)
res = np.array(res)
print("=== 正确模型 ===")
print("force bias:", np.round(bias_c, 3))
print("gravity_base_n:", np.round(g_c, 3), "|g| =", round(np.linalg.norm(g_c), 3),
      "-> mass =", round(np.linalg.norm(g_c) / 9.81, 3), "kg")
print("residual force RMS: %.4f N" % np.sqrt(np.mean(res**2)))

# ---------- 现有实现（buggy）模型: F_axis = bias_axis + dir·g ----------
A2, b2 = [], []
for abc, f in zip(angs, F):
    R = rz(abc[0]) @ ry(abc[1]) @ rx(abc[2])
    d = R.T @ np.array([0, 0, -1.0])
    for ax in range(3):
        row = np.zeros(6); row[ax] = 1.0; row[3:] = d
        A2.append(row); b2.append(f[ax])
sol2, *_ = np.linalg.lstsq(np.array(A2), np.array(b2), rcond=None)
res2 = np.array(b2) - np.array(A2) @ sol2
print("\n=== 现有实现模型 ===")
print("gravity_base_n:", np.round(sol2[3:], 3))
print("residual force RMS: %.4f N" % np.sqrt(np.mean(res2**2)))

# ---------- 力矩：正确模型 tau = bias_t + r_com x (R^T g) ----------
g_unit = g_c / np.linalg.norm(g_c)
A3, b3 = [], []
for abc, m in zip(angs, M):
    R = rz(abc[0]) @ ry(abc[1]) @ rx(abc[2])
    gs = R.T @ g_c
    S = np.array([[0, -gs[2], gs[1]], [gs[2], 0, -gs[0]], [-gs[1], gs[0], 0]])
    # tau = bias + skew(r) @ gs = bias - skew(gs) @ r
    for ax in range(3):
        row = np.zeros(6); row[ax] = 1.0; row[3:] = -S[ax]
        A3.append(row); b3.append(m[ax])
sol3, *_ = np.linalg.lstsq(np.array(A3), np.array(b3), rcond=None)
tb, r_com = sol3[:3], sol3[3:]
res3 = np.array(b3) - np.array(A3) @ sol3
print("\n=== 力矩正确模型 ===")
print("torque bias:", np.round(tb, 4))
print("com_in_sensor_m:", np.round(r_com / np.linalg.norm(g_c) * 9.81 * 0 + r_com, 4),
      "(单位: N·m 系数, 除以|g|=%.1f 得米)" % np.linalg.norm(g_c))
print("com (m):", np.round(r_com / np.linalg.norm(g_c), 4))
print("residual torque RMS: %.4f N*m" % np.sqrt(np.mean(res3**2)))

# ---------- 姿态激励分析 ----------
print("\n=== 重力方向在传感器系中的分布（激励多样性）===")
dirs = []
for abc in angs:
    R = rz(abc[0]) @ ry(abc[1]) @ rx(abc[2])
    dirs.append(R.T @ np.array([0, 0, -1.0]))
dirs = np.array(dirs)
print("g_dir 范围: x [%.3f, %.3f], y [%.3f, %.3f], z [%.3f, %.3f]" % (
    dirs[:,0].min(), dirs[:,0].max(), dirs[:,1].min(), dirs[:,1].max(), dirs[:,2].min(), dirs[:,2].max()))
D = np.hstack([np.ones((len(dirs),1)), dirs])
print("设计矩阵 [1, g_dir] 条件数: %.1f" % np.linalg.cond(D))

# 力数据自身变化范围
print("F 范围: x [%.1f, %.1f], y [%.1f, %.1f], z [%.1f, %.1f]" % (
    F[:,0].min(), F[:,0].max(), F[:,1].min(), F[:,1].max(), F[:,2].min(), F[:,2].max()))
