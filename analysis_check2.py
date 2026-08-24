# 进一步诊断：42N 残差的来源
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

def fit_report(name, build_rows, targets, n_params):
    A, b = [], []
    for i in range(len(angs)):
        for row, t in build_rows(i, targets[i]):
            A.append(row); b.append(t)
    A, b = np.array(A), np.array(b)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    res = b - A @ sol
    print("%s: RMS = %.4f  (cond=%.1f)" % (name, np.sqrt(np.mean(res**2)), np.linalg.cond(A)))
    return sol, res

# 模型 A: 严格物理 F = bias + R^T g_base （6 参数）
def rows_A(i, f):
    R = rz(*angs[i][0:1]) @ ry(angs[i][1]) @ rx(angs[i][2])
    Rt = R.T
    return [(np.r_[np.eye(3)[ax], Rt[ax]], f[ax]) for ax in range(3)]
fit_report("A 物理模型 bias+R^T g", rows_A, F, 6)

# 模型 B: 自由 3x3 矩阵 F = bias + S @ d, d = R^T[0,0,-1] （12 参数）
def rows_B(i, f):
    R = rz(angs[i][0]) @ ry(angs[i][1]) @ rx(angs[i][2])
    d = R.T @ np.array([0, 0, -1.0])
    out = []
    for ax in range(3):
        row = np.zeros(12); row[ax] = 1.0; row[3 + ax*3: 6 + ax*3] = d
        out.append((row, f[ax]))
    return out
solB, resB = fit_report("B 自由3x3 F=bias+S d", rows_B, F, 12)
S = solB[3:].reshape(3, 3)
print("  S =\n", np.round(S, 2))
print("  |S列| =", np.round(np.linalg.norm(S, axis=0), 2))
u, sv, vt = np.linalg.svd(S)
print("  S 奇异值:", np.round(sv, 2))

# 模型 C: 逐样本看残差（模型B下），找离群点
per_sample = resB.reshape(len(angs), 3)
for i, (abc, r) in enumerate(zip(angs, per_sample)):
    print("  样本%2d A/B/C=%7.1f/%6.1f/%6.1f  残差=(%7.2f,%7.2f,%7.2f)" % (
        i + 1, abc[0], abc[1], abc[2], r[0], r[1], r[2]))

# 模型 D: 力矩 tau = bias_t + r x (F_meas - bias_f)  用实测力（最鲁棒）
bias_f = solB[:3]
def rows_D(i, m):
    fg = F[i] - bias_f
    S_ = np.array([[0, -fg[2], fg[1]], [fg[2], 0, -fg[0]], [-fg[1], fg[0], 0]])
    out = []
    for ax in range(3):
        row = np.zeros(6); row[ax] = 1.0; row[3:] = -S_[ax]
        out.append((row, m[ax]))
    return out
solD, resD = fit_report("D 力矩 tau=bias+r x F_meas", rows_D, M, 6)
print("  torque bias:", np.round(solD[:3], 4), " r(m):", np.round(solD[3:], 4))
per = resD.reshape(len(angs), 3)
for i, r in enumerate(per):
    print("  样本%2d 力矩残差=(%7.3f,%7.3f,%7.3f)" % (i + 1, r[0], r[1], r[2]))

# 检查样本内噪声 vs 拟合残差
stds = np.array([s["sensor_std"][:3] for s in samples])
print("\n样本内力噪声 std 平均:", np.round(stds.mean(axis=0), 3), "N")
