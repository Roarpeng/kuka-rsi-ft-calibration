# 用修复后的模型重跑现有 12 点标定，并与旧结果对比
import json
from calibration_io import load_config, save_calibration_result, load_calibration_result
from calibration_math import fit_gravity_model
from calibration_models import CalibrationSample

config = load_config("ft_calibration_config.json")
config.mode = "calibration_collect"

payload = json.load(open("ft_calibration_samples.json", encoding="utf-8"))
samples = [CalibrationSample(**s) for s in payload["samples"]]

# 复现 runner 里的 sensor->tcp 变换（identity rotation, translation 由 config 合成）
from calibration_runner import CalibrationRunner
runner = CalibrationRunner(config)
rot, trans = runner.sensor_to_tcp_rotation, runner.sensor_to_tcp_translation_m

result = fit_gravity_model(
    samples=samples,
    gravity_mps2=config.gravity_mps2,
    sensor_to_tcp_rotation=rot,
    sensor_to_tcp_translation_m=trans,
    rsi_rotation_order=config.rsi_rotation_order,
)
save_calibration_result("ft_calibration_fixed.json", result)

old = json.load(open("ft_calibration.json", encoding="utf-8"))

print("=" * 60)
print("对比（同一批 12 点数据）")
print("=" * 60)
print(f"{'指标':<24}{'旧结果':>14}{'修复后':>14}")
print(f"{'力残差 RMS (N)':<24}{old['residual_force_rms_n']:>14.3f}{result.residual_force_rms_n:>14.3f}")
print(f"{'力矩残差 RMS (N·m)':<24}{old['residual_torque_rms_nm']:>14.3f}{result.residual_torque_rms_nm:>14.3f}")
print(f"{'质量估计 (kg)':<24}{old['mass_kg']:>14.3f}{result.mass_kg:>14.3f}")
print()
print("修复后参数：")
print("  force_bias_n  =", [round(v, 3) for v in result.force_bias_n])
print("  torque_bias_nm=", [round(v, 4) for v in result.torque_bias_nm])
print("  com_in_sensor =", [round(v, 4) for v in result.com_in_sensor_m], "m")
print("  gravity_base_n=", [round(v, 2) for v in result.gravity_base_n],
      " |g| =", round(sum(v * v for v in result.gravity_base_n) ** 0.5, 2), "N")
print("  gravity_matrix_n (S, N):")
for row in result.gravity_matrix_n:
    print("   ", [round(v, 2) for v in row])
print()
print("结果已保存: ft_calibration_fixed.json")

# 用修复后结果做逐样本补偿残差检查
from calibration_math import compensate_wrench
print()
print("逐样本补偿后残余（传感器系，应为小量）：")
for i, s in enumerate(samples):
    cf, ct, _, _ = compensate_wrench(s.sensor_mean, s.tcp_angles_deg, result)
    print("  样本%2d A/B/C=%7.1f/%6.1f/%6.1f  dF=(%6.2f,%6.2f,%6.2f) N  dM=(%6.3f,%6.3f,%6.3f) N·m" % (
        i + 1, s.tcp_angles_deg[0], s.tcp_angles_deg[1], s.tcp_angles_deg[2],
        cf[0], cf[1], cf[2], ct[0], ct[1], ct[2]))
