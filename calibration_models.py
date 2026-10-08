from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


Vector3 = list[float]
Vector6 = list[float]
Matrix3 = list[list[float]]


@dataclass
class ScaleCalibration:
    force_scales: Vector3 = field(default_factory=lambda: [1.0, 1.0, 1.0])
    torque_scales: Vector3 = field(default_factory=lambda: [1.0, 1.0, 1.0])
    raw_bias: Vector6 = field(default_factory=lambda: [0.0] * 6)
    torque_unit: str = "N_m"


@dataclass
class EulerTransform:
    translation_m: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation_deg: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation_order: str = "ZYX"


@dataclass
class StaticDetectionConfig:
    window_size: int = 50
    min_dwell_seconds: float = 0.4
    position_threshold_m: float = 0.0008
    angle_threshold_deg: float = 0.25
    force_std_threshold_n: float = 1.5
    torque_std_threshold_nm: float = 0.08
    min_pose_separation_deg: float = 12.0
    min_samples: int = 12
    max_samples: int = 12


@dataclass
class ForceControlConfig:
    """恒力打磨控制器参数。"""
    axis: str = "X"                        # 压紧轴（工具系）
    press_sign: int = -1                   # 接触力读数符号：压紧时该轴读数为负 -> -1
    press_motion_sign: int = 1             # 增大压紧力的运动方向（工具系）：+1 = 沿工具 +axis 进给（退刀为 -X）
    default_target_force_n: float = 50.0   # 机器人未发 target_force 时的默认目标力
    kp_mm_per_s_per_n: float = 0.08        # 比例增益：每牛误差 -> mm/s 修正速度
    ki_mm_per_s2_per_n: float = 0.3        # 积分增益
    integral_limit_n_s: float = 50.0       # 积分限幅（抗饱和）
    per_cycle_max_mm: float = 0.08         # 每拍 RKorr 增量限幅（0.08mm/4ms=20mm/s）；PosCorr Lim 是总行程不是这一项
    cumulative_max_mm: float = 80.0        # 内部总偏移限幅；须 ≤ RSIVisual POSCORRMON.MaxTrans
    deadband_n: float = 1.0                # 力误差死区
    contact_threshold_n: float = 3.0       # 接触判定阈值
    search_speed_mm_s: float = 4.0         # 未接触时的搜索速度（仅 search_before_contact=True 时生效）
    search_before_contact: bool = True     # True=打磨（接触前主动找表面）；False=钻孔（接触前零修正，路径自行进给，且钻穿后不前冲）
    advance_speed_mm_s: float = 4.0        # 加压方向净 TCP 速度上限；力在快速爬升时会被置 0，避免刚性表面顶死
    path_feed_mm_s: float = 10.0           # 编程路径沿压紧方向的进给（mm/s），接触后由 RKorr 抵消；须与 $VEL.CP 一致
    path_feed_hold_s: float = 0.08         # 接触中 Act 反向（弹跳）时，在此时间内仍按编程进给抵消，避免前冲
    contact_lost_s: float = 0.024          # 连续卸荷这么久才退出接触；短于 30Hz 半周期则会被弹跳抖掉
    trip_clear_s: float = 0.20             # 超力解除须连续卸荷这么久；单帧过零不得松手
    rkorr_frame: str = "tool"              # 与 RSIVisual PosCorr.RefCorrSys 对齐：tool 或 base
    filter_window: int = 5                 # 兼容旧配置：默认当作 filter_median_window
    filter_median_window: int = 5          # 控制通道中值窗口（帧），抑制 1～2 拍脉冲
    filter_protect_window: int = 3         # 接触判定短中值（帧），忽略单帧毛刺
    filter_lpf_hz: float = 8.0             # 控制通道一阶低通截止（Hz），压掉 ~30Hz 弹跳
    max_force_n: float = 120.0             # 超力保护阈值
    cycle_s: float = 0.004                 # RSI 周期
    default_ov_pro: float = 100.0          # 非钻孔（凿击占位/叠加测试）时回发的 $OV_PRO（%）
    ov_pro_slew_pct: float = 2.0           # 钻孔中每拍倍率变化上限（2%/4ms）；结束钻孔时直接拉回默认
    # ---- 凿击（RobotStatus=TRUE）：X 仍按 OV_PRO 恒力压紧；Y/Z 横向零力让位 ----
    chisel_lateral_deadband_n: float = 15.0   # 横向让位启动阈值（N）；须高于空载残差 5~12N，否则空载慢漂
    chisel_lateral_gain_mm_per_s_per_n: float = 0.04  # 让位速度增益：超出死区每牛 -> mm/s 漂移
    chisel_lateral_max_mm: float = 20.0       # 横向让位行程上限（单轴累计；与 X 共享 80mm PosCorrMon 总限）
    chisel_lateral_trip_n: float = 100.0      # 横向卡滞硬阈值（原始力锁存 -> 沿 -X 全速退刀）
    chisel_lateral_sign: int = 1              # 让位方向 = 符号 * 读数方向；读数=工件对工具作用力，同号(+1)让位即卸载
    chisel_lateral_median_window: int = 9     # 横向中值窗口（帧）：压掉单次凿击冲击尖峰
    chisel_lateral_lpf_hz: float = 4.0        # 横向低通截止（Hz）：比 X 更缓，只追持续卡滞力


@dataclass
class CalibrationFileConfig:
    calibration_path: str = "ft_calibration.json"
    sample_path: str = "ft_calibration_samples.json"


@dataclass
class CalibrationConfig:
    mode: str = "record_only"
    gravity_mps2: float = 9.81
    rsi_rotation_order: str = "ZYX"
    sensor_to_flange: EulerTransform = field(default_factory=EulerTransform)
    flange_to_tcp: EulerTransform = field(default_factory=EulerTransform)
    scale: ScaleCalibration = field(default_factory=ScaleCalibration)
    static_detection: StaticDetectionConfig = field(default_factory=StaticDetectionConfig)
    force_control: ForceControlConfig = field(default_factory=ForceControlConfig)
    files: CalibrationFileConfig = field(default_factory=CalibrationFileConfig)


@dataclass
class CalibratedWrench:
    force_sensor: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    torque_sensor: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    force_tcp: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    torque_tcp: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class CalibrationSample:
    timestamp: str = ""
    frame_count: int = 0
    duration_seconds: float = 0.0
    tcp_position_m: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tcp_angles_deg: Vector3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    raw_mean: Vector6 = field(default_factory=lambda: [0.0] * 6)
    sensor_mean: Vector6 = field(default_factory=lambda: [0.0] * 6)
    sensor_std: Vector6 = field(default_factory=lambda: [0.0] * 6)


@dataclass
class CalibrationResult:
    created_at: str
    sample_count: int
    gravity_base_n: Vector3
    force_bias_n: Vector3
    torque_bias_nm: Vector3
    com_in_sensor_m: Vector3
    mass_kg: float
    residual_force_rms_n: float
    residual_torque_rms_nm: float
    sensor_to_tcp_rotation: Matrix3
    sensor_to_tcp_translation_m: Vector3
    rsi_rotation_order: str
    gravity_mps2: float
    gravity_matrix_n: Optional[Matrix3] = None
    notes: list[str] = field(default_factory=list)
