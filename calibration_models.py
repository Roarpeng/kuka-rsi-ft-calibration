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
    press_sign: int = -1                   # 接触力读数符号：沿 +axis 压紧时读数为负 -> -1
    default_target_force_n: float = 50.0   # 机器人未发 target_force 时的默认目标力
    kp_mm_per_s_per_n: float = 0.08        # 比例增益：每牛误差 -> mm/s 修正速度
    ki_mm_per_s2_per_n: float = 0.3        # 积分增益
    integral_limit_n_s: float = 50.0       # 积分限幅（抗饱和）
    per_cycle_max_mm: float = 0.2          # 每周期（4ms）RKorr 限幅
    cumulative_max_mm: float = 20.0        # 累积修正限幅（防跑飞）
    deadband_n: float = 1.0                # 力误差死区
    contact_threshold_n: float = 3.0       # 接触判定阈值
    search_speed_mm_s: float = 4.0         # 未接触时的搜索速度（仅 search_before_contact=True 时生效）
    search_before_contact: bool = True     # True=打磨（接触前主动找表面）；False=钻孔（接触前零修正，路径自行进给，且钻穿后不前冲）
    advance_speed_mm_s: float = 2.0        # "加压方向"修正速度上限（防接触瞬态前冲）；退刀方向不受此限
    filter_window: int = 8                 # 力滑动平均窗口（帧，4ms/帧）
    max_force_n: float = 120.0             # 超力保护阈值
    cycle_s: float = 0.004                 # RSI 周期


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
