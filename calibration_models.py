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
    notes: list[str] = field(default_factory=list)
