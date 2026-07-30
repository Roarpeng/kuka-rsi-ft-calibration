from __future__ import annotations

from collections import deque
from dataclasses import asdict
from datetime import datetime
from typing import Any

from calibration_io import save_calibration_result, save_samples
from calibration_math import (
    compose_transform,
    convert_raw_to_wrench,
    euler_to_matrix,
    fit_gravity_model,
    matrix_multiply,
    matrix_transpose,
    matrix_vector_multiply,
    norm,
    vector_mean,
    vector_std,
)
from calibration_models import CalibratedWrench, CalibrationConfig, CalibrationResult, CalibrationSample


class CalibrationRunner:
    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.window = deque(maxlen=config.static_detection.window_size)
        self.current_segment: list[dict[str, Any]] = []
        self.samples: list[CalibrationSample] = []
        self.calibration_result: CalibrationResult | None = None
        self.last_sample_angles: list[float] | None = None
        self.sensor_to_tcp_rotation, self.sensor_to_tcp_translation_m = self._build_sensor_to_tcp()

    def _build_sensor_to_tcp(self) -> tuple[list[list[float]], list[float]]:
        sensor_to_flange = self.config.sensor_to_flange
        flange_to_tcp = self.config.flange_to_tcp
        rotation_sensor_flange = euler_to_matrix(sensor_to_flange.rotation_deg, sensor_to_flange.rotation_order)
        translation_sensor_flange = sensor_to_flange.translation_m
        rotation_flange_tcp = euler_to_matrix(flange_to_tcp.rotation_deg, flange_to_tcp.rotation_order)
        translation_flange_tcp = flange_to_tcp.translation_m
        rotation_sensor_tcp, translation_sensor_tcp = compose_transform(
            rotation_sensor_flange,
            translation_sensor_flange,
            rotation_flange_tcp,
            translation_flange_tcp,
        )
        return rotation_sensor_tcp, translation_sensor_tcp

    def process_frame(self, frame: dict[str, Any]) -> dict[str, Any]:
        raw_wrench = [
            frame["Fx_raw"], frame["Fy_raw"], frame["Fz_raw"],
            frame["Mx_raw"], frame["My_raw"], frame["Mz_raw"],
        ]
        sensor_wrench = convert_raw_to_wrench(raw_wrench, self.config.scale)
        frame["sensor_wrench"] = sensor_wrench
        frame["sample_status"] = "streaming"

        self.window.append(frame)
        if self.config.mode == "calibration_collect":
            frame["sample_status"] = self._update_sampling(frame)
        elif self.config.mode == "calibrated_runtime" and self.calibration_result is not None:
            frame["calibrated_wrench"] = self._compensate(sensor_wrench, frame)
            frame["sample_status"] = "compensated"
        return frame

    def _update_sampling(self, frame: dict[str, Any]) -> str:
        in_position = frame.get("in_position", False)

        # 如果机器人未到位，继续等待
        if not in_position:
            if self.current_segment:
                self.current_segment.clear()
            return "moving"

        # 机器人已到位，开始采集当前帧
        self.current_segment.append(frame)
        duration_seconds = self._segment_duration_seconds(self.current_segment)

        if duration_seconds < self.config.static_detection.min_dwell_seconds:
            return "holding"

        sample = self._build_sample(self.current_segment)
        self.current_segment.clear()
        if not self._sample_is_distinct(sample):
            return "duplicate_pose"

        self.samples.append(sample)
        self.last_sample_angles = sample.tcp_angles_deg
        save_samples(self.config.files.sample_path, self.samples)

        if len(self.samples) >= self.config.static_detection.min_samples:
            self.calibration_result = fit_gravity_model(
                samples=self.samples,
                gravity_mps2=self.config.gravity_mps2,
                sensor_to_tcp_rotation=self.sensor_to_tcp_rotation,
                sensor_to_tcp_translation_m=self.sensor_to_tcp_translation_m,
                rsi_rotation_order=self.config.rsi_rotation_order,
            )
            save_calibration_result(self.config.files.calibration_path, self.calibration_result)
            if self.config.mode == "calibration_collect":
                self.config.mode = "calibrated_runtime"
            return "calibration_ready"
        return "sample_saved"

    def _window_is_static(self) -> bool:
        if len(self.window) < self.window.maxlen:
            return False
        positions = [[frame["Act_X"], frame["Act_Y"], frame["Act_Z"]] for frame in self.window]
        angles = [[frame["Act_A"], frame["Act_B"], frame["Act_C"]] for frame in self.window]
        wrenches = [frame["sensor_wrench"] for frame in self.window]

        position_spread = max(self._axis_span(positions, axis) for axis in range(3))
        angle_spread = max(self._axis_span(angles, axis) for axis in range(3))
        wrench_std = vector_std(wrenches, vector_mean(wrenches))
        force_std = max(wrench_std[:3])
        torque_std = max(wrench_std[3:6])

        return (
            position_spread <= self.config.static_detection.position_threshold_m and
            angle_spread <= self.config.static_detection.angle_threshold_deg and
            force_std <= self.config.static_detection.force_std_threshold_n and
            torque_std <= self.config.static_detection.torque_std_threshold_nm
        )

    def _build_sample(self, segment: list[dict[str, Any]]) -> CalibrationSample:
        raw_vectors = [
            [frame["Fx_raw"], frame["Fy_raw"], frame["Fz_raw"], frame["Mx_raw"], frame["My_raw"], frame["Mz_raw"]]
            for frame in segment
        ]
        sensor_vectors = [frame["sensor_wrench"] for frame in segment]
        raw_mean = vector_mean(raw_vectors)
        sensor_mean = vector_mean(sensor_vectors)
        sensor_std = vector_std(sensor_vectors, sensor_mean)
        first_frame = segment[0]
        last_frame = segment[-1]
        return CalibrationSample(
            timestamp=last_frame["timestamp"],
            frame_count=len(segment),
            duration_seconds=self._segment_duration_seconds(segment),
            tcp_position_m=[last_frame["Act_X"], last_frame["Act_Y"], last_frame["Act_Z"]],
            tcp_angles_deg=[last_frame["Act_A"], last_frame["Act_B"], last_frame["Act_C"]],
            raw_mean=raw_mean,
            sensor_mean=sensor_mean,
            sensor_std=sensor_std,
        )

    def _sample_is_distinct(self, sample: CalibrationSample) -> bool:
        if self.last_sample_angles is None:
            return True
        delta = [sample.tcp_angles_deg[index] - self.last_sample_angles[index] for index in range(3)]
        return norm(delta) >= self.config.static_detection.min_pose_separation_deg

    def _segment_duration_seconds(self, segment: list[dict[str, Any]]) -> float:
        if len(segment) < 2:
            return 0.0
        start = datetime.strptime(segment[0]["timestamp"], "%Y-%m-%d %H:%M:%S.%f")
        end = datetime.strptime(segment[-1]["timestamp"], "%Y-%m-%d %H:%M:%S.%f")
        return (end - start).total_seconds()

    def _axis_span(self, vectors: list[list[float]], axis: int) -> float:
        values = [vector[axis] for vector in vectors]
        return max(values) - min(values)

    def _compensate(self, sensor_wrench: list[float], frame: dict[str, Any]) -> CalibratedWrench:
        from calibration_math import compensate_wrench

        compensated_force_sensor, compensated_torque_sensor, compensated_force_tcp, compensated_torque_tcp = compensate_wrench(
            raw_wrench_sensor=sensor_wrench,
            tcp_angles_deg=[frame["Act_A"], frame["Act_B"], frame["Act_C"]],
            calibration_result=self.calibration_result,
        )
        return CalibratedWrench(
            force_sensor=compensated_force_sensor,
            torque_sensor=compensated_torque_sensor,
            force_tcp=compensated_force_tcp,
            torque_tcp=compensated_torque_tcp,
        )
