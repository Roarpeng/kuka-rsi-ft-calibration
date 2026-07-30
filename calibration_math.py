from __future__ import annotations

import math
from dataclasses import asdict
from typing import Iterable

from calibration_models import CalibrationResult, CalibrationSample, Matrix3, ScaleCalibration, Vector3, Vector6


EPSILON = 1e-12


def deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0


def vector_add(a: Iterable[float], b: Iterable[float]) -> list[float]:
    return [x + y for x, y in zip(a, b)]


def vector_sub(a: Iterable[float], b: Iterable[float]) -> list[float]:
    return [x - y for x, y in zip(a, b)]


def vector_scale(values: Iterable[float], scalar: float) -> list[float]:
    return [scalar * value for value in values]


def vector_mean(vectors: list[Vector6]) -> Vector6:
    if not vectors:
        return [0.0] * 6
    length = len(vectors)
    sums = [0.0] * len(vectors[0])
    for vector in vectors:
        for index, value in enumerate(vector):
            sums[index] += value
    return [value / length for value in sums]


def vector_std(vectors: list[Vector6], mean_values: Vector6) -> Vector6:
    if not vectors:
        return [0.0] * 6
    length = len(vectors)
    sums = [0.0] * len(vectors[0])
    for vector in vectors:
        for index, value in enumerate(vector):
            delta = value - mean_values[index]
            sums[index] += delta * delta
    return [math.sqrt(value / length) for value in sums]


def dot(a: Iterable[float], b: Iterable[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def norm(values: Iterable[float]) -> float:
    return math.sqrt(dot(values, values))


def cross(a: Vector3, b: Vector3) -> Vector3:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def matrix_identity() -> Matrix3:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def matrix_transpose(matrix: Matrix3) -> Matrix3:
    return [[matrix[row][column] for row in range(3)] for column in range(3)]


def matrix_multiply(a: Matrix3, b: Matrix3) -> Matrix3:
    result = [[0.0] * 3 for _ in range(3)]
    for row in range(3):
        for column in range(3):
            result[row][column] = sum(a[row][k] * b[k][column] for k in range(3))
    return result


def matrix_vector_multiply(matrix: Matrix3, vector: Vector3) -> Vector3:
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def rotation_x(angle_deg: float) -> Matrix3:
    angle = deg_to_rad(angle_deg)
    c_value = math.cos(angle)
    s_value = math.sin(angle)
    return [
        [1.0, 0.0, 0.0],
        [0.0, c_value, -s_value],
        [0.0, s_value, c_value],
    ]


def rotation_y(angle_deg: float) -> Matrix3:
    angle = deg_to_rad(angle_deg)
    c_value = math.cos(angle)
    s_value = math.sin(angle)
    return [
        [c_value, 0.0, s_value],
        [0.0, 1.0, 0.0],
        [-s_value, 0.0, c_value],
    ]


def rotation_z(angle_deg: float) -> Matrix3:
    angle = deg_to_rad(angle_deg)
    c_value = math.cos(angle)
    s_value = math.sin(angle)
    return [
        [c_value, -s_value, 0.0],
        [s_value, c_value, 0.0],
        [0.0, 0.0, 1.0],
    ]


def euler_to_matrix(angles_deg: Vector3, order: str) -> Matrix3:
    rotation_map = {
        "X": rotation_x,
        "Y": rotation_y,
        "Z": rotation_z,
    }
    result = matrix_identity()
    upper_order = order.upper()
    if len(upper_order) != 3 or any(axis not in rotation_map for axis in upper_order):
        raise ValueError(f"Unsupported rotation order: {order}")
    for axis, angle in zip(upper_order, angles_deg):
        result = matrix_multiply(result, rotation_map[axis](angle))
    return result


def compose_transform(rotation_a: Matrix3, translation_a: Vector3, rotation_b: Matrix3, translation_b: Vector3) -> tuple[Matrix3, Vector3]:
    rotation = matrix_multiply(rotation_a, rotation_b)
    translation = vector_add(matrix_vector_multiply(rotation_a, translation_b), translation_a)
    return rotation, translation


def convert_raw_to_wrench(raw_values: Vector6, scale: ScaleCalibration) -> Vector6:
    corrected = [value - bias for value, bias in zip(raw_values, scale.raw_bias)]
    forces = [corrected[index] * scale.force_scales[index] for index in range(3)]
    torques = [corrected[index + 3] * scale.torque_scales[index] for index in range(3)]
    return forces + torques


def fit_gravity_model(samples: list[CalibrationSample], gravity_mps2: float, sensor_to_tcp_rotation: Matrix3, sensor_to_tcp_translation_m: Vector3, rsi_rotation_order: str) -> CalibrationResult:
    if len(samples) < 6:
        raise ValueError("At least 6 samples are required for calibration")

    force_rows: list[list[float]] = []
    force_targets: list[float] = []
    torque_rows: list[list[float]] = []
    torque_targets: list[float] = []

    for sample in samples:
        r_base_tcp = euler_to_matrix(sample.tcp_angles_deg, rsi_rotation_order)
        r_sensor_tcp = sensor_to_tcp_rotation
        r_base_sensor = matrix_multiply(r_base_tcp, matrix_transpose(r_sensor_tcp))
        gravity_sensor_dir = matrix_vector_multiply(matrix_transpose(r_base_sensor), [0.0, 0.0, -1.0])

        fx, fy, fz, mx, my, mz = sample.sensor_mean
        for axis in range(3):
            row = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            row[axis] = 1.0
            row[3] = gravity_sensor_dir[0]
            row[4] = gravity_sensor_dir[1]
            row[5] = gravity_sensor_dir[2]
            force_rows.append(row)
        force_targets.extend([fx, fy, fz])

        torque_rows.extend([
            [1.0, 0.0, 0.0, 0.0, gravity_sensor_dir[2], -gravity_sensor_dir[1]],
            [0.0, 1.0, 0.0, -gravity_sensor_dir[2], 0.0, gravity_sensor_dir[0]],
            [0.0, 0.0, 1.0, gravity_sensor_dir[1], -gravity_sensor_dir[0], 0.0],
        ])
        torque_targets.extend([mx, my, mz])

    force_solution = solve_least_squares(force_rows, force_targets)
    torque_solution = solve_least_squares(torque_rows, torque_targets)

    force_bias = force_solution[:3]
    gravity_base = force_solution[3:6]
    gravity_norm = norm(gravity_base)
    mass_kg = gravity_norm / gravity_mps2 if gravity_mps2 > EPSILON else 0.0
    com_in_sensor = [0.0, 0.0, 0.0]
    if gravity_norm > EPSILON:
        com_in_sensor = [value / gravity_norm for value in torque_solution[3:6]]
    torque_bias = torque_solution[:3]

    force_residuals = []
    torque_residuals = []
    for sample in samples:
        predicted_force_sensor, predicted_torque_sensor = predict_static_wrench_sensor(
            sample.tcp_angles_deg,
            gravity_base,
            force_bias,
            torque_bias,
            com_in_sensor,
            sensor_to_tcp_rotation,
            rsi_rotation_order,
        )
        measured_force = sample.sensor_mean[:3]
        measured_torque = sample.sensor_mean[3:6]
        force_residuals.extend(vector_sub(measured_force, predicted_force_sensor))
        torque_residuals.extend(vector_sub(measured_torque, predicted_torque_sensor))

    return CalibrationResult(
        created_at=samples[-1].timestamp,
        sample_count=len(samples),
        gravity_base_n=gravity_base,
        force_bias_n=force_bias,
        torque_bias_nm=torque_bias,
        com_in_sensor_m=com_in_sensor,
        mass_kg=mass_kg,
        residual_force_rms_n=rms(force_residuals),
        residual_torque_rms_nm=rms(torque_residuals),
        sensor_to_tcp_rotation=sensor_to_tcp_rotation,
        sensor_to_tcp_translation_m=sensor_to_tcp_translation_m,
        rsi_rotation_order=rsi_rotation_order,
        gravity_mps2=gravity_mps2,
        notes=[],
    )


def predict_static_wrench_sensor(tcp_angles_deg: Vector3, gravity_base_n: Vector3, force_bias_n: Vector3, torque_bias_nm: Vector3, com_in_sensor_m: Vector3, sensor_to_tcp_rotation: Matrix3, rsi_rotation_order: str) -> tuple[Vector3, Vector3]:
    r_base_tcp = euler_to_matrix(tcp_angles_deg, rsi_rotation_order)
    r_base_sensor = matrix_multiply(r_base_tcp, matrix_transpose(sensor_to_tcp_rotation))
    gravity_sensor = matrix_vector_multiply(matrix_transpose(r_base_sensor), gravity_base_n)
    torque_gravity = cross(com_in_sensor_m, gravity_sensor)
    predicted_force = vector_add(force_bias_n, gravity_sensor)
    predicted_torque = vector_add(torque_bias_nm, torque_gravity)
    return predicted_force, predicted_torque


def compensate_wrench(raw_wrench_sensor: Vector6, tcp_angles_deg: Vector3, calibration_result: CalibrationResult) -> tuple[Vector3, Vector3, Vector3, Vector3]:
    predicted_force_sensor, predicted_torque_sensor = predict_static_wrench_sensor(
        tcp_angles_deg=tcp_angles_deg,
        gravity_base_n=calibration_result.gravity_base_n,
        force_bias_n=calibration_result.force_bias_n,
        torque_bias_nm=calibration_result.torque_bias_nm,
        com_in_sensor_m=calibration_result.com_in_sensor_m,
        sensor_to_tcp_rotation=calibration_result.sensor_to_tcp_rotation,
        rsi_rotation_order=calibration_result.rsi_rotation_order,
    )
    measured_force_sensor = raw_wrench_sensor[:3]
    measured_torque_sensor = raw_wrench_sensor[3:6]
    compensated_force_sensor = vector_sub(measured_force_sensor, predicted_force_sensor)
    compensated_torque_sensor = vector_sub(measured_torque_sensor, predicted_torque_sensor)
    rotation_sensor_to_tcp = calibration_result.sensor_to_tcp_rotation
    compensated_force_tcp = matrix_vector_multiply(rotation_sensor_to_tcp, compensated_force_sensor)
    torque_with_shift = vector_add(compensated_torque_sensor, cross(calibration_result.sensor_to_tcp_translation_m, compensated_force_sensor))
    compensated_torque_tcp = matrix_vector_multiply(rotation_sensor_to_tcp, torque_with_shift)
    return compensated_force_sensor, compensated_torque_sensor, compensated_force_tcp, compensated_torque_tcp


def rms(values: Iterable[float]) -> float:
    items = list(values)
    if not items:
        return 0.0
    return math.sqrt(sum(value * value for value in items) / len(items))


def solve_least_squares(rows: list[list[float]], targets: list[float]) -> list[float]:
    columns = len(rows[0])
    ata = [[0.0] * columns for _ in range(columns)]
    atb = [0.0] * columns
    for row, target in zip(rows, targets):
        for row_index in range(columns):
            atb[row_index] += row[row_index] * target
            for column_index in range(columns):
                ata[row_index][column_index] += row[row_index] * row[column_index]
    return solve_linear_system(ata, atb)


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    augmented = [matrix[row][:] + [vector[row]] for row in range(size)]
    for pivot_index in range(size):
        pivot_row = max(range(pivot_index, size), key=lambda row: abs(augmented[row][pivot_index]))
        if abs(augmented[pivot_row][pivot_index]) < EPSILON:
            raise ValueError("Calibration matrix is singular. Collect more varied poses.")
        augmented[pivot_index], augmented[pivot_row] = augmented[pivot_row], augmented[pivot_index]
        pivot_value = augmented[pivot_index][pivot_index]
        for column in range(pivot_index, size + 1):
            augmented[pivot_index][column] /= pivot_value
        for row in range(size):
            if row == pivot_index:
                continue
            factor = augmented[row][pivot_index]
            for column in range(pivot_index, size + 1):
                augmented[row][column] -= factor * augmented[pivot_index][column]
    return [augmented[row][size] for row in range(size)]


def result_to_dict(result: CalibrationResult) -> dict:
    return asdict(result)
