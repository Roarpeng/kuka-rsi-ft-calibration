from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from calibration_models import CalibrationConfig, CalibrationResult, CalibrationSample, CalibrationFileConfig, EulerTransform, ForceControlConfig, ScaleCalibration, StaticDetectionConfig


DEFAULT_CONFIG_PATH = Path("ft_calibration_config.json")


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _write_json(path: Path, payload: dict[str, Any]):
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)


def _dump_config_compact_arrays(payload: dict[str, Any]) -> str:
    """配置文件序列化：2 空格缩进，数组保持单行（与仓库手工格式一致，便于 diff）。"""
    def fmt(value: Any, indent: int) -> str:
        pad = "  " * indent
        if isinstance(value, dict):
            if not value:
                return "{}"
            inner = ",\n".join(
                f"{pad}  {json.dumps(str(k), ensure_ascii=False)}: {fmt(v, indent + 1)}"
                for k, v in value.items()
            )
            return "{\n" + inner + "\n" + pad + "}"
        return json.dumps(value, ensure_ascii=False)

    return fmt(payload, 0) + "\n"


def _parse_transform(payload: dict[str, Any]) -> EulerTransform:
    return EulerTransform(
        translation_m=list(payload.get("translation_m", [0.0, 0.0, 0.0])),
        rotation_deg=list(payload.get("rotation_deg", [0.0, 0.0, 0.0])),
        rotation_order=payload.get("rotation_order", "ZYX"),
    )


def _parse_scale(payload: dict[str, Any]) -> ScaleCalibration:
    return ScaleCalibration(
        force_scales=list(payload.get("force_scales", [1.0, 1.0, 1.0])),
        torque_scales=list(payload.get("torque_scales", [1.0, 1.0, 1.0])),
        raw_bias=list(payload.get("raw_bias", [0.0] * 6)),
        torque_unit=payload.get("torque_unit", "N_m"),
    )


def _parse_static_detection(payload: dict[str, Any]) -> StaticDetectionConfig:
    return StaticDetectionConfig(
        window_size=int(payload.get("window_size", 50)),
        min_dwell_seconds=float(payload.get("min_dwell_seconds", 0.4)),
        position_threshold_m=float(payload.get("position_threshold_m", 0.0008)),
        angle_threshold_deg=float(payload.get("angle_threshold_deg", 0.25)),
        force_std_threshold_n=float(payload.get("force_std_threshold_n", 1.5)),
        torque_std_threshold_nm=float(payload.get("torque_std_threshold_nm", 0.08)),
        min_pose_separation_deg=float(payload.get("min_pose_separation_deg", 12.0)),
        min_samples=int(payload.get("min_samples", 12)),
        max_samples=int(payload.get("max_samples", 12)),
    )


def _parse_files(payload: dict[str, Any]) -> CalibrationFileConfig:
    return CalibrationFileConfig(
        calibration_path=payload.get("calibration_path", "ft_calibration.json"),
        sample_path=payload.get("sample_path", "ft_calibration_samples.json"),
    )


def _parse_force_control(payload: dict[str, Any]) -> ForceControlConfig:
    defaults = ForceControlConfig()
    return ForceControlConfig(
        axis=str(payload.get("axis", defaults.axis)),
        press_sign=int(payload.get("press_sign", defaults.press_sign)),
        press_motion_sign=int(payload.get("press_motion_sign", defaults.press_motion_sign)),
        default_target_force_n=float(payload.get("default_target_force_n", defaults.default_target_force_n)),
        kp_mm_per_s_per_n=float(payload.get("kp_mm_per_s_per_n", defaults.kp_mm_per_s_per_n)),
        ki_mm_per_s2_per_n=float(payload.get("ki_mm_per_s2_per_n", defaults.ki_mm_per_s2_per_n)),
        integral_limit_n_s=float(payload.get("integral_limit_n_s", defaults.integral_limit_n_s)),
        per_cycle_max_mm=float(payload.get("per_cycle_max_mm", defaults.per_cycle_max_mm)),
        cumulative_max_mm=float(payload.get("cumulative_max_mm", defaults.cumulative_max_mm)),
        deadband_n=float(payload.get("deadband_n", defaults.deadband_n)),
        contact_threshold_n=float(payload.get("contact_threshold_n", defaults.contact_threshold_n)),
        search_speed_mm_s=float(payload.get("search_speed_mm_s", defaults.search_speed_mm_s)),
        search_before_contact=bool(payload.get("search_before_contact", defaults.search_before_contact)),
        advance_speed_mm_s=float(payload.get("advance_speed_mm_s", defaults.advance_speed_mm_s)),
        path_feed_mm_s=float(payload.get("path_feed_mm_s", defaults.path_feed_mm_s)),
        path_feed_hold_s=float(payload.get("path_feed_hold_s", defaults.path_feed_hold_s)),
        contact_lost_s=float(payload.get("contact_lost_s", defaults.contact_lost_s)),
        trip_clear_s=float(payload.get("trip_clear_s", defaults.trip_clear_s)),
        rkorr_frame=str(payload.get("rkorr_frame", defaults.rkorr_frame)).lower(),
        filter_window=int(payload.get("filter_window", defaults.filter_window)),
        filter_median_window=int(payload.get(
            "filter_median_window",
            payload.get("filter_window", defaults.filter_median_window),
        )),
        filter_protect_window=int(payload.get("filter_protect_window", defaults.filter_protect_window)),
        filter_lpf_hz=float(payload.get("filter_lpf_hz", defaults.filter_lpf_hz)),
        max_force_n=float(payload.get("max_force_n", defaults.max_force_n)),
        cycle_s=float(payload.get("cycle_s", defaults.cycle_s)),
        default_ov_pro=float(payload.get("default_ov_pro", defaults.default_ov_pro)),
        ov_pro_slew_pct=float(payload.get("ov_pro_slew_pct", defaults.ov_pro_slew_pct)),
        chisel_lateral_deadband_n=float(payload.get("chisel_lateral_deadband_n", defaults.chisel_lateral_deadband_n)),
        chisel_lateral_gain_mm_per_s_per_n=float(payload.get(
            "chisel_lateral_gain_mm_per_s_per_n", defaults.chisel_lateral_gain_mm_per_s_per_n
        )),
        chisel_lateral_max_mm=float(payload.get("chisel_lateral_max_mm", defaults.chisel_lateral_max_mm)),
        chisel_lateral_trip_n=float(payload.get("chisel_lateral_trip_n", defaults.chisel_lateral_trip_n)),
        chisel_lateral_sign=int(payload.get("chisel_lateral_sign", defaults.chisel_lateral_sign)),
        chisel_lateral_median_window=int(payload.get(
            "chisel_lateral_median_window", defaults.chisel_lateral_median_window
        )),
        chisel_lateral_lpf_hz=float(payload.get("chisel_lateral_lpf_hz", defaults.chisel_lateral_lpf_hz)),
    )


def load_config(path: str | Path = DEFAULT_CONFIG_PATH) -> CalibrationConfig:
    config_path = Path(path)
    if not config_path.exists():
        return CalibrationConfig()
    payload = _read_json(config_path)
    return CalibrationConfig(
        mode=payload.get("mode", "record_only"),
        gravity_mps2=float(payload.get("gravity_mps2", 9.81)),
        rsi_rotation_order=payload.get("rsi_rotation_order", "ZYX"),
        sensor_to_flange=_parse_transform(payload.get("sensor_to_flange", {})),
        flange_to_tcp=_parse_transform(payload.get("flange_to_tcp", {})),
        scale=_parse_scale(payload.get("scale", {})),
        static_detection=_parse_static_detection(payload.get("static_detection", {})),
        force_control=_parse_force_control(payload.get("force_control", {})),
        files=_parse_files(payload.get("files", {})),
    )


def save_force_control_updates(
    path: str | Path, updates: dict[str, float | int]
) -> dict[str, Any]:
    """把若干 force_control 参数合并落盘（保留其他段落与字段），返回更新后的段。

    供 Web 层 /api/force_config 持久化在线设定；键白名单与范围由调用方校验。
    """
    config_path = Path(path)
    payload: dict[str, Any] = {}
    if config_path.exists():
        payload = _read_json(config_path)
    section = payload.setdefault("force_control", {})
    if not isinstance(section, dict):
        raise ValueError("ft_calibration_config.json 的 force_control 段损坏（非对象）")
    section.update(updates)
    with config_path.open("w", encoding="utf-8") as handle:
        handle.write(_dump_config_compact_arrays(payload))
    return section


def save_calibration_result(path: str | Path, result: CalibrationResult):
    _write_json(Path(path), asdict(result))


def load_calibration_result(path: str | Path) -> CalibrationResult | None:
    result_path = Path(path)
    if not result_path.exists():
        return None
    payload = _read_json(result_path)
    return CalibrationResult(
        created_at=payload["created_at"],
        sample_count=int(payload["sample_count"]),
        gravity_base_n=list(payload["gravity_base_n"]),
        force_bias_n=list(payload["force_bias_n"]),
        torque_bias_nm=list(payload["torque_bias_nm"]),
        com_in_sensor_m=list(payload["com_in_sensor_m"]),
        mass_kg=float(payload["mass_kg"]),
        residual_force_rms_n=float(payload["residual_force_rms_n"]),
        residual_torque_rms_nm=float(payload["residual_torque_rms_nm"]),
        sensor_to_tcp_rotation=[list(row) for row in payload["sensor_to_tcp_rotation"]],
        sensor_to_tcp_translation_m=list(payload["sensor_to_tcp_translation_m"]),
        rsi_rotation_order=payload["rsi_rotation_order"],
        gravity_mps2=float(payload["gravity_mps2"]),
        gravity_matrix_n=([list(row) for row in payload["gravity_matrix_n"]] if payload.get("gravity_matrix_n") else None),
        notes=list(payload.get("notes", [])),
    )


def save_samples(path: str | Path, samples: list[CalibrationSample]):
    _write_json(Path(path), {"samples": [asdict(sample) for sample in samples]})
