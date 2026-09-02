# 自动重标定测试：不起 socket，直接驱动 RSIServer.process_frame，
# 验证"换工具→示教器跑 FT_Calibration.src→自动重标定→力控挂起/恢复"全流程：
# 1) data_collection=TRUE 边沿自动进入标定（清旧样本、切 calibration_collect）
# 2) 力控运行中进入标定时挂起力控（reset、RKorr=0、OV_PRO=100%）
# 3) 满 min_samples 自动求解、写标定文件、--force 启动时自动恢复力控
# 4) 非 --force 启动时标定完成后不建力控器
# 标定/样本文件全部写到临时目录，不污染仓库真实 ft_calibration*.json。
from __future__ import annotations

import os
import tempfile
from datetime import datetime, timedelta

from calibration_io import load_config
from calibration_math import cross, gravity_direction_sensor, matrix_vector_multiply
from calibration_models import CalibrationSample
from calibration_runner import CalibrationRunner
from force_controller import ForceController
from udp_server import RSIConfig, RSIData, RSIServer

# 合成传感器数据的"真值"模型：F = BIAS_F + S_TRUE·d，tau = BIAS_T + COM x (S_TRUE·d)
BIAS_F = [1.0, -2.0, 3.0]
S_TRUE = [[29.43, 0.0, 0.0], [0.0, 29.43, 0.0], [0.0, 0.0, 29.43]]  # 约 3 kg
BIAS_T = [0.1, -0.1, 0.2]
COM = [0.01, 0.02, 0.03]


class FrameFactory:
    """伪造递增时间戳的 RSIData（瞬时满足 min_dwell_seconds，无需真实等待）。"""

    def __init__(self, runner: CalibrationRunner):
        self._t = datetime(2026, 1, 1, 0, 0, 0)
        self._runner = runner

    def make(self, a=0.0, b=0.0, c=0.0, data_collection=False, with_force=True):
        self._t += timedelta(milliseconds=100)
        data = RSIData()
        data.timestamp = self._t.strftime("%Y-%m-%d %H:%M:%S.%f")
        data.Act_X, data.Act_Y, data.Act_Z = 900.0, -80.0, 1200.0
        data.Act_A, data.Act_B, data.Act_C = a, b, c
        data.data_collection = data_collection
        if with_force:
            # 按真值模型合成该姿态下的传感器读数（raw = 物理量 / 0.001）
            d_vec = gravity_direction_sensor(
                [a, b, c], self._runner.sensor_to_tcp_rotation, "ZYX"
            )
            g_force = matrix_vector_multiply(S_TRUE, d_vec)
            force_n = [bias + g for bias, g in zip(BIAS_F, g_force)]
            torque_nm = [bias + t for bias, t in zip(BIAS_T, cross(COM, g_force))]
            raw = [int(round(v * 1000.0)) for v in force_n + torque_nm]
            (data.Fx_raw, data.Fy_raw, data.Fz_raw,
             data.Mx_raw, data.My_raw, data.Mz_raw) = raw
        return data


def make_server(tmpdir: str, force_requested: bool):
    """临时文件路径的标定配置 + RSIServer；返回 (server, config, events)。"""
    config = load_config("ft_calibration_config.json")
    config.files.calibration_path = os.path.join(tmpdir, "ft_calibration.json")
    config.files.sample_path = os.path.join(tmpdir, "ft_calibration_samples.json")
    config.mode = "calibrated_runtime"
    server = RSIServer(config=RSIConfig())
    server.calibration_config = config
    server.calibration_runner = CalibrationRunner(config)
    events: list[tuple[str, str]] = []
    server.on_event = lambda level, msg: events.append((level, msg))
    server.force_requested = force_requested
    server.force_mode = force_requested
    return server, config, events


def run_poses(server: RSIServer, factory: FrameFactory):
    """模拟 FT_Calibration.src：每个分散姿态 data_collection=TRUE 采 0.6s，段间 FALSE。"""
    min_samples = server.calibration_config.static_detection.min_samples
    poses = [
        (a, b, c)
        for a in (-60.0, 60.0)
        for b in (-60.0, 60.0)
        for c in (-60.0, 0.0, 60.0, 120.0)
    ]
    assert len(poses) >= min_samples
    # 先补一帧 FALSE，把进入标定时那一帧的残段冲掉（段太短会被丢弃）
    server.process_frame(factory.make(data_collection=False))
    for a, b, c in poses[:min_samples]:
        for _ in range(6):  # 6 * 100ms = 0.6s >= min_dwell_seconds
            server.process_frame(factory.make(a, b, c, data_collection=True))
        server.process_frame(factory.make(a, b, c, data_collection=False))


def test_auto_calibrate_restores_force():
    """--force 无标定启动（力控待激活）→ data_collection=TRUE → 自动标定 → 力控恢复。"""
    tmpdir = tempfile.mkdtemp(prefix="rsi_auto_cal_")
    server, config, events = make_server(tmpdir, force_requested=True)
    runner = server.calibration_runner

    # 模拟 --force 无标定启动：挂起待激活、无力控器、无标定结果
    server.force_suspended = True
    assert server.force_controller is None
    assert runner.calibration_result is None

    # 预置旧样本，验证进入标定时被清空（重新开始一轮）
    runner.samples.append(CalibrationSample())
    runner.last_sample_angles = [1.0, 2.0, 3.0]

    factory = FrameFactory(runner)
    server.process_frame(factory.make(data_collection=True, with_force=False))

    assert server.auto_calibrating is True
    assert config.mode == "calibration_collect"
    assert runner.samples == []
    assert runner.last_sample_angles is None
    assert server.force_suspended is True  # 无力控器，保持挂起待激活
    assert server.config.ov_pro == 100.0
    assert any("自动进入标定模式" in m for _, m in events)
    assert not any("力控已挂起" in m for _, m in events)
    print("场景1 边沿检测：自动进入标定、旧样本已清空，OK")

    run_poses(server, factory)

    assert config.mode == "calibrated_runtime"
    assert runner.calibration_result is not None
    assert abs(runner.calibration_result.mass_kg - 3.0) < 0.5
    assert runner.calibration_result.residual_force_rms_n < 0.5
    assert os.path.exists(config.files.calibration_path)
    assert os.path.exists(config.files.sample_path)
    assert server.auto_calibrating is False
    assert server.force_suspended is False
    assert server.force_mode is True
    assert server.force_controller is not None
    assert any("力控已自动恢复" in m for _, m in events)
    print("场景1 求解完成：标定文件已写、力控自动恢复（新建控制器），OK")


def test_auto_calibrate_without_force():
    """非 --force 启动：自动标定正常完成，但不建力控器。"""
    tmpdir = tempfile.mkdtemp(prefix="rsi_auto_cal_nf_")
    server, config, events = make_server(tmpdir, force_requested=False)
    factory = FrameFactory(server.calibration_runner)

    server.process_frame(factory.make(data_collection=True, with_force=False))
    assert server.auto_calibrating is True
    assert config.mode == "calibration_collect"

    run_poses(server, factory)

    assert config.mode == "calibrated_runtime"
    assert server.calibration_runner.calibration_result is not None
    assert server.auto_calibrating is False
    assert server.force_controller is None
    assert server.force_suspended is False
    assert not any("力控已自动恢复" in m for _, m in events)
    print("场景2 非 --force 启动：标定完成但不建力控器，OK")


def test_force_suspended_on_calibration_entry():
    """力控运行中收到 data_collection=TRUE：挂起力控、复位控制器、RKorr=0、OV_PRO=100%。"""
    tmpdir = tempfile.mkdtemp(prefix="rsi_auto_cal_sus_")
    server, config, events = make_server(tmpdir, force_requested=True)
    controller = ForceController(
        config.force_control, rsi_rotation_order=config.rsi_rotation_order
    )
    server.force_controller = controller

    # 弄脏控制器/回包状态，验证挂起时被复位
    controller.integral_n_s = 5.0
    controller.corr_cumulative_mm = [1.0, 2.0, 3.0]
    controller.tripped = True
    controller.ov_pro_pct = 37.0
    server.config.ov_pro = 37.0
    server.config.rkorr["RKorr.X"] = 0.05

    factory = FrameFactory(server.calibration_runner)
    server.process_frame(factory.make(data_collection=True, with_force=False))

    assert server.auto_calibrating is True
    assert config.mode == "calibration_collect"
    assert server.force_suspended is True
    assert controller.integral_n_s == 0.0
    assert controller.corr_cumulative_mm == [0.0, 0.0, 0.0]
    assert controller.tripped is False
    assert controller.ov_pro_pct == config.force_control.default_ov_pro
    assert server.config.ov_pro == 100.0
    assert all(value == 0.0 for value in server.config.rkorr.values())
    assert any(level == "warning" and "力控已挂起" in m for level, m in events)
    print("场景3 力控运行中进入标定：控制器复位、RKorr=0、OV_PRO=100%，OK")

    # 标定完成后恢复力控应复用同一控制器实例（已 reset）
    run_poses(server, factory)
    assert server.force_controller is controller
    assert server.force_suspended is False
    assert server.force_controller.integral_n_s == 0.0
    assert any("力控已自动恢复" in m for _, m in events)
    print("场景3 标定完成：力控恢复并复用原控制器，OK")


test_auto_calibrate_restores_force()
test_auto_calibrate_without_force()
test_force_suspended_on_calibration_entry()
print("\n自动重标定全部自检通过")
