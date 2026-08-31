"""恒力打磨控制器：力误差 -> 单轴位置修正（导纳式位置外环）。

每个 RSI 周期（4ms）调用一次 update()：
- 输入：补偿后的 TCP 系接触力、当前 TCP 姿态、目标力
- 输出：基坐标系 XYZ 位置修正（mm），写入 RKorr

符号约定（默认配置）：
- 压紧轴 = 工具系 X；沿工具 +X 压向工件时，工件对传感器的反力读数为 -X
- press_sign = -1 表示"接触力读数符号为负"，press_force = press_sign * f_axis 恒为正值
- 若现场压紧方向相反（读数为 +X），把 config 里 press_sign 改为 1 即可
"""
from __future__ import annotations

from collections import deque
from typing import Optional

from calibration_math import euler_to_matrix, matrix_vector_multiply
from calibration_models import ForceControlConfig


AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


class ForceController:
    def __init__(self, config: ForceControlConfig, rsi_rotation_order: str = "ZYX"):
        self.config = config
        self.rsi_rotation_order = rsi_rotation_order
        self.axis_index = AXIS_INDEX[config.axis.upper()]
        self.integral_n_s = 0.0
        self.corr_cumulative_mm = [0.0, 0.0, 0.0]  # 基坐标系累积修正，用于限幅
        self.force_window: deque[float] = deque(maxlen=max(1, config.filter_window))
        self.tripped = False       # 超力保护锁存
        self.in_contact = False
        self._last_print_state: Optional[bool] = None

    def reset(self):
        self.integral_n_s = 0.0
        self.corr_cumulative_mm = [0.0, 0.0, 0.0]
        self.force_window.clear()
        self.tripped = False
        self.in_contact = False

    def update(self, force_tcp: list[float], tcp_angles_deg: list[float], target_force_n: float) -> dict[str, float]:
        cfg = self.config
        zero = {f"RKorr.{axis}": 0.0 for axis in "XYZABC"}

        self.force_window.append(force_tcp[self.axis_index])
        f_axis_raw = force_tcp[self.axis_index]
        f_axis = sum(self.force_window) / len(self.force_window)
        press_force = cfg.press_sign * f_axis  # 正值 = 压紧力大小

        # ---- 超力保护：用原始值（不过滤）立即锁存；钻孔时路径仍在进给，
        #      清零等于放任撞刀，因此保护动作 = 全速退刀，直到力回落到一半阈值以下 ----
        motion_sign = -cfg.press_sign  # 增大压紧力的运动方向（工具系）
        if abs(f_axis_raw) > cfg.max_force_n:
            if not self.tripped:
                print(f"[力控] 超力保护！{cfg.axis} 轴力 {f_axis_raw:.1f} N 超过 {cfg.max_force_n:.1f} N，全速退刀")
            self.tripped = True
        elif self.tripped and abs(f_axis_raw) < cfg.max_force_n * 0.5:
            self.tripped = False
            print("[力控] 力已回落，解除保护")
        if self.tripped:
            self.integral_n_s = 0.0
            corr_retreat = [0.0, 0.0, 0.0]
            corr_retreat[self.axis_index] = -motion_sign * cfg.per_cycle_max_mm
            return self._apply_correction(corr_retreat, tcp_angles_deg)

        # ---- 目标力无效：不动作 ----
        if target_force_n <= 0.0:
            self.integral_n_s = 0.0
            return zero

        was_in_contact = self.in_contact
        self.in_contact = press_force >= cfg.contact_threshold_n
        if self.in_contact != was_in_contact:
            print(f"[力控] 接触状态 -> {'已接触' if self.in_contact else '未接触'}"
                  f"（{cfg.axis} 轴压紧力 {press_force:.1f} N）")

        corr_tool = [0.0, 0.0, 0.0]
        if not self.in_contact:
            self.integral_n_s = 0.0
            if cfg.search_before_contact:
                # 打磨：未接触时慢速搜索表面，防止空走后猛撞
                corr_tool[self.axis_index] = motion_sign * cfg.search_speed_mm_s * cfg.cycle_s
            # 钻孔：零修正，编程路径自行进给；钻穿后力消失也不会前冲
        else:
            error = target_force_n - press_force
            if abs(error) < cfg.deadband_n:
                error = 0.0
            self.integral_n_s = _clamp(
                self.integral_n_s + error * cfg.cycle_s,
                -cfg.integral_limit_n_s,
                cfg.integral_limit_n_s,
            )
            velocity_mm_s = cfg.kp_mm_per_s_per_n * error + cfg.ki_mm_per_s2_per_n * self.integral_n_s
            # 非对称限速：加压方向限 advance_speed（防接触瞬态前冲）；
            # 退刀方向允许到每周期限幅对应的全速，保证能"拉住"编程进给
            if velocity_mm_s >= 0.0:
                velocity_mm_s = min(velocity_mm_s, cfg.advance_speed_mm_s)
            else:
                v_retreat_max = cfg.per_cycle_max_mm / cfg.cycle_s
                velocity_mm_s = max(velocity_mm_s, -v_retreat_max)
            corr_tool[self.axis_index] = motion_sign * velocity_mm_s * cfg.cycle_s

        # ---- 每周期限幅（单轴，直接钳） ----
        corr_tool[self.axis_index] = _clamp(
            corr_tool[self.axis_index], -cfg.per_cycle_max_mm, cfg.per_cycle_max_mm
        )

        return self._apply_correction(corr_tool, tcp_angles_deg)

    def _apply_correction(self, corr_tool: list[float], tcp_angles_deg: list[float]) -> dict[str, float]:
        """工具系修正 -> 基坐标系，统一做每周期限幅和累积限幅（退刀路径也走这里）。"""
        cfg = self.config
        result = {f"RKorr.{axis}": 0.0 for axis in "XYZABC"}

        # 工具系 -> 基坐标系（RKorr 在基坐标系生效）
        r_base_tcp = euler_to_matrix(tcp_angles_deg, self.rsi_rotation_order)
        corr_base = matrix_vector_multiply(r_base_tcp, corr_tool)

        # 累积限幅：先钳累积量本身，修正量 = 钳后目标 - 当前累积，
        # 保证累积越界时不会产生反向大跳变
        for axis in range(3):
            total = _clamp(
                self.corr_cumulative_mm[axis] + corr_base[axis],
                -cfg.cumulative_max_mm,
                cfg.cumulative_max_mm,
            )
            corr_base[axis] = total - self.corr_cumulative_mm[axis]
            self.corr_cumulative_mm[axis] = total

        # 最终每周期限幅（兜底）
        for axis in range(3):
            corr_base[axis] = _clamp(corr_base[axis], -cfg.per_cycle_max_mm, cfg.per_cycle_max_mm)

        result["RKorr.X"] = corr_base[0]
        result["RKorr.Y"] = corr_base[1]
        result["RKorr.Z"] = corr_base[2]
        return result

    @property
    def status_line(self) -> str:
        return (
            f"接触={'是' if self.in_contact else '否'} "
            f"累积修正=({self.corr_cumulative_mm[0]:.2f},"
            f"{self.corr_cumulative_mm[1]:.2f},"
            f"{self.corr_cumulative_mm[2]:.2f}) mm"
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
