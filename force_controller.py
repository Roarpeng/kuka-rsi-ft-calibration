"""恒力钻孔控制器：力误差 -> 单轴位置修正（导纳式位置外环）。

每个 RSI 周期（4ms）调用一次 update()：
- 输入：补偿后的 TCP 系接触力、当前 TCP 姿态、目标力
- 输出：与 RSIVisual PosCorr.RefCorrSys 一致的**本拍增量**（默认 **工具系** XYZ）

RKorr 语义（KST RSI 4.0 + 现场 RSIVisual）：
- KRL 必须 RSI_ON(#RELATIVE)。POSCORR 把每包 RKorr 当作**本拍叠加增量**；
  发 0 表示保持当前叠加；单拍幅值由 per_cycle_max_mm 钳位（默认 0.08 mm ≈ 20 mm/s）。
- PosCorr Lower/UpperLim ±0.2 限制的就是这一拍；PosCorrMon.MaxTrans=80 限制
  的是叠加后的总偏移。内部 corr_cumulative_mm 跟踪总偏移，不下发。
- **RefCorrSys=Tool**：增量就是工具系毫米，禁止再旋到基座。
  若按基座系发出，A=-170° 时退刀会变成加压，空气中正反馈把运动残差打成 ±200N。
- RSI 重启后机器人叠加归零，上位机靠收包中断检测复位累积量（见 udp_server.py）。

符号约定（按现场实测，见 rsi_data_20260831_131835.csv 复盘）：
- 压紧轴 = 工具系 X；进给 = 工具 +X（正增量），退刀 = 工具 −X（负增量）
- press_sign = -1：press_force = press_sign * f_axis 恒为正值（压紧时该轴读数为负）
- press_motion_sign = +1：增大压紧力的运动方向为工具 +X
  该符号与传感器读数符号无必然关系（中间隔着传感器安装方向），
  所以独立成参数；现场若改装夹方向，只改 config 即可

空载超限（rsi_data_20260831_152955.csv）：前方空气、RKorr 仍为 0 时
静止残差约 3N；LIN 启动后工具悬臂振荡把压紧力顶到 12N，被当成接触。
随后 PC 按基座系发 RKorr，而 PosCorr.RefCorrSys=Tool → 退刀变成加压（正反馈），
空气中把惯性力打到 ±200N。修正：RKorr 按工具系输出；接触阈高于运动振荡。
"""
from __future__ import annotations

import math
from collections import deque
from typing import Callable, Optional

from calibration_math import euler_to_matrix, matrix_vector_multiply
from calibration_models import ForceControlConfig


AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


def _median(values: deque[float] | list[float]) -> float:
    data = sorted(values)
    n = len(data)
    mid = n // 2
    if n % 2:
        return data[mid]
    return 0.5 * (data[mid - 1] + data[mid])


class _ForceAxisFilter:
    """中值去脉冲 + 一阶低通。保护用短中值，控制用长中值+低通。"""

    def __init__(self, median_n: int, protect_n: int, lpf_hz: float, cycle_s: float):
        self.median_win: deque[float] = deque(maxlen=max(1, median_n))
        self.protect_win: deque[float] = deque(maxlen=max(1, protect_n))
        if lpf_hz <= 0.0:
            self.alpha = 1.0
        else:
            tau = 1.0 / (2.0 * math.pi * lpf_hz)
            self.alpha = cycle_s / (tau + cycle_s)
        self.lpf: Optional[float] = None

    def reset(self):
        self.median_win.clear()
        self.protect_win.clear()
        self.lpf = None

    def push(self, value: float) -> tuple[float, float, float]:
        self.median_win.append(value)
        self.protect_win.append(value)
        protect = _median(self.protect_win)
        med = _median(self.median_win)
        if self.lpf is None:
            self.lpf = med
        else:
            self.lpf += self.alpha * (med - self.lpf)
        return value, protect, self.lpf


class ForceController:
    def __init__(self, config: ForceControlConfig, rsi_rotation_order: str = "ZYX"):
        self.config = config
        self.rsi_rotation_order = rsi_rotation_order
        self.axis_index = AXIS_INDEX[config.axis.upper()]
        # 垂直于钻轴的两轴：横向让位（平移）与对中（力矩->同轴旋转）都跟随钻轴推导。
        # 例：axis=X -> Y/Z + My/Mz->B/C；axis=Z -> X/Y + Mx/My->A/B。
        self.perp = [i for i in range(3) if i != self.axis_index]
        self.integral_n_s = 0.0
        self.corr_cumulative_mm = [0.0, 0.0, 0.0]  # 输出坐标系总偏移（不下发，限幅/路径估计用）
        self.force_filter = _ForceAxisFilter(
            median_n=max(1, config.filter_median_window),
            protect_n=max(1, config.filter_protect_window),
            lpf_hz=config.filter_lpf_hz,
            cycle_s=config.cycle_s,
        )
        # 凿击横向让位：Y/Z 各一个慢通道滤波（长中值 + 深低通，不追凿击冲击尖峰）
        self.lateral_filters = [
            _ForceAxisFilter(
                median_n=max(1, config.chisel_lateral_median_window),
                protect_n=max(1, config.filter_protect_window),
                lpf_hz=config.chisel_lateral_lpf_hz,
                cycle_s=config.cycle_s,
            )
            for _ in range(2)
        ]
        self.tripped = False       # 超力保护锁存（X 超限、横向卡滞或力矩卡滞）
        # 轴线零力矩对中（B←My、C←Mz）：力矩慢通道滤波；绕 TCP 旋转与平移让位正交
        self.align_filters = [
            _ForceAxisFilter(
                median_n=max(1, config.align_median_window),
                protect_n=max(1, config.filter_protect_window),
                lpf_hz=config.align_lpf_hz,
                cycle_s=config.cycle_s,
            )
            for _ in range(2)
        ]
        self.corr_cumulative_deg = [0.0, 0.0, 0.0]  # A/B/C 旋转叠加累计（°，不下发，限幅用）
        self.align_active = False    # 对中调节中（旋转与 X 倍率正交，不冻结 OV_PRO）
        self.in_contact = False
        self._last_print_state: Optional[bool] = None
        self._last_path_pos_mm: Optional[list[float]] = None
        self._held_path_feed_mm_s = 0.0
        self._zero_feed_count = 0
        self._lost_count = 0
        self._trip_ok_count = 0
        self._settle_count = 0
        self._last_press_slow: Optional[float] = None
        self._last_press_raw: Optional[float] = None
        self._bounce_hold = 0
        self.ov_pro_pct = config.default_ov_pro
        self.chisel_lateral_active = False  # 凿击横向让位进行中（此期间 X 恒力冻结）
        self._last_mode: Optional[str] = None
        self._lateral_limit_warned = False   # 横向让位到行程上限只提示一次
        self._align_limit_warned = False     # 对中到角度上限只提示一次
        # Web 监控层可选事件回调：best-effort，异常不拖垮控制环
        self.on_event: Optional[Callable[[str, str], None]] = None

    def _emit_event(self, level: str, message: str):
        """事件回调（level: info/warning/error）；回调异常一律吞掉。"""
        if self.on_event is not None:
            try:
                self.on_event(level, message)
            except Exception:
                pass

    def reset(self):
        self.integral_n_s = 0.0
        self.corr_cumulative_mm = [0.0, 0.0, 0.0]
        self.corr_cumulative_deg = [0.0, 0.0, 0.0]
        self.force_filter.reset()
        for filt in self.lateral_filters:
            filt.reset()
        for filt in self.align_filters:
            filt.reset()
        self.tripped = False
        self.in_contact = False
        self._last_path_pos_mm = None
        self._held_path_feed_mm_s = 0.0
        self._zero_feed_count = 0
        self._lost_count = 0
        self._trip_ok_count = 0
        self._settle_count = 0
        self._last_press_slow = None
        self._last_press_raw = None
        self._bounce_hold = 0
        self.ov_pro_pct = self.config.default_ov_pro
        self.chisel_lateral_active = False
        self.align_active = False
        self._last_mode = None
        self._lateral_limit_warned = False
        self._align_limit_warned = False

    def _desired_ov_pro(self, press_fast: float, press_slow: float, target_force_n: float) -> float:
        """力 → $OV_PRO（0–100%）。空载满倍率接近；到目标/超力停 LIN；不能倒车。

        调速与停止判断用滤波通道 press_slow（中值+低通）：实测静止钻削存在
        1~3 帧力尖峰（噪声/粘滑振铃），用原始值判断会把倍率反复打到 0
        （实测滤波均值 30N 时 OV 有 98% 时间为 0）。超力瞬时支路与 trip 段
        一致用短中值 press_fast（杀单帧噪声，12ms 响应）。
        """
        cfg = self.config
        default = cfg.default_ov_pro
        if self.tripped or abs(press_fast) > cfg.max_force_n:
            return 0.0
        press = press_slow
        if target_force_n <= 0.0:
            return 0.0
        if press < cfg.contact_threshold_n:
            return default
        if press >= target_force_n:
            return 0.0
        span = target_force_n - cfg.contact_threshold_n
        if span <= 1e-9:
            return 0.0
        return default * (1.0 - (press - cfg.contact_threshold_n) / span)

    def _slew_ov(self, command: float) -> float:
        cfg = self.config
        command = _clamp(command, 0.0, max(cfg.default_ov_pro, 0.0))
        slew = max(0.0, cfg.ov_pro_slew_pct)
        cur = self.ov_pro_pct
        if abs(command - cur) <= slew:
            self.ov_pro_pct = command
        elif command > cur:
            self.ov_pro_pct = min(command, cur + slew)
        else:
            self.ov_pro_pct = max(command, cur - slew)
        return self.ov_pro_pct

    def update(
        self,
        force_tcp: list[float],
        tcp_angles_deg: list[float],
        target_force_n: float,
        tcp_position_mm: Optional[list[float]] = None,
        mode: str = "overlay",
        torque_tcp: Optional[list[float]] = None,
    ) -> dict[str, float]:
        cfg = self.config
        zero = {f"RKorr.{axis}": 0.0 for axis in "XYZABC"}

        if mode != self._last_mode:
            # 模式切换（钻孔<->凿击等）清横向/对中滤波历史，避免旧模式残值串扰
            for filt in self.lateral_filters:
                filt.reset()
            for filt in self.align_filters:
                filt.reset()
            self._last_mode = mode
            self._lateral_limit_warned = False
            self._align_limit_warned = False

        f_axis_raw, f_axis_fast, f_axis_slow = self.force_filter.push(force_tcp[self.axis_index])
        press_raw = cfg.press_sign * f_axis_raw
        press_fast = cfg.press_sign * f_axis_fast
        press_slow = cfg.press_sign * f_axis_slow
        # 加压用慢通道（中值+低通，不追 30Hz 弹跳）；超目标用原始力立刻退
        press_force = press_raw if press_raw > target_force_n else press_slow

        motion_sign = cfg.press_motion_sign  # +1：进给工具 +X；退刀工具 -X
        v_retreat_max = cfg.per_cycle_max_mm / cfg.cycle_s
        lost_n = max(1, int(round(cfg.contact_lost_s / cfg.cycle_s)))
        jerk = abs(press_raw - (self._last_press_raw if self._last_press_raw is not None else press_raw))
        if jerk > 25.0:
            self._bounce_hold = max(self._bounce_hold, lost_n + 2)
        elif self._bounce_hold > 0:
            self._bounce_hold -= 1
        path_feed = self._path_feed_mm_s(tcp_position_mm, tcp_angles_deg, motion_sign)

        # 凿击：推横向慢通道（Y/Z）；中值窗未填满前不让位（防冷启动单帧尖峰）
        lat_raw = [0.0, 0.0]
        lat_slow = [0.0, 0.0]
        lat_ready = [False, False]
        if mode == "chisel":
            for i in range(2):
                lat_raw[i], _, lat_slow[i] = self.lateral_filters[i].push(force_tcp[self.perp[i]])
                lat_ready[i] = len(self.lateral_filters[i].median_win) >= max(
                    1, cfg.chisel_lateral_median_window
                )

        # 轴线对中（凿击/钻孔分别可开关）：推 My/Mz 力矩慢通道
        align_on = (mode == "chisel" and cfg.align_chisel_enable) or (
            mode == "drill" and cfg.align_drill_enable
        )
        torque = torque_tcp if torque_tcp is not None else [0.0, 0.0, 0.0]
        align_raw = [0.0, 0.0]
        align_slow = [0.0, 0.0]
        if align_on:
            for i in range(2):
                _, _, align_slow[i] = self.align_filters[i].push(torque[self.perp[i]])

        # ---- 超力保护：用原始值立即锁存。弹跳过零不得解锁，须连续卸荷 trip_clear_s ----
        trip_clear_n = max(1, int(round(cfg.trip_clear_s / cfg.cycle_s)))
        # X 轴超力保护锁存用滤波通道 press_slow：实测 EMI 尖峰可达 190N（1~3 帧），
        # 原始值/中值3 都会被误锁存（OV 被反复打到 0）。真实持续超力 ~60ms 内
        # 触发；瞬时支路（_desired_ov_pro）仍用中值3 快速降倍率。
        over_force = abs(press_slow) > cfg.max_force_n
        over_lateral = mode == "chisel" and (
            abs(lat_raw[0]) > cfg.chisel_lateral_trip_n
            or abs(lat_raw[1]) > cfg.chisel_lateral_trip_n
        )
        # 力矩卡滞用滤波通道判断：实测静止钻削的原始力矩存在 1~3 帧 ±50 N·m 级尖峰
        # （噪声/粘滑振铃），用原始值会连续误触发保护把倍率打到 0（实测一轮 62 次）。
        over_align = align_on and (
            abs(align_slow[0]) > cfg.align_trip_nm or abs(align_slow[1]) > cfg.align_trip_nm
        )
        if over_force or over_lateral or over_align:
            if not self.tripped:
                if over_align and not over_force and not over_lateral:
                    message = (
                        f"[力控] 力矩卡滞保护！My/Mz 滤波力矩 {align_slow[0]:.2f}/{align_slow[1]:.2f} N·m"
                        f" 超过 {cfg.align_trip_nm:.1f} N·m，沿 -X 全速退刀"
                    )
                elif over_lateral and not over_force:
                    message = (
                        f"[力控] 横向卡滞保护！FY/FZ 原始力 {lat_raw[0]:.1f}/{lat_raw[1]:.1f} N"
                        f" 超过 {cfg.chisel_lateral_trip_n:.1f} N，沿 -X 全速退刀"
                    )
                else:
                    message = f"[力控] 超力保护！{cfg.axis} 轴滤波力 {press_slow:.1f} N 超过 {cfg.max_force_n:.1f} N，全速退刀"
                print(message)
                self._emit_event("error", message)
            self.tripped = True
            self._trip_ok_count = 0
        elif self.tripped:
            lateral_clear = mode != "chisel" or (
                abs(lat_raw[0]) < cfg.chisel_lateral_trip_n * 0.5
                and abs(lat_raw[1]) < cfg.chisel_lateral_trip_n * 0.5
            )
            align_clear = not align_on or (
                abs(align_slow[0]) < cfg.align_trip_nm * 0.5
                and abs(align_slow[1]) < cfg.align_trip_nm * 0.5
            )
            if (
                lateral_clear
                and align_clear
                and press_raw < cfg.contact_threshold_n
                and abs(press_slow) < cfg.max_force_n * 0.5
            ):
                self._trip_ok_count += 1
                if self._trip_ok_count >= trip_clear_n:
                    self.tripped = False
                    self._trip_ok_count = 0
                    print("[力控] 力已回落，解除保护")
                    self._emit_event("info", "[力控] 力已回落，解除保护")
            else:
                self._trip_ok_count = 0

        # ---- 钻孔：用 $OV_PRO 控编程进给，RKorr 平移保持 0（凿击残留横向叠加缓撤到 0）；
        # 轴线对中开启时输出 B/C 旋转（治 X 轴与作业面不垂直），与 OV_PRO 速度环正交 ----
        if mode == "drill":
            self._slew_ov(self._desired_ov_pro(press_fast, press_slow, target_force_n))
            self._last_press_slow = press_slow
            self._last_press_raw = press_raw
            self.integral_n_s = 0.0
            self.chisel_lateral_active = False
            corr = [0.0, 0.0, 0.0]
            for i in range(2):
                axis = self.perp[i]
                if abs(self.corr_cumulative_mm[axis]) > 1e-9:
                    corr[axis] = _clamp(
                        -self.corr_cumulative_mm[axis],
                        -cfg.per_cycle_max_mm,
                        cfg.per_cycle_max_mm,
                    )
            if cfg.align_drill_enable and not self.tripped:
                rot, self.align_active = self._align_correction(align_slow)
            else:
                self.align_active = False
                rot = self._rotation_unwind()
            if any(abs(v) > 1e-12 for v in corr) or any(abs(v) > 1e-12 for v in rot):
                return self._apply_correction(corr, tcp_angles_deg, rot)
            return dict(zero)
        if mode == "chisel":
            # 凿击：X 仍按 OV_PRO 恒力压紧；Y/Z 横向零力让位（滑坑/卡滞时顺势卸载，
            # 方向 = chisel_lateral_sign * 读数方向；读数=工件对工具作用力，同号让位即背离障碍物）
            self._last_press_slow = press_slow
            self._last_press_raw = press_raw
            self.integral_n_s = 0.0
            if self.tripped:
                self.chisel_lateral_active = False
                corr_retreat = [0.0, 0.0, 0.0]
                corr_retreat[self.axis_index] = -motion_sign * cfg.per_cycle_max_mm
                return self._apply_correction(corr_retreat, tcp_angles_deg)
            corr_tool = [0.0, 0.0, 0.0]
            lateral_active = False
            for i in range(2):
                axis = self.perp[i]
                f_slow = lat_slow[i]
                if not lat_ready[i] or abs(f_slow) <= cfg.chisel_lateral_deadband_n:
                    continue
                lateral_active = True
                excess = abs(f_slow) - cfg.chisel_lateral_deadband_n
                speed = min(cfg.chisel_lateral_gain_mm_per_s_per_n * excess, v_retreat_max)
                step = (
                    cfg.chisel_lateral_sign
                    * (1.0 if f_slow > 0.0 else -1.0)
                    * speed
                    * cfg.cycle_s
                )
                # 单轴行程上限：本拍后不得越过 ±chisel_lateral_max_mm（内部另受 80mm 总限）
                cur = self.corr_cumulative_mm[axis]
                step = _clamp(
                    step,
                    -cfg.chisel_lateral_max_mm - cur,
                    cfg.chisel_lateral_max_mm - cur,
                )
                if step == 0.0:
                    if not self._lateral_limit_warned:
                        message = (
                            f"[力控] 横向让位到行程上限 ±{cfg.chisel_lateral_max_mm:.1f} mm，"
                            "该轴暂停让位（横向力仍在，请检查是否卡死）"
                        )
                        print(message)
                        self._emit_event("warning", message)
                        self._lateral_limit_warned = True
                    continue
                corr_tool[axis] = step
            self.chisel_lateral_active = lateral_active
            if lateral_active:
                pass  # X 恒力冻结：保持当前 OV_PRO 不动，避免与横向调节互相拉扯
            else:
                self._slew_ov(self._desired_ov_pro(press_fast, press_slow, target_force_n))
            # 轴线对中叠加（旋转与平移让位正交，不冻结 X 倍率）
            if cfg.align_chisel_enable:
                rot, self.align_active = self._align_correction(align_slow)
            else:
                self.align_active = False
                rot = self._rotation_unwind()
            return self._apply_correction(corr_tool, tcp_angles_deg, rot)
        self.ov_pro_pct = cfg.default_ov_pro
        self.chisel_lateral_active = False

        if self.tripped:
            self.integral_n_s = 0.0
            self._last_press_slow = press_slow
            self._last_press_raw = press_raw
            corr_retreat = [0.0, 0.0, 0.0]
            corr_retreat[self.axis_index] = -motion_sign * cfg.per_cycle_max_mm
            return self._apply_correction(corr_retreat, tcp_angles_deg)

        # ---- 目标力无效：发反向增量把叠加缓撤到 0；空载仍接触则按限幅退刀 ----
        # #RELATIVE 下发 0 只是保持叠加。直接 RSI_OFF 仍会丢掉叠加、跳回编程路径。
        if target_force_n <= 0.0:
            self.integral_n_s = 0.0
            if press_raw >= cfg.contact_threshold_n:
                corr_retreat = [0.0, 0.0, 0.0]
                corr_retreat[self.axis_index] = -motion_sign * cfg.per_cycle_max_mm
                return self._apply_correction(corr_retreat, tcp_angles_deg)
            result = dict(zero)
            for axis in range(3):
                step = _clamp(
                    -self.corr_cumulative_mm[axis],
                    -cfg.per_cycle_max_mm,
                    cfg.per_cycle_max_mm,
                )
                self.corr_cumulative_mm[axis] += step
                result[f"RKorr.{'XYZ'[axis]}"] = step
            return result

        was_in_contact = self.in_contact
        if not self.in_contact:
            self.in_contact = press_fast >= cfg.contact_threshold_n
            self._lost_count = 0
            if self.in_contact:
                self._settle_count = max(1, int(round(0.02 / cfg.cycle_s)))
        else:
            lost_candidate = (
                press_fast < cfg.contact_threshold_n * 0.5
                and abs(press_raw) < cfg.contact_threshold_n
            )
            if lost_candidate and self._bounce_hold > 0:
                self._lost_count += 1
                if self._lost_count >= lost_n:
                    self.in_contact = False
                    self.integral_n_s = 0.0
            elif lost_candidate:
                self.in_contact = False
                self.integral_n_s = 0.0
                self._lost_count = 0
            else:
                self._lost_count = 0
        if self.in_contact != was_in_contact:
            message = (f"[力控] 接触状态 -> {'已接触' if self.in_contact else '未接触'}"
                       f"（{cfg.axis} 轴压紧力 fast={press_fast:.1f} slow={press_slow:.1f} raw={press_raw:.1f} N）")
            print(message)
            self._emit_event("info", message)

        corr_tool = [0.0, 0.0, 0.0]
        holding_lost = self.in_contact and self._lost_count > 0
        if not self.in_contact:
            self.integral_n_s = 0.0
            self._settle_count = 0
            if cfg.search_before_contact:
                # 打磨：未接触时慢速搜索表面，防止空走后猛撞
                corr_tool[self.axis_index] = motion_sign * cfg.search_speed_mm_s * cfg.cycle_s
            # 钻孔：零修正，编程路径自行进给；钻穿后力消失也不会前冲
        elif holding_lost:
            # 疑似弹跳过零：发 0 增量保持叠加，避免 LIN 在未接触窗口里 10mm/s 往墙上顶
            pass
        else:
            error = target_force_n - press_force
            if abs(error) < cfg.deadband_n:
                error = 0.0
            self.integral_n_s = _clamp(
                self.integral_n_s + error * cfg.cycle_s,
                -cfg.integral_limit_n_s,
                cfg.integral_limit_n_s,
            )
            # pi = 沿压紧方向的**净 TCP 速度**；编程进给由 path_feed 在 RKorr 里抵消
            pi = cfg.kp_mm_per_s_per_n * error + cfg.ki_mm_per_s2_per_n * self.integral_n_s
            if self._settle_count > 0:
                # 刚接触：只抵消进给，净速度 ≤0，不往刚性表面再顶
                self._settle_count -= 1
                pi = min(pi, 0.0)
            # 力仍在快速爬升（刚性/撞击）：禁止继续加压，只抵消进给或退刀
            rising_n_per_cycle = 0.8  # 0.8N/4ms ≈ 200N/s，空载撞墙约 500N/s
            if (
                self._last_press_slow is not None
                and press_slow > self._last_press_slow + rising_n_per_cycle
                and press_raw >= cfg.contact_threshold_n
            ):
                pi = min(pi, 0.0)
            if pi >= 0.0:
                pi = min(pi, cfg.advance_speed_mm_s)
            else:
                pi = max(pi, -v_retreat_max)
            velocity_mm_s = pi - path_feed
            corr_tool[self.axis_index] = motion_sign * velocity_mm_s * cfg.cycle_s

        self._last_press_slow = press_slow
        self._last_press_raw = press_raw

        # ---- 每周期限幅（单轴，直接钳） ----
        corr_tool[self.axis_index] = _clamp(
            corr_tool[self.axis_index], -cfg.per_cycle_max_mm, cfg.per_cycle_max_mm
        )

        return self._apply_correction(corr_tool, tcp_angles_deg)

    def _path_feed_mm_s(
        self,
        tcp_position_mm: Optional[list[float]],
        tcp_angles_deg: list[float],
        motion_sign: int,
    ) -> float:
        """编程路径沿压紧方向的进给（mm/s）。有实测位姿时从 Act-RKorr 差分估计，
        停进给后自动变为 0；无位姿时回退到配置值（仿真）。"""
        cfg = self.config
        if tcp_position_mm is None:
            feed = max(0.0, cfg.path_feed_mm_s)
            if self.in_contact:
                self._held_path_feed_mm_s = feed
            return feed

        corr_base = self._cumulative_in_base(tcp_angles_deg)
        path_pos = [
            tcp_position_mm[i] - corr_base[i] for i in range(3)
        ]
        feed = 0.0
        if self._last_path_pos_mm is not None:
            vel_base = [
                (path_pos[i] - self._last_path_pos_mm[i]) / cfg.cycle_s for i in range(3)
            ]
            r_base_tcp = euler_to_matrix(tcp_angles_deg, self.rsi_rotation_order)
            vel_tool = [
                sum(r_base_tcp[j][i] * vel_base[j] for j in range(3)) for i in range(3)
            ]
            feed = max(0.0, motion_sign * vel_tool[self.axis_index])
            feed = min(feed, cfg.path_feed_mm_s)
        self._last_path_pos_mm = path_pos

        if feed >= 1.0:
            self._zero_feed_count = 0
            self._held_path_feed_mm_s = feed
            return feed
        self._zero_feed_count += 1
        # 仅弹跳反向时保持进给抵消；编程 LIN 正常结束必须立刻把 feed 降到 0
        if self.in_contact and self._bounce_hold > 0 and self._held_path_feed_mm_s > 0.0:
            return self._held_path_feed_mm_s
        self._held_path_feed_mm_s = feed
        return feed

    def _cumulative_in_base(self, tcp_angles_deg: list[float]) -> list[float]:
        """累积修正在基座系的表达。Act 是基座笛卡尔；RKorr 可能是工具系。"""
        if self.config.rkorr_frame != "tool":
            return list(self.corr_cumulative_mm)
        r_base_tcp = euler_to_matrix(tcp_angles_deg, self.rsi_rotation_order)
        return matrix_vector_multiply(r_base_tcp, self.corr_cumulative_mm)

    def _align_correction(self, align_slow: list[float]) -> tuple[list[float], bool]:
        """轴线零力矩对中：B←My、C←Mz（超出死区部分按增益旋转，方向 = align_sign × 力矩方向）。

        力矩慢通道只追持续弯矩（轴线被别），单次冲击被中值压掉；
        累计角度钳在 ±align_max_deg（机器人侧旋转限位很宽，必须上位机钳）。"""
        cfg = self.config
        rot = [0.0, 0.0, 0.0]
        active = False
        for i in range(2):  # 通道 = 垂直于钻轴的两轴（axis=X 时 My->B/Mz->C；axis=Z 时 Mx->A/My->B）
            f_slow = align_slow[i]
            ready = len(self.align_filters[i].median_win) >= max(1, cfg.align_median_window)
            if not ready or abs(f_slow) <= cfg.align_deadband_nm:
                continue
            active = True
            excess = abs(f_slow) - cfg.align_deadband_nm
            speed = min(
                cfg.align_gain_deg_per_s_per_nm * excess,
                cfg.align_per_cycle_max_deg / cfg.cycle_s,
            )
            step = (
                cfg.align_sign
                * (1.0 if f_slow > 0.0 else -1.0)
                * speed
                * cfg.cycle_s
            )
            cur = self.corr_cumulative_deg[self.perp[i]]
            step = _clamp(step, -cfg.align_max_deg - cur, cfg.align_max_deg - cur)
            if step == 0.0:
                if not self._align_limit_warned:
                    message = (
                        f"[力控] 轴线对中到角度上限 ±{cfg.align_max_deg:.1f}°，该轴暂停对中"
                        "（力矩仍在，请检查装夹/姿态）"
                    )
                    print(message)
                    self._emit_event("warning", message)
                    self._align_limit_warned = True
                continue
            rot[self.perp[i]] = step
        if not active:
            self._align_limit_warned = False
        return rot, active

    def _rotation_unwind(self) -> list[float]:
        """把残留的旋转叠加按每拍限幅缓撤到 0（对中关闭/模式退出时）。"""
        cfg = self.config
        rot = [0.0, 0.0, 0.0]
        for i in range(3):
            cur = self.corr_cumulative_deg[i]
            if abs(cur) > 1e-9:
                rot[i] = _clamp(-cur, -cfg.align_per_cycle_max_deg, cfg.align_per_cycle_max_deg)
        return rot

    def _apply_correction(
        self,
        corr_tool: list[float],
        tcp_angles_deg: list[float],
        corr_rot: Optional[list[float]] = None,
    ) -> dict[str, float]:
        """把本周期工具系平移/旋转变成 #RELATIVE 下发的 RKorr 增量。

        RefCorrSys=Tool：平移增量即工具系毫米；旋转增量（deg）同样按工具系
        （绕 TCP 旋转不平移 TCP，与平移让位正交；rkorr_frame=base 时不支持旋转）。
        顶到 cumulative_max 时实际增量会小于请求值（机器人侧叠加不再增加）。
        """
        cfg = self.config
        result = {f"RKorr.{axis}": 0.0 for axis in "XYZABC"}
        if cfg.rkorr_frame == "tool":
            delta = list(corr_tool)
        else:
            r_base_tcp = euler_to_matrix(tcp_angles_deg, self.rsi_rotation_order)
            delta = matrix_vector_multiply(r_base_tcp, corr_tool)

        for axis in range(3):
            step = _clamp(delta[axis], -cfg.per_cycle_max_mm, cfg.per_cycle_max_mm)
            old = self.corr_cumulative_mm[axis]
            total = _clamp(old + step, -cfg.cumulative_max_mm, cfg.cumulative_max_mm)
            self.corr_cumulative_mm[axis] = total
            delta[axis] = total - old

        result["RKorr.X"] = delta[0]
        result["RKorr.Y"] = delta[1]
        result["RKorr.Z"] = delta[2]

        if corr_rot is not None:
            for i in range(3):
                step = _clamp(corr_rot[i], -cfg.align_per_cycle_max_deg, cfg.align_per_cycle_max_deg)
                old = self.corr_cumulative_deg[i]
                total = _clamp(old + step, -cfg.align_max_deg, cfg.align_max_deg)
                self.corr_cumulative_deg[i] = total
                result[f"RKorr.{'ABC'[i]}"] = total - old
        return result

    @property
    def status_line(self) -> str:
        lateral = " 横向让位中" if self.chisel_lateral_active else ""
        align = " 轴线对中中" if self.align_active else ""
        return (
            f"接触={'是' if self.in_contact else '否'}{lateral}{align} "
            f"累积修正=({self.corr_cumulative_mm[0]:.2f},"
            f"{self.corr_cumulative_mm[1]:.2f},"
            f"{self.corr_cumulative_mm[2]:.2f}) mm "
            f"({self.corr_cumulative_deg[0]:+.2f},"
            f"{self.corr_cumulative_deg[1]:+.2f},"
            f"{self.corr_cumulative_deg[2]:+.2f})° "
            f"OV={self.ov_pro_pct:.1f}%"
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
