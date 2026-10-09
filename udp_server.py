from __future__ import annotations

import argparse
import collections
import csv
import math
import os
import random
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime
from typing import TYPE_CHECKING, Callable, Optional

from calibration_io import load_calibration_result, load_config
from calibration_models import CalibratedWrench
from calibration_runner import CalibrationRunner
from force_controller import ForceController

if TYPE_CHECKING:
    from calibration_models import CalibrationConfig


@dataclass
class RSIConfig:
    """与机器人 RSI Ethernet XML 的 CONFIG 段对齐。"""
    IP_NUMBER: str = "192.168.2.250"  # 本机 IP，机器人把 UDP 发到这里
    PORT: int = 59152
    SENTYPE: str = "ImFree"
    ONLYSEND: bool = False  # FALSE：双向闭环，必须回传 IPOC 和 RKorr
    ROBOT_IP: str = "192.168.2.10"
    BIND_IP: str = "0.0.0.0"
    # 采集/运行时默认回传全 0；仅链路测试可改为随机扰动
    reply_zeros: bool = True
    rkorr_min: float = -0.1
    rkorr_max: float = 0.1
    rkorr: dict[str, float] = field(
        default_factory=lambda: {
            "RKorr.X": 0.0,
            "RKorr.Y": 0.0,
            "RKorr.Z": 0.0,
            "RKorr.A": 0.0,
            "RKorr.B": 0.0,
            "RKorr.C": 0.0,
        }
    )
    ov_pro: float = 100.0  # 回发 $OV_PRO（0–100%），Map2OV_PRO 输入范围须一致


@dataclass
class RSIData:
    """RSI 数据结构"""
    Fx_raw: int = 0
    Fy_raw: int = 0
    Fz_raw: int = 0
    Mx_raw: int = 0
    My_raw: int = 0
    Mz_raw: int = 0
    Act_X: float = 0.0
    Act_Y: float = 0.0
    Act_Z: float = 0.0
    Act_A: float = 0.0
    Act_B: float = 0.0
    Act_C: float = 0.0
    timestamp: str = ""
    iPOC: int = 0
    ipoc_text: str = "0"
    data_collection: bool = False  # 标定采样触发：true 采集，false 不采集
    RobotStatus: int = 2           # INT：1=标定 2=钻孔（OV_PRO 力-速度）3=凿击（恒力+横向让位+对中）
    FORCEDATA: int = 0              # 机器人下发的恒力目标值 (N)（SEND IND 15；0=未下发用默认）
    target_force: float = 0.0      # 力控目标力（可选 XML；未带则用配置默认值）
    target_force_present: bool = False  # RSI XML 是否带了 <target_force>；未带则用配置默认值
    sensor_fx: float = 0.0
    sensor_fy: float = 0.0
    sensor_fz: float = 0.0
    sensor_mx: float = 0.0
    sensor_my: float = 0.0
    sensor_mz: float = 0.0
    tcp_fx: float = 0.0
    tcp_fy: float = 0.0
    tcp_fz: float = 0.0
    tcp_mx: float = 0.0
    tcp_my: float = 0.0
    tcp_mz: float = 0.0
    sample_status: str = "streaming"


def local_ip_exists(ip: str) -> bool:
    """本机是否拥有该 IPv4（能 bind 即存在）。"""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((ip, 0))
        return True
    except OSError:
        return False
    finally:
        probe.close()


def ping_host(ip: str, timeout_ms: int = 1000) -> bool:
    flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
    result = subprocess.run(
        ["ping", "-n" if sys.platform == "win32" else "-c", "1",
         "-w" if sys.platform == "win32" else "-W", str(timeout_ms if sys.platform == "win32" else max(1, timeout_ms // 1000)),
         ip],
        capture_output=True,
        creationflags=flags,
    )
    return result.returncode == 0


SAMPLE_ROB_XML = (
    "<Rob>"
    "<Fx_raw>100</Fx_raw><Fy_raw>200</Fy_raw><Fz_raw>300</Fz_raw>"
    "<Mx_raw>1</Mx_raw><My_raw>2</My_raw><Mz_raw>3</Mz_raw>"
    "<Act_X>903.0</Act_X><Act_Y>-80.5</Act_Y><Act_Z>1213.1</Act_Z>"
    "<Act_A>-83.7</Act_A><Act_B>0.8</Act_B><Act_C>179.8</Act_C>"
    "<data_collection>FALSE</data_collection>"
    "<RobotStatus>2</RobotStatus>"
    "<FORCEDATA>50</FORCEDATA>"
    "<IPOC>123645634563</IPOC>"
    "</Rob>"
)


class _WrenchAxisFilter:
    """补偿后 TCP 力/力矩的入口滤波：中值杀尖峰 + 一阶低通。

    实测 EMI 尖峰 1~3 帧 ±150N 级（独立测力计实测真实力 ≤50N），在数据入口
    一次滤除；显示（SSE/仪表）、力控、CSV 的 tcp_* 列共用同一份干净数据。
    sensor_* 列保持未滤波（干扰源诊断用）。"""

    def __init__(self, median_n: int, lpf_hz: float, cycle_s: float):
        self.win: "collections.deque[float]" = collections.deque(maxlen=max(1, median_n))
        if lpf_hz <= 0.0:
            self.alpha = 1.0
        else:
            tau = 1.0 / (2.0 * math.pi * lpf_hz)
            self.alpha = cycle_s / (tau + cycle_s)
        self.lpf: Optional[float] = None

    def push(self, value: float) -> float:
        self.win.append(value)
        data = sorted(self.win)
        n = len(data)
        mid = n // 2
        med = data[mid] if n % 2 else 0.5 * (data[mid - 1] + data[mid])
        self.lpf = med if self.lpf is None else self.lpf + self.alpha * (med - self.lpf)
        return self.lpf


class RSIServer:
    """KUKA RSI UDP 服务器"""

    # 对应机器人 RSI XML 的 SEND/ELEMENTS（机器人 -> 上位机）
    SEND_ELEMENTS = [
        ("Fx_raw", "LONG", 1),
        ("Fy_raw", "LONG", 2),
        ("Fz_raw", "LONG", 3),
        ("Mx_raw", "LONG", 4),
        ("My_raw", "LONG", 5),
        ("Mz_raw", "LONG", 6),
        ("Act_X", "DOUBLE", 7),
        ("Act_Y", "DOUBLE", 8),
        ("Act_Z", "DOUBLE", 9),
        ("Act_A", "DOUBLE", 10),
        ("Act_B", "DOUBLE", 11),
        ("Act_C", "DOUBLE", 12),
        ("data_collection", "BOOL", 13),
        ("RobotStatus", "INT", 14),   # 1=标定；2=钻孔（OV_PRO 力-速度）；3=凿击（恒力+横向让位+对中）
        ("FORCEDATA", "INT", 15),      # 恒力目标值 (N)：KRL 程序决定用多大的力钻孔/凿击；0=未下发
    ]

    # 对应机器人 RSI XML 的 RECEIVE/ELEMENTS（上位机 -> 机器人）
    RECEIVE_ELEMENTS = [
        ("RKorr.X", "DOUBLE", 1),
        ("RKorr.Y", "DOUBLE", 2),
        ("RKorr.Z", "DOUBLE", 3),
        ("RKorr.A", "DOUBLE", 4),
        ("RKorr.B", "DOUBLE", 5),
        ("RKorr.C", "DOUBLE", 6),
        ("OV_PRO", "DOUBLE", 7),  # Ethernet Out7 → Map2OV_PRO；量纲 0–100%，不是 mm/s
    ]

    CSV_HEADER = [
        "timestamp", "iPOC", "data_collection",
        "Fx_raw", "Fy_raw", "Fz_raw", "Mx_raw", "My_raw", "Mz_raw",
        "Act_X", "Act_Y", "Act_Z", "Act_A", "Act_B", "Act_C",
        "sensor_Fx_N", "sensor_Fy_N", "sensor_Fz_N", "sensor_Mx_Nm", "sensor_My_Nm", "sensor_Mz_Nm",
        "tcp_Fx_N", "tcp_Fy_N", "tcp_Fz_N", "tcp_Mx_Nm", "tcp_My_Nm", "tcp_Mz_Nm",
        "sample_status",
        "target_force_N", "FORCEDATA_N", "rkorr_x_mm", "rkorr_y_mm", "rkorr_z_mm",
        "rkorr_a_deg", "rkorr_b_deg", "rkorr_c_deg",
        "corr_cum_x_mm", "corr_cum_y_mm", "corr_cum_z_mm",
        "corr_cum_a_deg", "corr_cum_b_deg", "corr_cum_c_deg",
        "RobotStatus", "ov_pro_pct",
    ]

    def __init__(self, config: Optional[RSIConfig] = None, csv_filename: str = "rsi_data.csv"):
        self.config = config or RSIConfig()
        self.csv_filename = csv_filename
        self.sock: Optional[socket.socket] = None
        self.client_address: Optional[tuple] = None
        # 有界缓冲（最近 ~10s @250Hz），供 Web 监控读取；不再无限增长
        self.rsi_data_list: collections.deque[RSIData] = collections.deque(maxlen=2500)
        self.data_dir: str = "data"  # CSV 数据目录（_init_csv 时生效，可运行时改）
        self.csv_file = None
        self.csv_writer = None

        self.calibration_config: Optional[CalibrationConfig] = None
        self.calibration_runner: Optional[CalibrationRunner] = None
        self.force_controller: Optional[ForceController] = None
        self.force_mode = False
        # 自动重标定状态（换工具后示教器跑 FT_Calibration.src，服务无需重启/改模式）
        self.auto_calibrating = False  # 本轮标定由 data_collection=TRUE 边沿自动触发
        self.force_suspended = False   # 标定期间力控挂起：RKorr=0、OV_PRO=100%
        self.force_requested = False   # 服务以 --force 启动（标定完成后自动恢复力控）
        self._last_data_collection = False  # 上一帧 data_collection，用于上升沿检测
        self._last_robot_status = 2         # 上一帧 RobotStatus（INT），用于进入标定的沿检测
        # 调试台固定输出（Web /api/debug_override 设置，优先于力控）：
        # enabled/x/y/z/a/b/c（RKorr 每拍增量，mm/°）+ ov_pro（%）。仅此字典为数据源，
        # RSI 重启 / 自动标定会自动置 enabled=False（见 disable_debug_override）。
        self.debug_override: Optional[dict] = None
        self._debug_rx_since_enable = False  # 启用后是否已收到过 RSI 帧（预启用等首段会话，不算中断）
        self.tx_count = 0
        self.rx_count = 0
        self.parse_ok_count = 0
        self.packet_count = 0  # run() 主循环累计成功解析包数（Web 监控可读）
        self.last_rx_monotonic: Optional[float] = None  # 最近收包时刻（供 Web 监控判断链路活性）

        # Web 监控层可选回调：均为 best-effort，异常不拖垮控制环
        self.on_frame: Optional[Callable[[RSIData], None]] = None
        self.on_event: Optional[Callable[[str, str], None]] = None
        # 补偿后 TCP 力/力矩入口滤波（6 轴），参数变化时自动重建
        self._tcp_filters: list[_WrenchAxisFilter] = []
        self._tcp_filter_key: Optional[tuple] = None

    def _tcp_filters_ensure(self) -> "list[_WrenchAxisFilter]":
        fc = getattr(self.calibration_config, "force_control", None)
        median_n = int(getattr(fc, "tcp_filter_median_window", 5) if fc else 5)
        lpf_hz = float(getattr(fc, "tcp_filter_lpf_hz", 20.0) if fc else 20.0)
        cycle_s = float(getattr(fc, "cycle_s", 0.004) if fc else 0.004)
        key = (median_n, round(lpf_hz, 3), round(cycle_s, 5))
        if key != self._tcp_filter_key:
            self._tcp_filters = [_WrenchAxisFilter(median_n, lpf_hz, cycle_s) for _ in range(6)]
            self._tcp_filter_key = key
        return self._tcp_filters

    def _emit_event(self, level: str, message: str):
        """事件回调（level: info/warning/error）；回调异常一律吞掉，不影响控制环。"""
        if self.on_event is not None:
            try:
                self.on_event(level, message)
            except Exception:
                pass

    def start(self, enable_csv: bool = True):
        """启动 UDP 服务器"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 不复用端口：避免与调试助手等同时占用 59152 时“看似在听、实际收不到”
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        bind_ip = self.config.BIND_IP
        if bind_ip in ("0.0.0.0", ""):
            # RSI 发到 IP_NUMBER；优先绑到该地址，避免多网卡歧义
            bind_ip = self.config.IP_NUMBER
        server_address = (bind_ip, self.config.PORT)
        try:
            self.sock.bind(server_address)
        except OSError as error:
            raise OSError(
                f"无法绑定 {server_address[0]}:{server_address[1]}。"
                f"请关闭调试助手/其它占用该端口的程序后重试。原始错误: {error}"
            ) from error
        self.sock.settimeout(0.2)

        print("KUKA RSI UDP 服务器已启动")
        print(f"监听地址：{server_address[0]}:{server_address[1]}")
        print(f"本机 RSI IP（机器人 XML IP_NUMBER）：{self.config.IP_NUMBER}")
        print(f"预期机器人 IP：{self.config.ROBOT_IP}")
        print(f"SENTYPE={self.config.SENTYPE}  ONLYSEND={self.config.ONLYSEND}")
        if self.config.ONLYSEND:
            print("RSI 回复：关闭（只收不发，机器人会超时）")
        elif self.config.reply_zeros:
            print("RSI 回复：开启（Sen/RKorr 全 0，IPOC 回传接收值）")
        else:
            print(
                f"RSI 回复：开启（Sen/RKorr 每包随机 "
                f"{self.config.rkorr_min}~{self.config.rkorr_max}，IPOC 回传接收值）"
            )
        if enable_csv:
            print(f"CSV 保存文件：{self.csv_filename}")
        if self.calibration_config is not None:
            print(f"当前模式：{self.calibration_config.mode}")
            print(f"标定文件：{self.calibration_config.files.calibration_path}")
        print("注意：请先关闭调试助手；切换接收程序后请在示教器重新启动 RSI")
        print("\n按 Ctrl+C 停止服务器\n")

        if enable_csv:
            self._init_csv()

    def _init_csv(self):
        """初始化 CSV 文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_filename = os.path.join(self.data_dir, f"rsi_data_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_writer is None:
            raise RuntimeError("CSV writer initialization failed")
        self.csv_writer.writerow(self.CSV_HEADER)
        self.csv_file.flush()
        print(f"CSV 文件已创建：{self.csv_filename}")

    def parse_rsi_xml(self, xml_data: bytes) -> Optional[RSIData]:
        """解析 RSI XML 数据包"""
        try:
            xml_str = xml_data.decode("utf-8")
            root = ET.fromstring(xml_str)

            rsi_data = RSIData()
            rsi_data.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            ipoc_elem = root.find(".//IPOC")
            if ipoc_elem is not None and ipoc_elem.text:
                rsi_data.ipoc_text = ipoc_elem.text.strip()
                rsi_data.iPOC = int(rsi_data.ipoc_text)

            for tag, elem_type, _ in self.SEND_ELEMENTS:
                elem = root.find(f".//{tag}")
                if elem is not None and elem.text:
                    value = elem.text.strip()
                    if elem_type in ("LONG", "INT"):
                        try:
                            setattr(rsi_data, tag, int(value))
                        except ValueError:
                            if tag == "RobotStatus":
                                continue  # 旧 BOOL 文本，交给后面的协议兼容映射
                            raise
                    elif elem_type == "DOUBLE":
                        setattr(rsi_data, tag, float(value))
                    elif elem_type == "BOOL":
                        setattr(rsi_data, tag, value.upper() in ("1", "TRUE", "YES", "ON"))
                elif elem_type == "BOOL":
                    setattr(rsi_data, tag, False)

            # RobotStatus 协议升级（2026-10-09）：机器人端改为 INT（1=标定 2=钻孔 3=凿击）。
            # 兼容旧 BOOL 文本：TRUE->3（凿击）/ FALSE->2（钻孔）；缺省 2。
            rs_elem = root.find(".//RobotStatus")
            if rs_elem is not None and rs_elem.text:
                rs_text = rs_elem.text.strip().upper()
                try:
                    rsi_data.RobotStatus = int(rs_text)
                except ValueError:
                    rsi_data.RobotStatus = 3 if rs_text in ("TRUE", "YES", "ON") else 2
            else:
                rsi_data.RobotStatus = 2

            # 可选：XML 另加 <target_force> 时可在线改目标力；当前机器人 SEND 无此标签
            tf_elem = root.find(".//target_force")
            if tf_elem is not None and tf_elem.text:
                rsi_data.target_force = float(tf_elem.text.strip())
                rsi_data.target_force_present = True

            return rsi_data

        except ET.ParseError as error:
            print(f"XML 解析错误：{error}")
            self._emit_event("error", f"XML 解析错误：{error}")
            return None
        except (ValueError, AttributeError) as error:
            print(f"数据解析错误：{error}")
            self._emit_event("error", f"数据解析错误：{error}")
            return None

    def generate_response(self, rsi_data: RSIData) -> str:
        """按 KST RSI 4.0 §6.5.3/6.5.4 生成传感器回包。

        TAG 带点号为属性书写形式：RKorr.X → <RKorr X="..." />。
        IPOC 必须与机器人刚发来的时间戳一致，否则数据包无效。
        """
        groups: dict[str, list[tuple[str, str]]] = {}
        group_order: list[str] = []
        scalar_tags: list[tuple[str, str]] = []

        for tag, _, _ in self.RECEIVE_ELEMENTS:
            if tag.startswith("RKorr."):
                if self.force_mode or self._debug_override_active():
                    value = self.config.rkorr.get(tag, 0.0)
                elif self.config.reply_zeros:
                    value = 0.0
                else:
                    value = random.uniform(self.config.rkorr_min, self.config.rkorr_max)
                self.config.rkorr[tag] = value
            elif tag == "OV_PRO":
                value = self.config.ov_pro
            else:
                value = 0.0
            text = f"{value:.4f}"
            if "." in tag:
                elem, attr = tag.split(".", 1)
                if elem not in groups:
                    groups[elem] = []
                    group_order.append(elem)
                groups[elem].append((attr, text))
            else:
                scalar_tags.append((tag, text))

        lines = [f'<Sen Type="{self.config.SENTYPE}">']
        for elem in group_order:
            attrs = " ".join(f'{attr}="{text}"' for attr, text in groups[elem])
            lines.append(f"<{elem} {attrs} />")
        for tag, text in scalar_tags:
            lines.append(f"<{tag}>{text}</{tag}>")
        lines.append(f"<IPOC>{rsi_data.ipoc_text}</IPOC>")
        lines.append("</Sen>")
        return "\n".join(lines)

    def _check_auto_calibration(self, rsi_data: RSIData):
        """换工具重标定：data_collection 的 FALSE->TRUE 上升沿触发（任何模式），
        自动清空旧样本并进入标定模式（服务无需重启/改模式）。
        必须用上升沿而非电平：求解完成当拍该段残留的 TRUE 不得重新触发。"""
        data_collection_rise = rsi_data.data_collection and not self._last_data_collection
        robot_status_enter_calib = rsi_data.RobotStatus == 1 and self._last_robot_status != 1
        if (
            not (data_collection_rise or robot_status_enter_calib)
            or self.auto_calibrating
            or self.calibration_config is None
            or self.calibration_runner is None
            or self.calibration_config.mode == "calibration_collect"
        ):
            return
        self.calibration_runner.samples.clear()
        self.calibration_runner.last_sample_angles = None
        self.calibration_config.mode = "calibration_collect"
        self.auto_calibrating = True
        source = "RobotStatus=1" if robot_status_enter_calib else "data_collection=TRUE"
        message = f"[标定] 检测到标定程序信号（{source}），自动进入标定模式"
        print(message)
        self._emit_event("info", message)
        # 标定走固定姿态，调试固定输出必须让位
        self.disable_debug_override("检测到标定信号")
        if self.force_mode and self.force_controller is not None:
            # 标定期间挂起力控：RKorr 全 0、倍率回 100%，防止旧标定补偿误动
            self.force_controller.reset()
            for tag in self.config.rkorr:
                self.config.rkorr[tag] = 0.0
            self.config.ov_pro = 100.0
            self.force_suspended = True
            message = "[标定] 力控已挂起，RKorr=0、OV_PRO=100%"
            print(message)
            self._emit_event("warning", message)

    def _check_force_restore(self):
        """标定求解完成（runner 已把 mode 切回 calibrated_runtime）后，
        若服务以 --force 启动则自动恢复力控。"""
        if (
            not self.auto_calibrating
            or self.calibration_config is None
            or self.calibration_config.mode != "calibrated_runtime"
        ):
            return
        self.auto_calibrating = False
        if not self.force_requested:
            return
        self.force_suspended = False
        if self.force_controller is None:
            self.force_controller = ForceController(
                self.calibration_config.force_control,
                rsi_rotation_order=self.calibration_config.rsi_rotation_order,
            )
        self.force_controller.reset()
        self.force_mode = True
        message = "[标定] 标定完成，力控已自动恢复"
        print(message)
        self._emit_event("info", message)

    def process_frame(self, rsi_data: RSIData) -> RSIData:
        self._check_auto_calibration(rsi_data)
        self._last_data_collection = rsi_data.data_collection
        self._last_robot_status = rsi_data.RobotStatus
        frame = {
            "timestamp": rsi_data.timestamp,
            "iPOC": rsi_data.iPOC,
            "data_collection": rsi_data.data_collection,
            "Fx_raw": rsi_data.Fx_raw,
            "Fy_raw": rsi_data.Fy_raw,
            "Fz_raw": rsi_data.Fz_raw,
            "Mx_raw": rsi_data.Mx_raw,
            "My_raw": rsi_data.My_raw,
            "Mz_raw": rsi_data.Mz_raw,
            "Act_X": rsi_data.Act_X / 1000.0,
            "Act_Y": rsi_data.Act_Y / 1000.0,
            "Act_Z": rsi_data.Act_Z / 1000.0,
            "Act_A": rsi_data.Act_A,
            "Act_B": rsi_data.Act_B,
            "Act_C": rsi_data.Act_C,
        }

        if self.calibration_runner is None:
            rsi_data.sample_status = "no_calibration"
            return rsi_data

        processed = self.calibration_runner.process_frame(frame)
        self._check_force_restore()
        sensor_wrench = processed.get("sensor_wrench", [0.0] * 6)
        calibrated_wrench: CalibratedWrench | None = processed.get("calibrated_wrench")

        rsi_data.sensor_fx, rsi_data.sensor_fy, rsi_data.sensor_fz = sensor_wrench[:3]
        rsi_data.sensor_mx, rsi_data.sensor_my, rsi_data.sensor_mz = sensor_wrench[3:6]
        rsi_data.sample_status = processed.get("sample_status", "streaming")

        if calibrated_wrench is not None:
            values = list(calibrated_wrench.force_tcp) + list(calibrated_wrench.torque_tcp)
            fc = getattr(self.calibration_config, "force_control", None)
            if fc is None or getattr(fc, "tcp_filter_enable", True):
                filters = self._tcp_filters_ensure()
                values = [f.push(v) for f, v in zip(filters, values)]
            rsi_data.tcp_fx, rsi_data.tcp_fy, rsi_data.tcp_fz = values[0:3]
            rsi_data.tcp_mx, rsi_data.tcp_my, rsi_data.tcp_mz = values[3:6]

        return rsi_data

    def save_to_csv(self, rsi_data: RSIData):
        """保存数据到 CSV"""
        if self.csv_writer:
            row = [
                rsi_data.timestamp,
                rsi_data.iPOC,
                1 if rsi_data.data_collection else 0,
                rsi_data.Fx_raw, rsi_data.Fy_raw, rsi_data.Fz_raw,
                rsi_data.Mx_raw, rsi_data.My_raw, rsi_data.Mz_raw,
                rsi_data.Act_X, rsi_data.Act_Y, rsi_data.Act_Z,
                rsi_data.Act_A, rsi_data.Act_B, rsi_data.Act_C,
                rsi_data.sensor_fx, rsi_data.sensor_fy, rsi_data.sensor_fz,
                rsi_data.sensor_mx, rsi_data.sensor_my, rsi_data.sensor_mz,
                rsi_data.tcp_fx, rsi_data.tcp_fy, rsi_data.tcp_fz,
                rsi_data.tcp_mx, rsi_data.tcp_my, rsi_data.tcp_mz,
                rsi_data.sample_status,
                rsi_data.target_force,
                rsi_data.FORCEDATA,
                self.config.rkorr["RKorr.X"], self.config.rkorr["RKorr.Y"], self.config.rkorr["RKorr.Z"],
                self.config.rkorr["RKorr.A"], self.config.rkorr["RKorr.B"], self.config.rkorr["RKorr.C"],
                *(
                    self.force_controller.corr_cumulative_mm
                    if self.force_controller is not None
                    else [0.0, 0.0, 0.0]
                ),
                *(
                    self.force_controller.corr_cumulative_deg
                    if self.force_controller is not None
                    else [0.0, 0.0, 0.0]
                ),
                rsi_data.RobotStatus,
                self.config.ov_pro,
            ]
            self.csv_writer.writerow(row)
            if self.csv_file is not None:
                self.csv_file.flush()

    def _reply(self, rsi_data: RSIData, address: tuple) -> str:
        if self.sock is None or self.config.ONLYSEND:
            return ""
        response_xml = self.generate_response(rsi_data)
        self.sock.sendto(response_xml.encode("utf-8"), address)
        self.tx_count += 1
        return response_xml

    def _debug_override_active(self) -> bool:
        return self.debug_override is not None and bool(self.debug_override.get("enabled"))

    def _apply_debug_override(self) -> bool:
        """调试台固定输出：直接改写本拍 RKorr/OV_PRO（优先于力控，任何模式生效）。"""
        if not self._debug_override_active():
            return False
        values = self.debug_override
        for axis in "XYZABC":
            self.config.rkorr[f"RKorr.{axis}"] = float(values[axis.lower()])
        self.config.ov_pro = float(values["ov_pro"])
        return True

    def disable_debug_override(self, reason: str):
        """停止固定输出：数值清零、倍率回 100%、力控器复位（跳帧期间其滤波/累积已过期）。

        #RELATIVE 语义：停止后发 0 只是保持叠加，已产生的偏移不会自动撤销。"""
        if not self._debug_override_active():
            return
        fresh = dict(self.debug_override or {})
        fresh.update({"enabled": False, "x": 0.0, "y": 0.0, "z": 0.0,
                      "a": 0.0, "b": 0.0, "c": 0.0, "ov_pro": 100.0})
        self.debug_override = fresh
        self._debug_rx_since_enable = False
        for tag in self.config.rkorr:
            self.config.rkorr[tag] = 0.0
        self.config.ov_pro = 100.0
        if self.force_controller is not None:
            self.force_controller.reset()
        message = f"[调试] 固定输出已停止（{reason}）：RKorr=0、OV_PRO=100%"
        print(message)
        self._emit_event("warning", message)

    SERVICE_MODES = ("monitor", "force")

    def service_mode(self) -> str:
        """当前服务模式：monitor=监控（重力补偿+记录）；force=力控（OV_PRO 恒力等）。"""
        return "force" if self.force_mode else "monitor"

    def set_service_mode(self, mode: str):
        """Web 层运行时切换服务模式（免重启）。

        monitor：力控关闭，重力补偿与记录继续；
        force：力控开启（需已有标定结果），力控器不存在则按当前配置创建。
        切换即操作意图变化：清空本拍修正、复位力控器、停用调试固定输出。
        """
        if mode not in self.SERVICE_MODES:
            raise ValueError(f"未知模式 {mode!r}（可选 {'/'.join(self.SERVICE_MODES)}）")
        if mode == "force":
            if (
                self.calibration_runner is None
                or self.calibration_runner.calibration_result is None
            ):
                raise ValueError("无标定结果，无法启用力控（请先完成标定）")
            if self.calibration_config is not None:
                self.calibration_config.mode = "calibrated_runtime"
            self.force_mode = True
            self.force_requested = True
            if not self.auto_calibrating:
                self.force_suspended = False
                if self.force_controller is None:
                    self.force_controller = ForceController(
                        self.calibration_config.force_control,
                        rsi_rotation_order=self.calibration_config.rsi_rotation_order,
                    )
        else:  # monitor
            self.force_mode = False
            self.force_requested = False
            self.force_suspended = False
        # 模式切换统一清理：停固定输出、复位力控器、修正清零、倍率回 100%
        self.disable_debug_override("切换服务模式")
        if self.force_controller is not None:
            self.force_controller.reset()
        for tag in self.config.rkorr:
            self.config.rkorr[tag] = 0.0
        self.config.ov_pro = 100.0
        label = "力控" if mode == "force" else "监控（补偿记录）"
        message = f"[模式] 服务已切换为{label}（Web 在线切换，无需重启）"
        print(message)
        self._emit_event("warning" if mode == "force" else "info", message)

    def _update_force_reply(self, rsi_data: RSIData):
        """力控模式：根据补偿后的 TCP 力计算 RKorr，写入 config.rkorr 供本周期回包。"""
        assert self.force_controller is not None
        assert self.calibration_config is not None
        fc_cfg = self.calibration_config.force_control
        if rsi_data.FORCEDATA > 0:
            # FORCEDATA（SEND IND 15）：KRL 程序下发的恒力目标值，优先级最高
            target = float(rsi_data.FORCEDATA)
        elif rsi_data.target_force_present:
            target = rsi_data.target_force
        else:
            target = fc_cfg.default_target_force_n
        rkorr = self.force_controller.update(
            force_tcp=[rsi_data.tcp_fx, rsi_data.tcp_fy, rsi_data.tcp_fz],
            tcp_angles_deg=[rsi_data.Act_A, rsi_data.Act_B, rsi_data.Act_C],
            target_force_n=target,
            tcp_position_mm=[rsi_data.Act_X, rsi_data.Act_Y, rsi_data.Act_Z],
            mode="chisel" if rsi_data.RobotStatus == 3 else "drill",
            torque_tcp=[rsi_data.tcp_mx, rsi_data.tcp_my, rsi_data.tcp_mz],
        )
        self.config.rkorr.update(rkorr)
        self.config.ov_pro = self.force_controller.ov_pro_pct

    def self_test_xml(self) -> bool:
        """不依赖机器人，校验解析与回包格式。"""
        parsed = self.parse_rsi_xml(SAMPLE_ROB_XML.encode("utf-8"))
        if parsed is None:
            print("[自检失败] 无法解析示例 SEND XML")
            return False
        reply = self.generate_response(parsed)
        rkorr_tags = [tag for tag, _, _ in self.RECEIVE_ELEMENTS if tag.startswith("RKorr.")]
        if self.config.reply_zeros:
            rkorr_ok = all(self.config.rkorr[tag] == 0.0 for tag in rkorr_tags)
            rkorr_text_ok = 'X="0.0000"' in reply and 'C="0.0000"' in reply
        else:
            rkorr_ok = all(
                self.config.rkorr_min <= self.config.rkorr[tag] <= self.config.rkorr_max
                for tag in rkorr_tags
            )
            rkorr_text_ok = 'X="' in reply and 'C="' in reply
        ok = (
            parsed.Fx_raw == 100
            and parsed.Act_C == 179.8
            and parsed.ipoc_text == "123645634563"
            and parsed.RobotStatus == 2
            and f'Type="{self.config.SENTYPE}"' in reply
            and "<RKorr " in reply
            and rkorr_text_ok
            and "<RKorr.X>" not in reply
            and rkorr_ok
            and "<OV_PRO>100.0000</OV_PRO>" in reply
            and "<IPOC>123645634563</IPOC>" in reply
        )
        print("=== XML 自检 ===")
        print(f"  解析 SEND: Fx_raw={parsed.Fx_raw} Act_C={parsed.Act_C} IPOC={parsed.ipoc_text}")
        print(f"  生成 RECEIVE: {reply}")
        print(f"  结果: {'通过' if ok else '失败'}")
        return ok

    def preflight(self) -> tuple[bool, bool]:
        """检查本机 IP、机器人 Ping、配置是否与 RSI XML 一致。"""
        print("=== 链路预检 ===")
        host_ok = local_ip_exists(self.config.IP_NUMBER)
        print(f"  本机拥有 {self.config.IP_NUMBER}: {'是' if host_ok else '否'}")
        if not host_ok:
            print("  机器人会把包发到该 IP，本机没有这个地址则收不到 UDP")
        robot_ok = ping_host(self.config.ROBOT_IP)
        print(f"  Ping 机器人 {self.config.ROBOT_IP}: {'通' if robot_ok else '不通'}")
        print(f"  监听 {self.config.BIND_IP}:{self.config.PORT}  SENTYPE={self.config.SENTYPE}  ONLYSEND={self.config.ONLYSEND}")
        print("  SEND 标签: " + ", ".join(tag for tag, _, _ in self.SEND_ELEMENTS))
        print("  RECEIVE 标签: " + ", ".join(tag for tag, _, _ in self.RECEIVE_ELEMENTS))
        return host_ok, robot_ok

    def self_test_udp_loopback(self) -> bool:
        """向本机 127.0.0.1:PORT 发一包，确认 bind/解析/回发闭环。"""
        if self.sock is None:
            return False
        print("=== 本机 UDP 回环 ===")
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(1.0)
        dest = ("127.0.0.1", self.config.PORT)
        try:
            client.sendto(SAMPLE_ROB_XML.encode("utf-8"), dest)
            self.sock.settimeout(1.0)
            data, address = self.sock.recvfrom(4096)
            parsed = self.parse_rsi_xml(data)
            if parsed is None:
                print("  结果: 失败（解析回环包失败）")
                return False
            self._reply(parsed, address)
            echoed, _ = client.recvfrom(4096)
            echo_text = echoed.decode("utf-8", errors="replace")
            ok = "<IPOC>123645634563</IPOC>" in echo_text and 'Type="ImFree"' in echo_text
            print(f"  发到 {dest}，服务端收到 {address}，{len(data)} 字节")
            print(f"  回包: {echo_text}")
            print(f"  结果: {'通过' if ok else '失败'}")
            return ok
        except socket.timeout:
            print("  结果: 失败（1 秒内未收到回包，检查端口占用或防火墙）")
            return False
        finally:
            client.close()
            self.sock.settimeout(0.2)
            self.tx_count = 0
            self.rx_count = 0
            self.parse_ok_count = 0

    def run_link_test(self, wait_seconds: float = 15.0, hold_seconds: float = 3.0) -> bool:
        """先自检 XML，再等待真实 RSI 包并回发，验证整条 UDP 闭环。"""
        xml_ok = self.self_test_xml()
        host_ok, robot_ping = self.preflight()
        self.start(enable_csv=False)
        if self.sock is None:
            raise RuntimeError("UDP socket initialization failed")
        loopback_ok = self.self_test_udp_loopback()

        print(f"=== 等待机器人 UDP（最多 {wait_seconds:.0f}s）===")
        print("请在示教器启动 RSI 程序")
        first_ipoc = ""
        last_ipoc = ""
        first_raw = ""
        first_reply = ""
        t0 = time.monotonic()
        first_at: Optional[float] = None
        last_at: Optional[float] = None

        try:
            while True:
                now = time.monotonic()
                if first_at is None and now - t0 >= wait_seconds:
                    break
                if first_at is not None and now - first_at >= hold_seconds:
                    break
                try:
                    data, address = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue

                self.rx_count += 1
                last_at = time.monotonic()
                if first_at is None:
                    first_at = last_at
                    first_raw = data.decode("utf-8", errors="replace")
                    print(f"\n收到来自 {address} 的第一包，{len(data)} 字节")
                    print(f"  原文: {first_raw[:500]}")

                rsi_data = self.parse_rsi_xml(data)
                if rsi_data is None:
                    continue
                self.parse_ok_count += 1
                reply = self._reply(rsi_data, address)
                last_ipoc = rsi_data.ipoc_text
                if not first_ipoc:
                    first_ipoc = rsi_data.ipoc_text
                    first_reply = reply
                    print(f"  解析 IPOC={rsi_data.ipoc_text} Fx_raw={rsi_data.Fx_raw} Act_X={rsi_data.Act_X}")
                    print(f"  已回发: {reply}")
        except KeyboardInterrupt:
            print("\n链路测试被中断")
        finally:
            self.stop()

        elapsed = (last_at - first_at) if first_at and last_at else 0.0
        rate = (self.rx_count / elapsed) if elapsed > 0 else 0.0
        print("\n=== 链路测试结果 ===")
        print(f"  XML 自检: {'通过' if xml_ok else '失败'}")
        print(f"  本机 IP {self.config.IP_NUMBER}: {'通过' if host_ok else '失败'}")
        print(f"  Ping 机器人 {self.config.ROBOT_IP}: {'通' if robot_ping else '不通'}")
        print(f"  本机 UDP 回环: {'通过' if loopback_ok else '失败'}")
        print(f"  机器人收包: {self.rx_count}  解析成功: {self.parse_ok_count}  回发: {self.tx_count}")
        if self.rx_count:
            print(f"  IPOC: {first_ipoc} -> {last_ipoc}")
            print(f"  约 {rate:.0f} 包/秒（RSI 周期 4ms 时应接近 250）")
        live_ok = self.rx_count > 0 and self.tx_count > 0 and self.parse_ok_count > 0
        if live_ok:
            print("  结论: 机器人 <-> 上位机 UDP 双向联通")
        else:
            print("  结论: 上位机收发栈已就绪，但未收到机器人 UDP。请在示教器启动 RSI")
        return xml_ok and host_ok and loopback_ok and live_ok

    def run(self):
        """运行服务器主循环"""
        try:
            self.start()
            if self.sock is None:
                raise RuntimeError("UDP socket initialization failed")
            self.packet_count = 0
            wait_started = time.monotonic()
            last_wait_print = wait_started
            self.last_rx_monotonic = None
            seen_sources: set[tuple] = set()

            while True:
                try:
                    data, address = self.sock.recvfrom(4096)

                    # #RELATIVE：机器人叠加在 RSI_ON 时从 0 起。收包中断 >1s
                    # 判定上下文已重建，PC 累积必须同步清零，否则会按旧总偏
                    # 移继续发增量，把机器人推到错误位置。
                    now_rx = time.monotonic()
                    if (
                        self.last_rx_monotonic is not None
                        and now_rx - self.last_rx_monotonic > 1.0
                    ):
                        if self.force_mode and self.force_controller is not None:
                            self.force_controller.reset()
                            print("[力控] 收包中断 >1s，判定 RSI 重启，修正累积已清零")
                            self._emit_event("warning", "[力控] 收包中断 >1s，判定 RSI 重启，修正累积已清零")
                        # 调试固定输出：随会话运行过才判定失效（机器人叠加已归零，续发会突然起跳）；
                        # 预启用等待首段 RSI 会话的（启用后还没收到过帧）不打断
                        if self._debug_override_active() and self._debug_rx_since_enable:
                            self.disable_debug_override("收包中断 >1s，判定 RSI 重启")
                        # 入口滤波历史一并清空（会话重建，从新值起滤）
                        self._tcp_filter_key = None
                    self.last_rx_monotonic = now_rx
                    if self._debug_override_active():
                        self._debug_rx_since_enable = True
                    else:
                        self._debug_rx_since_enable = False

                    if address not in seen_sources:
                        seen_sources.add(address)
                        print(f"\n收到来自 {address} 的 UDP（源 {len(seen_sources)}）")
                        self._emit_event("info", f"收到来自 {address} 的 UDP")
                    if self.client_address is None:
                        self.client_address = address

                    rsi_data = self.parse_rsi_xml(data)

                    if rsi_data:
                        self.packet_count += 1
                        # 力控生效需：有力控器 + 未挂起 + 已有标定结果（无结果时无补偿值，力控无意义）
                        force_active = (
                            self.force_mode
                            and self.force_controller is not None
                            and not self.force_suspended
                            and self.calibration_runner is not None
                            and self.calibration_runner.calibration_result is not None
                        )
                        if force_active:
                            # 力控模式：先用当前帧算 RKorr，再回包（同一 IPOC）
                            rsi_data = self.process_frame(rsi_data)
                            if self.force_suspended:
                                # 本帧刚触发自动标定：RKorr 全 0、倍率回 100%
                                for tag in self.config.rkorr:
                                    self.config.rkorr[tag] = 0.0
                                self.config.ov_pro = 100.0
                            elif self._apply_debug_override():
                                pass  # 调试台固定输出优先于力控
                            else:
                                self._update_force_reply(rsi_data)
                            response_xml = self._reply(rsi_data, address)
                        else:
                            if self.force_mode:
                                # 力控挂起中（自动标定进行中或暂无标定结果）：RKorr=0、OV_PRO=100%
                                self.config.ov_pro = 100.0
                            # 调试固定输出在任何模式下都改写本拍 RKorr/OV_PRO
                            self._apply_debug_override()
                            # RSI 4ms 周期：先回传同一 IPOC，再做标定/写盘
                            response_xml = self._reply(rsi_data, address)
                            rsi_data = self.process_frame(rsi_data)

                        if self.packet_count == 1:
                            print(f"  原文: {data.decode('utf-8', errors='replace')[:500]}")
                            if response_xml:
                                print(f"  RSI 回复已发送: {response_xml}")

                        if self.packet_count % 100 == 1:
                            print(f"\n已接收 {self.packet_count} 个数据包")
                            print(f"  IPOC={rsi_data.iPOC}")
                            print(f"  Fx_raw={rsi_data.Fx_raw}, Fy_raw={rsi_data.Fy_raw}, Fz_raw={rsi_data.Fz_raw}")
                            print(f"  Mx_raw={rsi_data.Mx_raw}, My_raw={rsi_data.My_raw}, Mz_raw={rsi_data.Mz_raw}")
                            print(f"  Act_X={rsi_data.Act_X:.3f}, Act_Y={rsi_data.Act_Y:.3f}, Act_Z={rsi_data.Act_Z:.3f}")
                            print(f"  Act_A={rsi_data.Act_A:.3f}, Act_B={rsi_data.Act_B:.3f}, Act_C={rsi_data.Act_C:.3f}")
                            print(
                                f"  data_collection={rsi_data.data_collection}  "
                                f"RobotStatus={rsi_data.RobotStatus}"
                            )
                            print(f"  Sensor(N/Nm)=({rsi_data.sensor_fx:.3f}, {rsi_data.sensor_fy:.3f}, {rsi_data.sensor_fz:.3f}, {rsi_data.sensor_mx:.3f}, {rsi_data.sensor_my:.3f}, {rsi_data.sensor_mz:.3f})")
                            print(f"  TCP补偿后(N/Nm)=({rsi_data.tcp_fx:.3f}, {rsi_data.tcp_fy:.3f}, {rsi_data.tcp_fz:.3f}, {rsi_data.tcp_mx:.3f}, {rsi_data.tcp_my:.3f}, {rsi_data.tcp_mz:.3f})")
                            rk = self.config.rkorr
                            parts = []
                            for tag, _, _ in self.RECEIVE_ELEMENTS:
                                if tag.startswith("RKorr."):
                                    parts.append(f"{tag}={rk[tag]:.4f}")
                                elif tag == "OV_PRO":
                                    parts.append(f"OV_PRO={self.config.ov_pro:.1f}")
                            print("  回发 " + " ".join(parts))
                            print(f"  状态={rsi_data.sample_status}")
                            if self.force_mode and self.force_controller is not None:
                                if rsi_data.target_force_present:
                                    tgt_n = rsi_data.target_force
                                elif self.calibration_config is not None:
                                    tgt_n = self.calibration_config.force_control.default_target_force_n
                                else:
                                    tgt_n = 0.0
                                print(f"  目标力={tgt_n:.1f} N  "
                                      f"{self.force_controller.status_line}")

                        self.save_to_csv(rsi_data)
                        self.rsi_data_list.append(rsi_data)
                        if self.on_frame is not None:
                            try:
                                self.on_frame(rsi_data)
                            except Exception:
                                pass

                except socket.timeout:
                    if self.packet_count == 0:
                        now = time.monotonic()
                        if now - last_wait_print >= 2.0:
                            waited = now - wait_started
                            print(
                                f"等待机器人 UDP 中… {waited:.0f}s "
                                f"(期望源 {self.config.ROBOT_IP} -> "
                                f"{self.config.IP_NUMBER}:{self.config.PORT}；"
                                f"若刚关调试助手，请在示教器重新启动 RSI)"
                            )
                            last_wait_print = now
                    continue

        except KeyboardInterrupt:
            print("\n\n服务器已停止")
            print(f"共接收 {len(self.rsi_data_list)} 个数据包")
            print(f"数据已保存到：{self.csv_filename}")
        finally:
            self.stop()

    def stop(self):
        """停止服务器"""
        if self.csv_file is not None:
            self.csv_file.close()
        if self.sock:
            self.sock.close()
            print("UDP 套接字已关闭")


def create_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器"""
    parser = argparse.ArgumentParser(
        description="KUKA RSI 力传感器标定与数据采集系统"
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="启动标定模式：采集多个静止姿态，自动求解标定参数"
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="启动运行模式：使用已有标定结果进行实时重力补偿"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="启动力控模式（RobotStatus: 1=标定 2=钻孔控OV_PRO 3=凿击恒力+横向让位）"
    )
    parser.add_argument(
        "--ip",
        type=str,
        default="192.168.2.10",
        help="机器人 IP（RSI 数据源，默认：192.168.2.10）"
    )
    parser.add_argument(
        "--host-ip",
        type=str,
        default="192.168.2.250",
        help="本机 IP，对应机器人 RSI XML 的 IP_NUMBER（默认：192.168.2.250）"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=59152,
        help="UDP 监听端口（默认：59152）"
    )
    parser.add_argument(
        "--test-link",
        action="store_true",
        help="只做链路联通测试：XML 自检 + 等待真实 RSI 包并回发后退出"
    )
    parser.add_argument(
        "--test-seconds",
        type=float,
        default=15.0,
        help="链路测试等待第一包的最长时间（秒）"
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=8080,
        help="Web 监控端口（默认 8080）"
    )
    parser.add_argument(
        "--no-web",
        action="store_true",
        help="不启动 Web 监控服务"
    )
    parser.add_argument(
        "--data-cap-mb",
        type=int,
        default=2048,
        help="CSV 数据目录容量上限 MB，超出自动删最旧未锁定文件（默认 2048）"
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default="data",
        help="CSV 数据目录（默认 data）"
    )
    return parser


def main():
    parser = create_parser()
    args = parser.parse_args()

    config = RSIConfig(
        IP_NUMBER=args.host_ip,
        PORT=args.port,
        SENTYPE="ImFree",
        ONLYSEND=False,
        ROBOT_IP=args.ip,
    )

    if args.test_link:
        print("=== 链路联通测试 ===")
        # 链路测试仍可随机 RKorr，便于确认回包字段在变
        config.reply_zeros = False
        server = RSIServer(config=config)
        ok = server.run_link_test(wait_seconds=args.test_seconds)
        sys.exit(0 if ok else 1)

    # 标定/运行：收包必须回包，RKorr 全 0，避免扰动机器人
    config.reply_zeros = True
    config.ONLYSEND = False

    calibration_config = load_config()

    if args.calibrate:
        calibration_config.mode = "calibration_collect"
        print("=== 标定模式 ===")
        print(f"最少样本数：{calibration_config.static_detection.min_samples}")
        print(f"最多样本数：{calibration_config.static_detection.max_samples}")
        print(f"最短采集时长：{calibration_config.static_detection.min_dwell_seconds}s")
        print("RSI 回包：RKorr 全 0（收包即回，维持双向闭环）")
        print("操作：每个姿态保持静止约 0.5s（data_collection 建议为 TRUE）")
        print("说明：UDP 中若 FALSE 未到达，静止满时长也会自动生成样本；姿态需分散")
        print("达到最少样本数后自动求解并切换到 TCP 补偿运行模式\n")
    elif args.run:
        calibration_config.mode = "calibrated_runtime"
        print("=== 运行模式 ===")
        print("使用已有标定结果进行实时重力补偿")
        print("RSI 回包：RKorr 全 0（收包即回，维持双向闭环）\n")
    elif args.force:
        calibration_config.mode = "calibrated_runtime"
        fc = calibration_config.force_control
        print("=== 力控模式（恒力打磨） ===")
        print(f"压紧轴：TOOL {fc.axis}（press_sign={fc.press_sign}）")
        print(f"RKorr 坐标系：{fc.rkorr_frame}（须与 PosCorr.RefCorrSys 一致）")
        print(f"目标力：默认 {fc.default_target_force_n} N（RSI XML 含 target_force 时可在线改）")
        print(f"增益：Kp={fc.kp_mm_per_s_per_n} (mm/s)/N, Ki={fc.ki_mm_per_s2_per_n} (mm/s²)/N")
        print(f"力滤波：中值 {fc.filter_median_window} 帧 + 低通 {fc.filter_lpf_hz} Hz")
        print(f"路径进给补偿：{fc.path_feed_mm_s} mm/s（须与 $VEL.CP 一致）")
        print(f"限幅：每拍增量 ±{fc.per_cycle_max_mm} mm（PosCorr），累积 ±{fc.cumulative_max_mm} mm（POSCORRMON），超力保护 {fc.max_force_n} N")
        print(f"倍率：RobotStatus=2 钻孔按力映射 $OV_PRO 0–100%（每拍 ≤{fc.ov_pro_slew_pct:.1f}%）；1=自动标定；3=凿击")
        print(f"凿击（TRUE）：X 仍 OV_PRO 恒力；Y/Z 横向让位阈值 {fc.chisel_lateral_deadband_n} N、行程 ±{fc.chisel_lateral_max_mm} mm、卡滞保护 {fc.chisel_lateral_trip_n} N、方向符号 {fc.chisel_lateral_sign:+d}（横向调节期间 X 恒力冻结）")
        print(f"轴线对中（B←My、C←Mz）：凿击{'开' if fc.align_chisel_enable else '关'}、钻孔{'开' if fc.align_drill_enable else '关'}；死区 {fc.align_deadband_nm} N·m、增益 {fc.align_gain_deg_per_s_per_nm} °/s/N·m、累计 ±{fc.align_max_deg}°、力矩保护 {fc.align_trip_nm} N·m、方向符号 {fc.align_sign:+d}（符号未经实机验证前勿开启）")
        print("RSI 回包：钻孔 RKorr=0 + OV_PRO（须 RSI_ON(#RELATIVE)；Ethernet Out7→Map2OV_PRO 量程 0–100）\n")
    else:
        calibration_config.mode = "record_only"
        print("=== 仅记录模式 ===")
        print("记录原始数据，不做标定或补偿")
        print("RSI 回包：RKorr 全 0（收包即回，维持双向闭环）\n")

    server = RSIServer(config=config)
    server.data_dir = args.data_dir
    server.calibration_config = calibration_config
    server.calibration_runner = CalibrationRunner(calibration_config)

    calibration_result = load_calibration_result(calibration_config.files.calibration_path)
    if calibration_result is not None:
        server.calibration_runner.calibration_result = calibration_result
        if calibration_config.mode == "record_only":
            calibration_config.mode = "calibrated_runtime"
            print("检测到已有标定结果，自动启用补偿")

    if args.force:
        server.force_requested = True
        server.force_mode = True
        if calibration_result is None:
            # 无标定结果不再退出：力控挂起待激活，自动标定完成后恢复
            # （覆盖"换工具→重新标定→恢复生产"全流程，服务常驻即可）
            server.force_suspended = True
            print("警告：暂无标定结果，力控待标定完成后自动激活")
            print("      在示教器运行 FT_Calibration.src 即可自动完成标定并恢复力控\n")
        else:
            server.force_controller = ForceController(
                calibration_config.force_control,
                rsi_rotation_order=calibration_config.rsi_rotation_order,
            )

    # Web 监控层由独立模块提供；缺失/启动失败不影响主控制环
    if not args.no_web:
        try:
            from web_monitor import start_web_server
            start_web_server(server, data_dir=args.data_dir, data_cap_mb=args.data_cap_mb, port=args.web_port)
        except Exception as error:
            print(f"警告：Web 监控启动失败（{error}），继续运行主程序")

    server.run()


if __name__ == "__main__":
    main()
