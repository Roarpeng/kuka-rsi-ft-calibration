from __future__ import annotations

import argparse
import csv
import socket
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from typing import TYPE_CHECKING, Optional

from calibration_io import load_calibration_result, load_config
from calibration_models import CalibratedWrench
from calibration_runner import CalibrationRunner

if TYPE_CHECKING:
    from calibration_models import CalibrationConfig


@dataclass
class RSIConfig:
    """RSI 配置参数"""
    IP_NUMBER: str = "192.168.2.20"
    PORT: int = 59152
    SENTYPE: str = "ImFree"
    ONLYSEND: bool = True


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
    in_position: bool = False  # 机器人到位标记
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


class RSIServer:
    """KUKA RSI UDP 服务器"""

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
        ("InPosition", "BOOL", 13),  # 机器人到位标记
    ]

    CSV_HEADER = [
        "timestamp", "iPOC", "in_position",
        "Fx_raw", "Fy_raw", "Fz_raw", "Mx_raw", "My_raw", "Mz_raw",
        "Act_X", "Act_Y", "Act_Z", "Act_A", "Act_B", "Act_C",
        "sensor_Fx_N", "sensor_Fy_N", "sensor_Fz_N", "sensor_Mx_Nm", "sensor_My_Nm", "sensor_Mz_Nm",
        "tcp_Fx_N", "tcp_Fy_N", "tcp_Fz_N", "tcp_Mx_Nm", "tcp_My_Nm", "tcp_Mz_Nm",
        "sample_status",
    ]

    def __init__(self, config: Optional[RSIConfig] = None, csv_filename: str = "rsi_data.csv"):
        self.config = config or RSIConfig()
        self.csv_filename = csv_filename
        self.sock: Optional[socket.socket] = None
        self.client_address: Optional[tuple] = None
        self.rsi_data_list: list[RSIData] = []
        self.csv_file = None
        self.csv_writer = None

        self.calibration_config: Optional[CalibrationConfig] = None
        self.calibration_runner: Optional[CalibrationRunner] = None

    def start(self):
        """启动 UDP 服务器"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = ("0.0.0.0", self.config.PORT)
        self.sock.bind(server_address)
        self.sock.settimeout(1.0)

        print("KUKA RSI UDP 服务器已启动")
        print(f"监听地址：{server_address[0]}:{server_address[1]}")
        print(f"预期发送方 IP: {self.config.IP_NUMBER}")
        print(f"CSV 保存文件：{self.csv_filename}")
        if self.calibration_config is not None:
            print(f"当前模式：{self.calibration_config.mode}")
            print(f"标定文件：{self.calibration_config.files.calibration_path}")
        print("\n按 Ctrl+C 停止服务器\n")

        self._init_csv()

    def _init_csv(self):
        """初始化 CSV 文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"rsi_data_{timestamp}.csv"
        self.csv_file = open(self.csv_filename, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_writer is None:
            raise RuntimeError("CSV writer initialization failed")
        self.csv_writer.writerow(self.CSV_HEADER)
        print(f"CSV 文件已创建：{self.csv_filename}")

    def parse_rsi_xml(self, xml_data: bytes) -> Optional[RSIData]:
        """解析 RSI XML 数据包"""
        try:
            xml_str = xml_data.decode("utf-8")
            root = ET.fromstring(xml_str)

            rsi_data = RSIData()
            rsi_data.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            ipoc_elem = root.find(".//IPOC")
            if ipoc_elem is not None:
                rsi_data.iPOC = int(ipoc_elem.text)

            for tag, elem_type, _ in self.SEND_ELEMENTS:
                elem = root.find(f".//{tag}")
                if elem is not None and elem.text:
                    value = elem.text.strip()
                    if elem_type == "LONG":
                        setattr(rsi_data, tag, int(value))
                    elif elem_type == "DOUBLE":
                        setattr(rsi_data, tag, float(value))
                    elif elem_type == "BOOL":
                        # KUKA RSI BOOL 类型：1 或 TRUE 表示 True，0 或 FALSE 表示 False
                        setattr(rsi_data, tag, value.upper() in ("1", "TRUE"))

            return rsi_data

        except ET.ParseError as error:
            print(f"XML 解析错误：{error}")
            return None
        except (ValueError, AttributeError) as error:
            print(f"数据解析错误：{error}")
            return None

    def generate_response(self, rsi_data: RSIData) -> str:
        """生成响应 XML"""
        if self.config.ONLYSEND:
            return ""

        response = ET.Element("Sen")
        response.set("Type", self.config.SENTYPE)
        elements = ET.SubElement(response, "Elements")
        for tag, elem_type, index in self.SEND_ELEMENTS:
            elem = ET.SubElement(elements, tag)
            elem.set("Type", elem_type)
            elem.set("Index", str(index))
            elem.text = str(getattr(rsi_data, tag))
        return ET.tostring(response, encoding="unicode")

    def process_frame(self, rsi_data: RSIData) -> RSIData:
        frame = {
            "timestamp": rsi_data.timestamp,
            "iPOC": rsi_data.iPOC,
            "in_position": rsi_data.in_position,
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
        sensor_wrench = processed.get("sensor_wrench", [0.0] * 6)
        calibrated_wrench: CalibratedWrench | None = processed.get("calibrated_wrench")

        rsi_data.sensor_fx, rsi_data.sensor_fy, rsi_data.sensor_fz = sensor_wrench[:3]
        rsi_data.sensor_mx, rsi_data.sensor_my, rsi_data.sensor_mz = sensor_wrench[3:6]
        rsi_data.sample_status = processed.get("sample_status", "streaming")

        if calibrated_wrench is not None:
            rsi_data.tcp_fx, rsi_data.tcp_fy, rsi_data.tcp_fz = calibrated_wrench.force_tcp
            rsi_data.tcp_mx, rsi_data.tcp_my, rsi_data.tcp_mz = calibrated_wrench.torque_tcp

        return rsi_data

    def save_to_csv(self, rsi_data: RSIData):
        """保存数据到 CSV"""
        if self.csv_writer:
            row = [
                rsi_data.timestamp,
                rsi_data.iPOC,
                1 if rsi_data.in_position else 0,
                rsi_data.Fx_raw, rsi_data.Fy_raw, rsi_data.Fz_raw,
                rsi_data.Mx_raw, rsi_data.My_raw, rsi_data.Mz_raw,
                rsi_data.Act_X, rsi_data.Act_Y, rsi_data.Act_Z,
                rsi_data.Act_A, rsi_data.Act_B, rsi_data.Act_C,
                rsi_data.sensor_fx, rsi_data.sensor_fy, rsi_data.sensor_fz,
                rsi_data.sensor_mx, rsi_data.sensor_my, rsi_data.sensor_mz,
                rsi_data.tcp_fx, rsi_data.tcp_fy, rsi_data.tcp_fz,
                rsi_data.tcp_mx, rsi_data.tcp_my, rsi_data.tcp_mz,
                rsi_data.sample_status,
            ]
            self.csv_writer.writerow(row)
            if self.csv_file is not None:
                self.csv_file.flush()

    def run(self):
        """运行服务器主循环"""
        try:
            self.start()
            if self.sock is None:
                raise RuntimeError("UDP socket initialization failed")
            packet_count = 0

            while True:
                try:
                    data, address = self.sock.recvfrom(4096)

                    if self.client_address is None:
                        self.client_address = address
                        print(f"\n收到来自 {address} 的连接")

                    rsi_data = self.parse_rsi_xml(data)

                    if rsi_data:
                        packet_count += 1
                        rsi_data = self.process_frame(rsi_data)

                        if packet_count % 100 == 1:
                            print(f"\n已接收 {packet_count} 个数据包")
                            print(f"  Fx_raw={rsi_data.Fx_raw}, Fy_raw={rsi_data.Fy_raw}, Fz_raw={rsi_data.Fz_raw}")
                            print(f"  Mx_raw={rsi_data.Mx_raw}, My_raw={rsi_data.My_raw}, Mz_raw={rsi_data.Mz_raw}")
                            print(f"  Act_X={rsi_data.Act_X:.3f}, Act_Y={rsi_data.Act_Y:.3f}, Act_Z={rsi_data.Act_Z:.3f}")
                            print(f"  Act_A={rsi_data.Act_A:.3f}, Act_B={rsi_data.Act_B:.3f}, Act_C={rsi_data.Act_C:.3f}")
                            print(f"  Sensor(N/Nm)=({rsi_data.sensor_fx:.3f}, {rsi_data.sensor_fy:.3f}, {rsi_data.sensor_fz:.3f}, {rsi_data.sensor_mx:.3f}, {rsi_data.sensor_my:.3f}, {rsi_data.sensor_mz:.3f})")
                            print(f"  TCP补偿后(N/Nm)=({rsi_data.tcp_fx:.3f}, {rsi_data.tcp_fy:.3f}, {rsi_data.tcp_fz:.3f}, {rsi_data.tcp_mx:.3f}, {rsi_data.tcp_my:.3f}, {rsi_data.tcp_mz:.3f})")
                            print(f"  状态={rsi_data.sample_status}")

                        self.save_to_csv(rsi_data)
                        self.rsi_data_list.append(rsi_data)

                        if not self.config.ONLYSEND:
                            response_xml = self.generate_response(rsi_data)
                            if response_xml:
                                self.sock.sendto(response_xml.encode("utf-8"), address)

                except socket.timeout:
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
        "--ip",
        type=str,
        default="192.168.2.10",
        help="RSI 数据源 IP 地址（默认：192.168.2.10）"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=59152,
        help="UDP 监听端口（默认：59152）"
    )
    return parser


def main():
    parser = create_parser()
    args = parser.parse_args()

    config = RSIConfig(
        IP_NUMBER=args.ip,
        PORT=args.port,
        SENTYPE="ImFree",
        ONLYSEND=True,
    )

    calibration_config = load_config()

    if args.calibrate:
        calibration_config.mode = "calibration_collect"
        print("=== 标定模式 ===")
        print(f"最少样本数：{calibration_config.static_detection.min_samples}")
        print(f"最多样本数：{calibration_config.static_detection.max_samples}")
        print("请将机器人运动到多个不同姿态，每个姿态停留约 1 秒")
        print("达到最少样本数后自动保存标定结果并切换到运行模式\n")
    elif args.run:
        calibration_config.mode = "calibrated_runtime"
        print("=== 运行模式 ===")
        print("使用已有标定结果进行实时重力补偿\n")
    else:
        calibration_config.mode = "record_only"
        print("=== 仅记录模式 ===")
        print("记录原始数据，不做标定或补偿\n")

    server = RSIServer(config=config)
    server.calibration_config = calibration_config
    server.calibration_runner = CalibrationRunner(calibration_config)

    calibration_result = load_calibration_result(calibration_config.files.calibration_path)
    if calibration_result is not None:
        server.calibration_runner.calibration_result = calibration_result
        if calibration_config.mode == "record_only":
            calibration_config.mode = "calibrated_runtime"
            print("检测到已有标定结果，自动启用补偿")

    server.run()


if __name__ == "__main__":
    main()
