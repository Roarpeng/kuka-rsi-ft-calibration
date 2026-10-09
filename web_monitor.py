"""Web 监控层：HTTP 状态查询、SSE 实时帧推送、事件流与 CSV 数据文件管理。

纯标准库实现（http.server.ThreadingHTTPServer）：只读监控 + 数据文件管理 +
力控参数在线设定（/api/force_config，白名单 + 范围校验，只写参数、不做控制）。
前端为 web_static/ 下的无构建单页（index.html + app.js + style.css + 本地化 Chart.js）。

对外入口：start_web_server(server, data_dir, data_cap_mb, port, host, force_config_path)。
"""

from __future__ import annotations

import collections
import json
import os
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional
from urllib.parse import parse_qs, unquote, urlparse

import calibration_io
import data_manager

# 前端静态文件目录（与本模块同级的 web_static/）
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_static")

# 事件环形缓冲容量
EVENT_BUFFER_SIZE = 500
# 帧环形缓冲容量（250Hz 下约 6s）
FRAME_BUFFER_SIZE = 1500
# 数据目录容量巡检周期（秒）
CAPACITY_CHECK_INTERVAL_S = 60.0
# SSE 推送周期（秒）
SSE_INTERVAL_S = 0.1
# read_series points 参数上限
SERIES_MAX_POINTS = 10000

# 力控参数在线设定白名单：键 -> (下限, 上限, 类型, 中文名)。
# 凿击横向让位 + 轴线零力矩对中（B←My、C←Mz）+ 默认目标力；方向符号只允许 ±1。
FORCE_CONFIG_FIELDS: dict[str, tuple[float, float, type, str]] = {
    "default_target_force_n": (0.0, 150.0, float, "默认目标力 (N)"),
    "chisel_lateral_deadband_n": (0.0, 100.0, float, "横向让位启动阈值 (N)"),
    "chisel_lateral_gain_mm_per_s_per_n": (0.001, 1.0, float, "横向让位增益 (mm/s/N)"),
    "chisel_lateral_max_mm": (0.5, 20.0, float, "横向让位行程上限 (mm)"),
    "chisel_lateral_trip_n": (20.0, 200.0, float, "横向卡滞保护阈值 (N)"),
    "chisel_lateral_sign": (-1.0, 1.0, int, "横向让位方向符号"),
    "align_chisel_enable": (False, True, bool, "凿击轴线对中开关"),
    "align_drill_enable": (False, True, bool, "钻孔轴线对中开关"),
    "align_deadband_nm": (0.1, 15.0, float, "对中力矩死区 (N·m)"),
    "align_gain_deg_per_s_per_nm": (0.01, 2.0, float, "对中增益 (°/s/N·m)"),
    "align_max_deg": (0.5, 5.0, float, "对中累计限幅 (°)"),
    "align_trip_nm": (2.0, 30.0, float, "对中力矩保护 (N·m)"),
    "align_sign": (-1.0, 1.0, int, "对中方向符号"),
    "tcp_filter_enable": (False, True, bool, "TCP 力入口滤波开关"),
    "tcp_filter_median_window": (1, 15, float, "入口滤波中值窗口(帧)"),
    "tcp_filter_lpf_hz": (1.0, 100.0, float, "入口滤波低通(Hz)"),
}

# 调试台固定输出：RKorr 每拍增量平移上限取现场 per_cycle_max_mm（与力控一致），
# 旋转上限 0.05°/拍；OV_PRO 0–100。会真实移动机械臂，仅供联调方向/链路验证。
DEBUG_ROT_MAX_DEG = 0.05
DEBUG_KEYS = ("x", "y", "z", "a", "b", "c")

# 工具系手型约定：三指轴分配必须是 XYZ 的排列；手性须与分配自洽
# （右手要求 拇指×食指 = 中指；左手为解剖镜像，要求 拇指×食指 = −中指）。
FRAME_AXES = ("X", "Y", "Z")
FRAME_FINGERS = (("thumb", "拇指"), ("index", "食指"), ("middle", "中指"))


def _cross_letter(a: str, b: str) -> tuple[str, int]:
    """右手系叉积（轴字母）：返回 (结果轴, 符号)。X×Y=Z，Y×Z=X，Z×X=Y。"""
    for p, q, r in (("X", "Y", "Z"), ("Y", "Z", "X"), ("Z", "X", "Y")):
        if (a, b) == (p, q):
            return r, 1
        if (a, b) == (q, p):
            return r, -1
    raise ValueError("叉积轴相同")


def _frame_derived(convention: Any, feed_axis: str = "X") -> dict:
    """由手型约定 + 钻轴（force_control.axis）推导方向参考表。

    KUKA 帧恒为右手系，叉积/旋转符号与手型无关：
    - 推尖端沿 +D 产生力矩 = feed × D；
    - 绕垂直轴 +r 的正旋转把进给端推向 r × feed 方向。
    """
    feed_axis = feed_axis.upper()
    axis_of_finger = {
        name: getattr(convention, name).upper()
        for name, _ in FRAME_FINGERS
    }
    finger_of_axis = {axis_of_finger[name]: cname for name, cname in FRAME_FINGERS}

    pushes = []
    for name, cname in FRAME_FINGERS:
        axis = axis_of_finger[name]
        if axis == feed_axis:
            continue  # 进给方向本身：推它只产生轴向力，无弯矩
        moment_axis, sign = _cross_letter(feed_axis, axis)
        pushes.append({
            "finger": cname,
            "axis": axis,
            "expect_force": f"F{axis.lower()} 读正",
            "expect_moment": f"M{moment_axis.lower()} 读{'正' if sign > 0 else '负'}",
        })

    def _finger_phrase(axis_letter: str, sign: int) -> str:
        finger = finger_of_axis.get(axis_letter, axis_letter)
        return f"{'+' if sign > 0 else '−'}{axis_letter}（{finger}{'方向' if sign > 0 else '反方向'}）"

    rotations = {}
    for r_axis in FRAME_AXES:
        if r_axis == feed_axis:
            continue
        tilt_axis, tilt_sign = _cross_letter(r_axis, feed_axis)
        rot_letter = {"X": "A", "Y": "B", "Z": "C"}[r_axis]
        rotations[rot_letter] = (
            f"{rot_letter}+ 绕 +{r_axis}：进给端向 {_finger_phrase(tilt_axis, tilt_sign)} 偏转"
        )

    return {
        "feed_finger": finger_of_axis.get(feed_axis, feed_axis),
        "pushes": pushes,
        "rotations": rotations,
    }

# 常见静态文件 Content-Type
_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def _frame_snapshot(rsi_data: Any, config: Any) -> dict:
    """把一帧 RSIData 转成 JSON 友好的快照（SSE 与状态查询共用字段）。"""
    rkorr = getattr(config, "rkorr", {}) if config is not None else {}
    return {
        "timestamp": rsi_data.timestamp,
        "iPOC": rsi_data.iPOC,
        "tcp": [
            rsi_data.tcp_fx, rsi_data.tcp_fy, rsi_data.tcp_fz,
            rsi_data.tcp_mx, rsi_data.tcp_my, rsi_data.tcp_mz,
        ],
        "act": [
            rsi_data.Act_X, rsi_data.Act_Y, rsi_data.Act_Z,
            rsi_data.Act_A, rsi_data.Act_B, rsi_data.Act_C,
        ],
        "sample_status": rsi_data.sample_status,
        "robot_status": bool(rsi_data.RobotStatus),
        "ov_pro": getattr(config, "ov_pro", None),
        "rkorr": [
            rkorr.get("RKorr.X", 0.0),
            rkorr.get("RKorr.Y", 0.0),
            rkorr.get("RKorr.Z", 0.0),
        ],
    }


class WebMonitor:
    """持有 RSIServer 引用、事件/帧缓冲与 HTTP 服务线程的监控对象。"""

    def __init__(
        self,
        server: Any,
        data_dir: str = "data",
        data_cap_mb: int = 2048,
        port: int = 8080,
        host: str = "0.0.0.0",
        force_config_path: str = "ft_calibration_config.json",
    ):
        self.server = server
        self.data_dir = data_dir
        self.data_cap_bytes = data_cap_mb * 1024 * 1024
        self.host = host
        self.requested_port = port
        self.force_config_path = force_config_path  # /api/force_config 持久化目标

        self.events: collections.deque[dict] = collections.deque(maxlen=EVENT_BUFFER_SIZE)
        self.frames: collections.deque[dict] = collections.deque(maxlen=FRAME_BUFFER_SIZE)
        self.latest_frame: Optional[dict] = None
        # 保护写操作（删除/锁定/解锁）与缓冲并发访问
        self.lock = threading.Lock()
        self._stop_event = threading.Event()

        # 挂载回调（已有回调时链式保留，不覆盖）
        self._hook_callbacks()

        # 绑定 HTTP 服务（端口占用在此抛清晰异常，由 main 侧 try/except 兜底）
        handler_cls = self._make_handler()
        try:
            self.httpd = ThreadingHTTPServer((host, port), handler_cls)
        except OSError as error:
            raise OSError(
                f"Web 监控无法绑定 {host}:{port}，端口可能被占用。原始错误: {error}"
            ) from error
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]

        self.emit_event("info", f"Web 监控已启动 http://{host}:{self.port}")

        self._http_thread = threading.Thread(
            target=self.httpd.serve_forever, name="web-monitor-http", daemon=True
        )
        self._capacity_thread = threading.Thread(
            target=self._capacity_loop, name="web-monitor-capacity", daemon=True
        )
        self._http_thread.start()
        self._capacity_thread.start()

    # ------------------------------------------------------------------
    # 回调挂载与缓冲
    # ------------------------------------------------------------------

    def _hook_callbacks(self) -> None:
        """把 server / force_controller / calibration_runner 的事件与帧回调接进来。"""
        server = self.server

        prev_on_frame = getattr(server, "on_frame", None)

        def on_frame(rsi_data: Any) -> None:
            if prev_on_frame is not None:
                prev_on_frame(rsi_data)
            self.push_frame(rsi_data)

        server.on_frame = on_frame

        prev_on_event = getattr(server, "on_event", None)

        def on_event(level: str, message: str) -> None:
            if prev_on_event is not None:
                prev_on_event(level, message)
            self.emit_event(level, message)

        server.on_event = on_event
        # 供运行时新建的力控器补挂同一事件回调（服务模式在线切换时用）
        self._event_hook = on_event

        # 力控/标定子模块可能为 None，挂载时判空
        force_controller = getattr(server, "force_controller", None)
        if force_controller is not None:
            force_controller.on_event = on_event
        calibration_runner = getattr(server, "calibration_runner", None)
        if calibration_runner is not None:
            calibration_runner.on_event = on_event

    def emit_event(self, level: str, message: str) -> None:
        """追加一条事件到环形缓冲。"""
        entry = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "level": level,
            "message": message,
        }
        with self.lock:
            self.events.append(entry)

    def push_frame(self, rsi_data: Any) -> None:
        """帧回调：生成快照写入帧缓冲与最新帧。"""
        snapshot = _frame_snapshot(rsi_data, getattr(self.server, "config", None))
        with self.lock:
            self.frames.append(snapshot)
            self.latest_frame = snapshot

    # ------------------------------------------------------------------
    # 容量巡检
    # ------------------------------------------------------------------

    def _active_csv_name(self) -> Optional[str]:
        """当前活动 CSV 的 basename（用于锁定活动文件，防止误删）。"""
        csv_filename = getattr(self.server, "csv_filename", None)
        return os.path.basename(csv_filename) if csv_filename else None

    def _capacity_loop(self) -> None:
        """每 60s 巡检数据目录容量，超限自动删最旧未锁定文件。"""
        while not self._stop_event.wait(CAPACITY_CHECK_INTERVAL_S):
            try:
                deleted = data_manager.enforce_capacity(
                    self.data_dir, self.data_cap_bytes, active_name=self._active_csv_name()
                )
            except Exception as error:  # 巡检失败不影响监控主流程
                self.emit_event("error", f"[容量] 数据目录巡检失败：{error}")
                continue
            if deleted:
                self.emit_event(
                    "warning",
                    f"[容量] 数据目录超限，已删除最旧文件：{', '.join(deleted)}",
                )

    # ------------------------------------------------------------------
    # 状态聚合
    # ------------------------------------------------------------------

    def build_status(self) -> dict:
        """聚合 /api/status 的全部字段。"""
        server = self.server
        calib_cfg = getattr(server, "calibration_config", None)
        runner = getattr(server, "calibration_runner", None)
        controller = getattr(server, "force_controller", None)
        config = getattr(server, "config", None)

        last_rx = getattr(server, "last_rx_monotonic", None)
        last_rx_age_s: Optional[float] = (
            round(time.monotonic() - last_rx, 3) if last_rx is not None else None
        )

        static_det = getattr(calib_cfg, "static_detection", None)
        samples = getattr(runner, "samples", []) if runner is not None else []
        calibration = {
            "samples": len(samples),
            "min_samples": getattr(static_det, "min_samples", None),
            "max_samples": getattr(static_det, "max_samples", None),
            "has_result": getattr(runner, "calibration_result", None) is not None,
        }

        if controller is not None:
            force: Optional[dict] = {
                "in_contact": bool(controller.in_contact),
                "tripped": bool(controller.tripped),
                "corr_cumulative_mm": list(controller.corr_cumulative_mm),
                "ov_pro_pct": controller.ov_pro_pct,
                "status_line": controller.status_line,
                "chisel_lateral_active": bool(getattr(controller, "chisel_lateral_active", False)),
                "align_active": bool(getattr(controller, "align_active", False)),
            }
        else:
            force = None

        with self.lock:
            latest = dict(self.latest_frame) if self.latest_frame is not None else None

        return {
            "mode": getattr(calib_cfg, "mode", None),
            "force_mode": bool(getattr(server, "force_mode", False)),
            "connection": {
                "last_rx_age_s": last_rx_age_s,
                "packet_count": getattr(server, "packet_count", 0),
                "rx_count": getattr(server, "rx_count", 0),
                "tx_count": getattr(server, "tx_count", 0),
                "parse_ok_count": getattr(server, "parse_ok_count", 0),
            },
            "calibration": calibration,
            "force": force,
            "tcp_wrench": latest["tcp"] if latest is not None else None,
            "ov_pro": getattr(config, "ov_pro", None),
            "rkorr": getattr(config, "rkorr", None),
            "active_csv": self._active_csv_name(),
            "data_dir": {
                "used_bytes": data_manager.total_size_bytes(self.data_dir),
                "cap_bytes": self.data_cap_bytes,
            },
        }

    # ------------------------------------------------------------------
    # HTTP 请求处理
    # ------------------------------------------------------------------

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        """生成绑定本 monitor 实例的请求处理类。"""
        monitor = self

        class WebRequestHandler(BaseHTTPRequestHandler):
            """监控台 HTTP 路由（全部 JSON 应答，除静态文件/下载/SSE）。"""

            server_version = "KukaRsiWebMonitor/1.0"

            def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
                """静默访问日志，避免刷屏控制台。"""

            # ---------------- 工具 ----------------

            def _send_json(self, payload: Any, status: int = 200) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_error_json(self, status: int, message: str) -> None:
                self._send_json({"error": message}, status=status)

            def _send_file(self, path: str, download_name: Optional[str] = None) -> None:
                try:
                    with open(path, "rb") as f:
                        body = f.read()
                except OSError:
                    self._send_error_json(HTTPStatus.NOT_FOUND, "文件不存在或不可读")
                    return
                ext = os.path.splitext(path)[1].lower()
                content_type = _CONTENT_TYPES.get(ext, "application/octet-stream")
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                if download_name is not None:
                    self.send_header(
                        "Content-Disposition", f'attachment; filename="{download_name}"'
                    )
                self.end_headers()
                self.wfile.write(body)

            def _resolve_static(self, rel_path: str) -> Optional[str]:
                """把 /static/ 后的相对路径解析到 web_static 内；越界返回 None。"""
                rel_path = unquote(rel_path).lstrip("/")
                base = os.path.realpath(STATIC_DIR)
                full = os.path.realpath(os.path.join(base, rel_path))
                if full != base and not full.startswith(base + os.sep):
                    return None
                return full

            def _valid_name_or_400(self, name: str) -> Optional[str]:
                """校验 CSV 文件名白名单；非法时回 400 并返回 None。"""
                name = unquote(name)
                if not data_manager.is_valid_csv_name(name):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, f"非法 CSV 文件名：{name!r}")
                    return None
                return name

            # ---------------- GET 路由 ----------------

            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                query = parse_qs(parsed.query)

                if path == "/":
                    self._send_file(os.path.join(STATIC_DIR, "index.html"))
                    return
                if path == "/debug.html" or path == "/debug":
                    self._send_file(os.path.join(STATIC_DIR, "debug.html"))
                    return
                if path.startswith("/static/"):
                    full = self._resolve_static(path[len("/static/"):])
                    if full is None or not os.path.isfile(full):
                        self._send_error_json(HTTPStatus.NOT_FOUND, "静态文件不存在")
                        return
                    self._send_file(full)
                    return
                if path == "/api/status":
                    self._send_json(monitor.build_status())
                    return
                if path == "/api/force_config":
                    self._handle_force_config_get()
                    return
                if path == "/api/debug_override":
                    self._handle_debug_get()
                    return
                if path == "/api/server_mode":
                    self._handle_server_mode_get()
                    return
                if path == "/api/frame_convention":
                    self._handle_frame_get()
                    return
                if path == "/api/stream":
                    self._handle_sse()
                    return
                if path == "/api/events":
                    self._handle_events(query)
                    return
                if path == "/api/files":
                    self._handle_files()
                    return
                if path.startswith("/api/files/"):
                    self._handle_file_get(path[len("/api/files/"):], query)
                    return
                self._send_error_json(HTTPStatus.NOT_FOUND, f"未知路径：{path}")

            # ---------------- POST 路由 ----------------

            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                if path == "/api/force_config":
                    self._handle_force_config_post()
                    return
                if path == "/api/debug_override":
                    self._handle_debug_post()
                    return
                if path == "/api/server_mode":
                    self._handle_server_mode_post()
                    return
                if path == "/api/frame_convention":
                    self._handle_frame_post()
                    return
                if path.startswith("/api/files/"):
                    rest = unquote(path[len("/api/files/"):])
                    for action in ("delete", "lock", "unlock"):
                        suffix = f"/{action}"
                        if rest.endswith(suffix):
                            name = rest[: -len(suffix)]
                            self._handle_file_action(name, action)
                            return
                self._send_error_json(HTTPStatus.NOT_FOUND, f"未知路径：{path}")

            # ---------------- 各端点实现 ----------------

            def _handle_sse(self) -> None:
                """SSE：每 100ms 推一条最新帧快照；无帧时推心跳注释行。"""
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    while not monitor._stop_event.is_set():
                        with monitor.lock:
                            frame = (
                                dict(monitor.latest_frame)
                                if monitor.latest_frame is not None
                                else None
                            )
                        if frame is not None:
                            payload = json.dumps(frame, ensure_ascii=False)
                            self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        else:
                            self.wfile.write(b": heartbeat\n\n")
                        self.wfile.flush()
                        time.sleep(SSE_INTERVAL_S)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass  # 客户端断开属正常

            def _handle_events(self, query: dict) -> None:
                try:
                    limit = int(query.get("limit", ["100"])[0])
                except ValueError:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "limit 必须为整数")
                    return
                limit = max(1, min(limit, EVENT_BUFFER_SIZE))
                with monitor.lock:
                    events = list(monitor.events)[-limit:]
                self._send_json({"events": events})

            def _handle_files(self) -> None:
                entries = data_manager.list_files(
                    monitor.data_dir, active_name=monitor._active_csv_name()
                )
                for entry in entries:
                    entry["size_mb"] = round(entry["size_bytes"] / 1024 / 1024, 3)
                self._send_json({"files": entries})

            def _handle_file_get(self, rest: str, query: dict) -> None:
                rest = unquote(rest)
                if rest.endswith("/series"):
                    name = rest[: -len("/series")]
                    self._handle_series(name, query)
                    return
                name = self._valid_name_or_400(rest)
                if name is None:
                    return
                path = os.path.join(monitor.data_dir, name)
                if not os.path.isfile(path):
                    self._send_error_json(HTTPStatus.NOT_FOUND, f"文件不存在：{name}")
                    return
                self._send_file(path, download_name=name)

            def _handle_series(self, name: str, query: dict) -> None:
                name = self._valid_name_or_400(name)
                if name is None:
                    return
                start = query.get("start", [None])[0] or None
                end = query.get("end", [None])[0] or None
                try:
                    points = int(query.get("points", ["2000"])[0])
                except ValueError:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "points 必须为整数")
                    return
                points = max(1, min(points, SERIES_MAX_POINTS))
                path = os.path.join(monitor.data_dir, name)
                if not os.path.isfile(path):
                    self._send_error_json(HTTPStatus.NOT_FOUND, f"文件不存在：{name}")
                    return
                try:
                    result = data_manager.read_series(
                        path, start=start, end=end, max_points=points
                    )
                except ValueError as error:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, str(error))
                    return
                self._send_json(result)

            def _live_force_config(self) -> Optional[Any]:
                """server 上当前生效的 ForceControlConfig（与力控器共享同一对象）。"""
                calib_cfg = getattr(monitor.server, "calibration_config", None)
                return getattr(calib_cfg, "force_control", None)

            def _force_config_payload(self, fc: Any) -> dict:
                return {"force_control": {key: getattr(fc, key) for key in FORCE_CONFIG_FIELDS}}

            def _handle_force_config_get(self) -> None:
                fc = self._live_force_config()
                if fc is None:
                    self._send_error_json(HTTPStatus.CONFLICT, "力控配置不可用（服务未加载 calibration_config）")
                    return
                self._send_json(self._force_config_payload(fc))

            def _handle_force_config_post(self) -> None:
                # 先把请求体读完再应答：未读数据会导致关闭时 RST（Windows 10053），
                # 客户端可能收不到已写入的响应
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    length = 0
                body = b""
                if length > 0:
                    if length > 4096:
                        remaining = length
                        while remaining > 0:  # 丢弃超大请求体
                            chunk = self.rfile.read(min(remaining, 4096))
                            if not chunk:
                                break
                            remaining -= len(chunk)
                        self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体过大")
                        return
                    body = self.rfile.read(length)
                fc = self._live_force_config()
                if fc is None:
                    self._send_error_json(HTTPStatus.CONFLICT, "力控配置不可用（服务未加载 calibration_config）")
                    return
                try:
                    updates = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "JSON 解析失败")
                    return
                if not isinstance(updates, dict) or not updates:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体须为非空 JSON 对象")
                    return
                unknown = [k for k in updates if k not in FORCE_CONFIG_FIELDS]
                if unknown:
                    self._send_error_json(
                        HTTPStatus.BAD_REQUEST, f"不在白名单的参数：{', '.join(unknown)}"
                    )
                    return
                cleaned: dict[str, float | int | bool] = {}
                for key, value in updates.items():
                    low, high, cast, _ = FORCE_CONFIG_FIELDS[key]
                    if cast is bool:
                        # 布尔开关：只接受真布尔（数值 0/1 不收，避免语义歧义）
                        if not isinstance(value, bool):
                            self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为布尔值")
                            return
                        cleaned[key] = value
                        continue
                    if isinstance(value, bool):
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为数值")
                        return
                    try:
                        typed = cast(value)
                    except (TypeError, ValueError):
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为数值")
                        return
                    if key in ("chisel_lateral_sign", "align_sign") and typed == 0:
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 只允许 1 或 -1")
                        return
                    if not low <= typed <= high:
                        self._send_error_json(
                            HTTPStatus.BAD_REQUEST, f"{key}={value} 超出范围 [{low}, {high}]"
                        )
                        return
                    cleaned[key] = typed
                with monitor.lock:
                    for key, value in cleaned.items():
                        setattr(fc, key, value)
                    try:
                        calibration_io.save_force_control_updates(
                            monitor.force_config_path, cleaned
                        )
                    except (OSError, ValueError) as error:
                        self._send_error_json(
                            HTTPStatus.INTERNAL_SERVER_ERROR, f"配置落盘失败：{error}"
                        )
                        return
                monitor.emit_event(
                    "info", f"[Web] 已更新力控参数：{json.dumps(cleaned, ensure_ascii=False)}"
                )
                self._send_json({"ok": True, **self._force_config_payload(fc)})

            def _debug_state(self) -> dict:
                current = getattr(monitor.server, "debug_override", None)
                if isinstance(current, dict):
                    return {
                        "enabled": bool(current.get("enabled", False)),
                        **{k: float(current.get(k, 0.0)) for k in DEBUG_KEYS},
                        "ov_pro": float(current.get("ov_pro", 100.0)),
                    }
                return {"enabled": False, **{k: 0.0 for k in DEBUG_KEYS}, "ov_pro": 100.0}

            def _debug_limits(self) -> dict:
                calib = getattr(monitor.server, "calibration_config", None)
                fc = getattr(calib, "force_control", None)
                return {
                    "trans_max": float(getattr(fc, "per_cycle_max_mm", 0.08) if fc is not None else 0.08),
                    "rot_max": DEBUG_ROT_MAX_DEG,
                    "cycle_s": float(getattr(fc, "cycle_s", 0.004) if fc is not None else 0.004),
                }

            def _handle_debug_get(self) -> None:
                self._send_json({"debug": self._debug_state(), "limits": self._debug_limits()})

            def _handle_debug_post(self) -> None:
                # 先读完请求体再应答（同 force_config，防未读数据触发 Windows RST）
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    length = 0
                body = b""
                if length > 0:
                    if length > 4096:
                        self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体过大")
                        return
                    body = self.rfile.read(length)
                try:
                    payload = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "JSON 解析失败")
                    return
                if not isinstance(payload, dict) or not payload:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体须为非空 JSON 对象")
                    return
                allowed = {"enabled", *DEBUG_KEYS, "ov_pro"}
                unknown = [k for k in payload if k not in allowed]
                if unknown:
                    self._send_error_json(
                        HTTPStatus.BAD_REQUEST, f"不在白名单的参数：{', '.join(unknown)}"
                    )
                    return
                limits = self._debug_limits()
                state = self._debug_state()
                for key, value in payload.items():
                    if key == "enabled":
                        if not isinstance(value, bool):
                            self._send_error_json(HTTPStatus.BAD_REQUEST, "enabled 须为布尔值")
                            return
                        state["enabled"] = value
                        continue
                    if isinstance(value, bool):
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为数值")
                        return
                    try:
                        typed = float(value)
                    except (TypeError, ValueError):
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为数值")
                        return
                    if key == "ov_pro":
                        if not 0.0 <= typed <= 100.0:
                            self._send_error_json(HTTPStatus.BAD_REQUEST, f"ov_pro={value} 超出范围 [0, 100]")
                            return
                    else:
                        bound = limits["trans_max"] if key in "xyz" else limits["rot_max"]
                        if not -bound <= typed <= bound:
                            self._send_error_json(
                                HTTPStatus.BAD_REQUEST,
                                f"{key}={value} 超出每拍限幅 ±{bound}（RSI 安全约束）",
                            )
                            return
                    state[key] = typed
                if state["enabled"] is False and "enabled" in payload:
                    state = {"enabled": False, **{k: 0.0 for k in DEBUG_KEYS}, "ov_pro": 100.0}
                    disabler = getattr(monitor.server, "disable_debug_override", None)
                    if callable(disabler):
                        # 服务端立即清 config.rkorr/OV_PRO 并复位力控器
                        disabler("Web 调试台停止")
                    else:
                        monitor.server.debug_override = dict(state)
                else:
                    monitor.server.debug_override = dict(state)
                    rk = ", ".join(f"{k.upper()}={state[k]:+.4f}" for k in DEBUG_KEYS)
                    level = "warning" if state["enabled"] else "info"
                    action = "已启用" if state["enabled"] else "已更新"
                    monitor.emit_event(
                        level, f"[Web] 调试固定输出{action}：{rk}，OV_PRO={state['ov_pro']:.0f}%"
                    )
                self._send_json({"ok": True, "debug": self._debug_state(), "limits": limits})

            def _handle_server_mode_get(self) -> None:
                mode = "force" if getattr(monitor.server, "force_mode", False) else "monitor"
                runner = getattr(monitor.server, "calibration_runner", None)
                self._send_json({
                    "mode": mode,
                    "has_calibration": getattr(runner, "calibration_result", None) is not None,
                })

            def _handle_server_mode_post(self) -> None:
                # 先读完请求体再应答（同上，防未读数据触发 Windows RST）
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    length = 0
                body = b""
                if length > 0:
                    if length > 4096:
                        self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体过大")
                        return
                    body = self.rfile.read(length)
                try:
                    payload = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "JSON 解析失败")
                    return
                if not isinstance(payload, dict) or set(payload) != {"mode"}:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体须为 {\"mode\": \"monitor\"|\"force\"}")
                    return
                if payload["mode"] not in ("monitor", "force"):
                    self._send_error_json(
                        HTTPStatus.BAD_REQUEST, f"未知模式 {payload['mode']!r}（可选 monitor/force）"
                    )
                    return
                setter = getattr(monitor.server, "set_service_mode", None)
                if not callable(setter):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "服务不支持在线切换模式")
                    return
                try:
                    setter(payload["mode"])
                except ValueError as error:
                    status = HTTPStatus.CONFLICT if "标定" in str(error) else HTTPStatus.BAD_REQUEST
                    self._send_error_json(status, str(error))
                    return
                # 运行时新建的力控器补挂事件回调（进入事件流/Web 页可见）
                controller = getattr(monitor.server, "force_controller", None)
                hook = getattr(monitor, "_event_hook", None)
                if controller is not None and hook is not None:
                    controller.on_event = hook
                self._send_json({
                    "ok": True,
                    "mode": "force" if getattr(monitor.server, "force_mode", False) else "monitor",
                })

            def _live_convention(self) -> Any:
                calib_cfg = getattr(monitor.server, "calibration_config", None)
                return getattr(calib_cfg, "frame_convention", None)

            def _frame_payload(self) -> dict:
                convention = self._live_convention()
                calib_cfg = getattr(monitor.server, "calibration_config", None)
                fc = getattr(calib_cfg, "force_control", None)
                feed_axis = str(getattr(fc, "axis", "X") if fc is not None else "X")
                return {
                    "convention": {
                        "hand": convention.hand,
                        "thumb": convention.thumb,
                        "index": convention.index,
                        "middle": convention.middle,
                    },
                    "feed_axis": feed_axis.upper(),
                    "derived": _frame_derived(convention, feed_axis),
                }

            def _handle_frame_get(self) -> None:
                if self._live_convention() is None:
                    self._send_error_json(HTTPStatus.CONFLICT, "配置不可用（无 calibration_config）")
                    return
                self._send_json(self._frame_payload())

            def _handle_frame_post(self) -> None:
                calib_cfg = getattr(monitor.server, "calibration_config", None)
                if calib_cfg is None or self._live_convention() is None:
                    self._send_error_json(HTTPStatus.CONFLICT, "配置不可用（无 calibration_config）")
                    return
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    length = 0
                body = b""
                if length > 0:
                    if length > 4096:
                        self._send_error_json(HTTPStatus.BAD_REQUEST, "请求体过大")
                        return
                    body = self.rfile.read(length)
                try:
                    payload = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "JSON 解析失败")
                    return
                if not isinstance(payload, dict) or set(payload) != {"hand", "thumb", "index", "middle"}:
                    self._send_error_json(
                        HTTPStatus.BAD_REQUEST,
                        "请求体须为 {hand, thumb, index, middle}",
                    )
                    return
                hand = str(payload["hand"]).lower()
                if hand not in ("left", "right"):
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "hand 须为 left 或 right")
                    return
                axes = {}
                for name, cname in FRAME_FINGERS:
                    axis = str(payload[name]).upper()
                    if axis not in FRAME_AXES:
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{cname} 须为 X/Y/Z")
                        return
                    axes[name] = axis
                if len(set(axes.values())) != 3:
                    self._send_error_json(HTTPStatus.BAD_REQUEST, "三根手指的轴分配必须是 X/Y/Z 的排列（不得重复）")
                    return
                # 手性自洽：右手要求 拇指×食指 = 中指；左手为镜像（= −中指）
                cross_axis, cross_sign = _cross_letter(axes["thumb"], axes["index"])
                fits_right = cross_axis == axes["middle"] and cross_sign > 0
                fits_left = cross_axis == axes["middle"] and cross_sign < 0
                if (hand == "right" and not fits_right) or (hand == "left" and not fits_left):
                    proper_hand = "右手" if fits_right else "左手"
                    self._send_error_json(
                        HTTPStatus.BAD_REQUEST,
                        f"手性不自洽：此分配下 拇指×食指 = {'+' if cross_sign > 0 else '−'}中指，"
                        f"对应{proper_hand}手势，但 hand 选的是{('右' if hand == 'right' else '左')}手；"
                        f"请把 hand 改为{proper_hand}或更换手指分配",
                    )
                    return
                from calibration_models import FrameConvention
                calib_cfg.frame_convention = FrameConvention(
                    hand=hand, thumb=axes["thumb"], index=axes["index"], middle=axes["middle"]
                )
                try:
                    calibration_io.save_config_section(
                        monitor.force_config_path, "frame_convention",
                        {"hand": hand, **axes},
                    )
                except (OSError, ValueError) as error:
                    self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, f"配置落盘失败：{error}")
                    return
                monitor.emit_event(
                    "info",
                    f"[Web] 工具系手型约定已更新：{('右' if hand == 'right' else '左')}手，"
                    f"拇指={axes['thumb']} 食指={axes['index']} 中指={axes['middle']}",
                )
                self._send_json({"ok": True, **self._frame_payload()})

            def _handle_file_action(self, name: str, action: str) -> None:
                name = self._valid_name_or_400(name)
                if name is None:
                    return
                with monitor.lock:
                    try:
                        if action == "delete":
                            data_manager.delete_file(
                                monitor.data_dir, name,
                                active_name=monitor._active_csv_name(),
                            )
                        elif action == "lock":
                            data_manager.lock_file(monitor.data_dir, name)
                        else:  # unlock
                            data_manager.unlock_file(monitor.data_dir, name)
                    except ValueError as error:
                        message = str(error)
                        # 删除活动文件属冲突，返回 409；其余参数类错误 400
                        status = (
                            HTTPStatus.CONFLICT
                            if action == "delete" and "活动文件" in message
                            else HTTPStatus.BAD_REQUEST
                        )
                        self._send_error_json(status, message)
                        return
                label = {"delete": "删除", "lock": "锁定", "unlock": "解锁"}[action]
                monitor.emit_event("info", f"[文件] 已{label} {name}")
                self._send_json({"ok": True, "action": action, "name": name})

        return WebRequestHandler

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """停止 HTTP 服务与容量巡检线程（主要用于测试）。"""
        self._stop_event.set()
        self.httpd.shutdown()
        self.httpd.server_close()


def start_web_server(
    server: Any,
    data_dir: str = "data",
    data_cap_mb: int = 2048,
    port: int = 8080,
    host: str = "0.0.0.0",
    force_config_path: str = "ft_calibration_config.json",
) -> WebMonitor:
    """创建并启动 Web 监控服务，返回 WebMonitor 实例。"""
    return WebMonitor(
        server,
        data_dir=data_dir,
        data_cap_mb=data_cap_mb,
        port=port,
        host=host,
        force_config_path=force_config_path,
    )
