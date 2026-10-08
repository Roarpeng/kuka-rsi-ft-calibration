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
# 只开放力控闭环参数（凿击横向让位 + 默认目标力）；方向符号只允许 ±1。
FORCE_CONFIG_FIELDS: dict[str, tuple[float, float, type, str]] = {
    "default_target_force_n": (0.0, 150.0, float, "默认目标力 (N)"),
    "chisel_lateral_deadband_n": (0.0, 100.0, float, "横向让位启动阈值 (N)"),
    "chisel_lateral_gain_mm_per_s_per_n": (0.001, 1.0, float, "横向让位增益 (mm/s/N)"),
    "chisel_lateral_max_mm": (0.5, 20.0, float, "横向让位行程上限 (mm)"),
    "chisel_lateral_trip_n": (20.0, 200.0, float, "横向卡滞保护阈值 (N)"),
    "chisel_lateral_sign": (-1.0, 1.0, int, "横向让位方向符号"),
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
                cleaned: dict[str, float | int] = {}
                for key, value in updates.items():
                    if isinstance(value, bool):
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为数值")
                        return
                    low, high, cast, _ = FORCE_CONFIG_FIELDS[key]
                    try:
                        typed = cast(value)
                    except (TypeError, ValueError):
                        self._send_error_json(HTTPStatus.BAD_REQUEST, f"{key} 须为数值")
                        return
                    if key == "chisel_lateral_sign" and typed == 0:
                        self._send_error_json(HTTPStatus.BAD_REQUEST, "chisel_lateral_sign 只允许 1 或 -1")
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
