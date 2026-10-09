# Web 监控层测试：Part 1 data_manager 单元测试 + Part 2 web_monitor 集成测试。
# 直接运行：python test_web_monitor.py（断言全过、退出码 0 为通过）。
from __future__ import annotations

import collections
import csv
import http.client
import json
import os
import tempfile
from types import SimpleNamespace

import data_manager
import web_monitor
from calibration_models import ForceControlConfig
from udp_server import RSIData

# ============================================================
# Part 1：data_manager 单元测试
# ============================================================

N_A = "rsi_data_20260901_100000.csv"
N_B = "rsi_data_20260901_110000.csv"
N_C = "rsi_data_20260901_120000.csv"
N_D = "rsi_data_20260901_130000.csv"


def _make_file(dirpath: str, name: str, size: int = 100, mtime: float | None = None) -> str:
    """在 dirpath 下造一个指定大小/修改时间的假 csv。"""
    path = os.path.join(dirpath, name)
    with open(path, "wb") as f:
        f.write(b"x" * size)
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


def _write_series_csv(path: str, rows: int, start_second: int = 0) -> None:
    """写一个 timestamp/Fx/sample_status 三列的 csv，时间戳逐秒递增。"""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "Fx", "sample_status"])
        for i in range(rows):
            sec = start_second + i
            ts = f"2026-09-01 16:{46 + sec // 60:02d}:{sec % 60:02d}.000"
            w.writerow([ts, f"{i * 0.5}", "streaming"])


def test_whitelist() -> None:
    assert data_manager.is_valid_csv_name("rsi_data_20260901_164623.csv")
    assert not data_manager.is_valid_csv_name("../evil")
    assert not data_manager.is_valid_csv_name("evil.csv")
    assert not data_manager.is_valid_csv_name("rsi_data_2026090_164623.csv")  # 日期 7 位
    assert not data_manager.is_valid_csv_name("rsi_data_20260901_164623.csv.lock")
    print("白名单校验 OK")


def test_list_files() -> None:
    with tempfile.TemporaryDirectory() as d:
        assert data_manager.list_files(os.path.join(d, "不存在")) == []
        base = 1_700_000_000.0
        _make_file(d, N_A, mtime=base)          # 最旧
        _make_file(d, N_B, mtime=base + 10)
        _make_file(d, N_C, mtime=base + 20)     # 最新
        _make_file(d, "notes.txt")              # 非白名单，应被排除
        data_manager.lock_file(d, N_B)
        # active_name 允许是完整路径（取 basename 比较）
        entries = data_manager.list_files(d, active_name=os.path.join("data", N_C))
        assert [e["name"] for e in entries] == [N_C, N_B, N_A]  # mtime 倒序
        by_name = {e["name"]: e for e in entries}
        assert by_name[N_C]["active"] is True and by_name[N_C]["locked"] is False
        assert by_name[N_B]["locked"] is True and by_name[N_B]["active"] is False
        assert by_name[N_A]["size_bytes"] == 100 and by_name[N_A]["mtime"] == base
    print("list_files OK")


def test_lock_unlock_delete() -> None:
    with tempfile.TemporaryDirectory() as d:
        _make_file(d, N_A)
        # lock/unlock 正常路径
        data_manager.lock_file(d, N_A)
        sidecar = os.path.join(d, N_A + ".lock")
        assert os.path.exists(sidecar)
        data_manager.unlock_file(d, N_A)
        assert not os.path.exists(sidecar)
        data_manager.unlock_file(d, N_A)  # sidecar 不存在视为成功
        # 非法名 ValueError
        for bad in ("../evil", "evil.csv"):
            for fn in (data_manager.lock_file, data_manager.unlock_file):
                try:
                    fn(d, bad)
                    raise AssertionError(f"{fn.__name__}({bad!r}) 应抛 ValueError")
                except ValueError:
                    pass
        # 删除正常路径：csv 与 sidecar 一起清掉
        data_manager.lock_file(d, N_A)
        data_manager.delete_file(d, N_A)
        assert not os.path.exists(os.path.join(d, N_A))
        assert not os.path.exists(sidecar)
        # 删活动文件 ValueError（active_name 给完整路径，验证 basename 比较）
        _make_file(d, N_B)
        try:
            data_manager.delete_file(d, N_B, active_name=os.path.join("data", N_B))
            raise AssertionError("删活动文件应抛 ValueError")
        except ValueError:
            pass
        assert os.path.exists(os.path.join(d, N_B))
        # 删不存在 / 非法名 ValueError
        try:
            data_manager.delete_file(d, N_C)
            raise AssertionError("删不存在文件应抛 ValueError")
        except ValueError:
            pass
        try:
            data_manager.delete_file(d, "../evil")
            raise AssertionError("删非法名应抛 ValueError")
        except ValueError:
            pass
    print("lock/unlock/delete OK")


def test_enforce_capacity() -> None:
    with tempfile.TemporaryDirectory() as d:
        base = 1_700_000_000.0
        for i, name in enumerate((N_A, N_B, N_C, N_D)):  # A 最旧 … D 最新，各 1000B
            _make_file(d, name, size=1000, mtime=base + i)
        # 未超容量：不动，返回空
        assert data_manager.enforce_capacity(d, 10 ** 9) == []
        assert data_manager.total_size_bytes(d) == 4000
        # cap=1500：从最旧删起；B 锁定、C 活动跳过；D 删掉后仍超但无可删
        data_manager.lock_file(d, N_B)
        deleted = data_manager.enforce_capacity(d, 1500, active_name=os.path.join("x", N_C))
        assert deleted == [N_A, N_D]
        remaining = {e["name"] for e in data_manager.list_files(d)}
        assert remaining == {N_B, N_C}
        assert os.path.exists(os.path.join(d, N_B + ".lock"))  # 锁定文件的 sidecar 保留
        # 只剩锁定/活动文件仍超额：不再删，返回空
        assert data_manager.enforce_capacity(d, 1, active_name=N_C) == []
        assert data_manager.enforce_capacity(os.path.join(d, "不存在"), 1) == []
    print("enforce_capacity OK")


def test_read_series() -> None:
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, N_A)
        _write_series_csv(path, 100)  # 16:46:00 ~ 16:47:39，每秒一行

        # 全量读取
        result = data_manager.read_series(path)
        assert result["columns"] == ["timestamp", "Fx", "sample_status"]
        assert len(result["timestamps"]) == 100
        assert "timestamp" not in result["series"]
        assert isinstance(result["series"]["Fx"][0], float)
        assert result["series"]["Fx"][3] == 1.5
        assert isinstance(result["series"]["sample_status"][0], str)
        assert result["series"]["sample_status"][0] == "streaming"

        # start/end 过滤（含边界）：16:46:10 ~ 16:46:15 共 6 行
        result = data_manager.read_series(
            path, start="2026-09-01 16:46:10", end="2026-09-01 16:46:15"
        )
        assert len(result["timestamps"]) == 6
        assert result["timestamps"][0] == "2026-09-01 16:46:10.000"
        assert result["timestamps"][-1] == "2026-09-01 16:46:15.000"
        assert len(result["series"]["Fx"]) == 6

        # 抽稀：100 行 max_points=10 → 步长 10 选 10 行 + 补末行 = 11 ≤ max_points+1
        result = data_manager.read_series(path, max_points=10)
        assert len(result["timestamps"]) <= 10 + 1
        assert result["timestamps"][0] == "2026-09-01 16:46:00.000"  # 首行保留
        assert result["timestamps"][-1] == "2026-09-01 16:47:39.000"  # 末行保留

        # 非法参数 ValueError
        try:
            data_manager.read_series(path, start="2026-09-01 16:47:00", end="2026-09-01 16:46:00")
            raise AssertionError("start 晚于 end 应抛 ValueError")
        except ValueError:
            pass
        try:
            data_manager.read_series(path, start="16:46")
            raise AssertionError("非法 start 格式应抛 ValueError")
        except ValueError:
            pass

        # 空文件（无表头）返回空结构
        empty = os.path.join(d, N_B)
        with open(empty, "w", encoding="utf-8"):
            pass
        result = data_manager.read_series(empty)
        assert result == {"columns": [], "timestamps": [], "series": {}}
    print("read_series OK")


# ============================================================
# Part 2：web_monitor 集成测试
# ============================================================

ACTIVE_NAME = "rsi_data_20260901_164623.csv"
OLD_NAME = "rsi_data_20260801_120000.csv"


def _make_dummy_server(data_dir: str) -> SimpleNamespace:
    """按 web_monitor.py 实际读取的属性构造 dummy RSIServer。"""
    return SimpleNamespace(
        calibration_config=SimpleNamespace(
            mode="record_only",
            static_detection=SimpleNamespace(min_samples=16, max_samples=32),
        ),
        force_mode=False,
        calibration_runner=None,
        force_controller=None,
        config=SimpleNamespace(
            ov_pro=100.0,
            rkorr={"RKorr.X": 0.0, "RKorr.Y": 0.0, "RKorr.Z": 0.0,
                   "RKorr.A": 0.0, "RKorr.B": 0.0, "RKorr.C": 0.0},
        ),
        csv_filename=os.path.join(data_dir, ACTIVE_NAME),
        rx_count=0, tx_count=0, parse_ok_count=0, packet_count=0,
        last_rx_monotonic=None,
        rsi_data_list=collections.deque(maxlen=10),
        on_frame=None,
        on_event=None,
    )


def _request(
    port: int, method: str, path: str, body: bytes | None = None
) -> tuple[int, str, bytes]:
    """发一次 HTTP 请求，返回 (状态码, Content-Type, body)。"""
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        headers = {"Content-Type": "application/json"} if body is not None else {}
        conn.request(method, path, body=body, headers=headers)
        resp = conn.getresponse()
        return resp.status, resp.getheader("Content-Type") or "", resp.read()
    finally:
        conn.close()


def _get_json(port: int, path: str) -> tuple[int, dict]:
    status, _, body = _request(port, "GET", path)
    return status, json.loads(body.decode("utf-8"))


def test_web_monitor() -> None:
    with tempfile.TemporaryDirectory() as d:
        _write_series_csv(os.path.join(d, ACTIVE_NAME), 5)
        _write_series_csv(os.path.join(d, OLD_NAME), 5)
        dummy = _make_dummy_server(d)

        monitor = web_monitor.start_web_server(dummy, data_dir=d, port=0, host="127.0.0.1")
        port = monitor.port
        assert port > 0
        try:
            # 回调已挂载到 dummy server
            assert callable(dummy.on_frame) and callable(dummy.on_event)

            # ---- 静态文件 ----
            status, ctype, _ = _request(port, "GET", "/")
            assert status == 200 and "text/html" in ctype
            status, ctype, _ = _request(port, "GET", "/static/app.js")
            assert status == 200 and "javascript" in ctype
            # 路径穿越必须被拦截（非 200）
            status, _, _ = _request(port, "GET", "/static/../web_monitor.py")
            assert status != 200

            # ---- /api/status ----
            status, payload = _get_json(port, "/api/status")
            assert status == 200
            for key in ("mode", "force_mode", "connection", "calibration",
                        "force", "active_csv", "data_dir", "ov_pro"):
                assert key in payload, f"/api/status 缺少字段 {key}"
            assert payload["mode"] == "record_only"
            assert payload["active_csv"] == ACTIVE_NAME
            for key in ("rx_count", "tx_count", "parse_ok_count",
                        "packet_count", "last_rx_age_s"):
                assert key in payload["connection"], f"connection 缺少字段 {key}"
            assert payload["calibration"]["min_samples"] == 16
            assert payload["data_dir"]["used_bytes"] > 0
            assert payload["data_dir"]["cap_bytes"] == 2048 * 1024 * 1024

            # ---- /api/events（含启动事件；limit 生效）----
            status, payload = _get_json(port, "/api/events")
            assert status == 200
            assert any("Web 监控" in e["message"] for e in payload["events"])
            status, payload = _get_json(port, "/api/events?limit=1")
            assert status == 200 and len(payload["events"]) == 1

            # ---- /api/files ----
            status, payload = _get_json(port, "/api/files")
            assert status == 200
            by_name = {f["name"]: f for f in payload["files"]}
            assert set(by_name) == {ACTIVE_NAME, OLD_NAME}
            assert by_name[ACTIVE_NAME]["active"] is True
            assert by_name[OLD_NAME]["active"] is False
            assert "size_mb" in by_name[OLD_NAME]

            # ---- series 回放 ----
            status, payload = _get_json(port, f"/api/files/{OLD_NAME}/series")
            assert status == 200
            assert set(payload) == {"columns", "timestamps", "series"}
            assert len(payload["timestamps"]) == 5
            assert "timestamp" not in payload["series"]

            # ---- lock/unlock ----
            sidecar = os.path.join(d, OLD_NAME + ".lock")
            status, _, _ = _request(port, "POST", f"/api/files/{OLD_NAME}/lock")
            assert status == 200 and os.path.exists(sidecar)
            status, payload = _get_json(port, "/api/files")
            assert {f["name"]: f for f in payload["files"]}[OLD_NAME]["locked"] is True
            status, _, _ = _request(port, "POST", f"/api/files/{OLD_NAME}/unlock")
            assert status == 200 and not os.path.exists(sidecar)

            # ---- 非法文件名 → 400 ----
            status, _, _ = _request(port, "POST", "/api/files/evil.csv/delete")
            assert status == 400
            status, _ = _get_json(port, "/api/files/not_a_csv/series")
            assert status == 400

            # ---- 删除：活动文件 409；不存在 400；非活动 200 且文件消失 ----
            status, _, _ = _request(port, "POST", f"/api/files/{ACTIVE_NAME}/delete")
            assert status == 409
            assert os.path.exists(os.path.join(d, ACTIVE_NAME))
            status, _, _ = _request(port, "POST", f"/api/files/{N_A}/delete")
            assert status == 400  # 有效名但文件不存在
            status, _, _ = _request(port, "POST", f"/api/files/{OLD_NAME}/delete")
            assert status == 200
            assert not os.path.exists(os.path.join(d, OLD_NAME))

            # ---- SSE：能收到数据行（帧快照或心跳）----
            dummy.on_frame(RSIData(timestamp="2026-09-01 16:46:23.000", iPOC=1))
            conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            lines: list[str] = []
            try:
                conn.request("GET", "/api/stream")
                resp = conn.getresponse()
                assert resp.status == 200
                assert "text/event-stream" in (resp.getheader("Content-Type") or "")
                for _ in range(20):
                    line = resp.fp.readline()
                    if not line:
                        break
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        lines.append(text)
                    if any(t.startswith("data:") for t in lines):
                        break
            except (TimeoutError, OSError) as error:
                raise AssertionError(f"SSE 读取超时/中断：{error}") from None
            finally:
                conn.close()
            assert any(t.startswith("data:") or t.startswith(":") for t in lines), \
                f"SSE 未收到任何数据行：{lines!r}"
            data_lines = [t for t in lines if t.startswith("data:")]
            if data_lines:  # 收到帧快照时校验结构
                frame = json.loads(data_lines[0][len("data:"):])
                assert "tcp" in frame and "act" in frame and "iPOC" in frame

            # ---- on_event 事件进入事件流 ----
            dummy.on_event("warning", "测试事件")
            status, payload = _get_json(port, "/api/events")
            assert any(e["message"] == "测试事件" and e["level"] == "warning"
                       for e in payload["events"])
        finally:
            monitor.shutdown()
            monitor._http_thread.join(timeout=2)
            monitor._capacity_thread.join(timeout=2)
        assert not monitor._http_thread.is_alive()
        assert not monitor._capacity_thread.is_alive()
    print("web_monitor 集成测试 OK")


def _post_json(port: int, path: str, payload: object) -> tuple[int, dict]:
    body = json.dumps(payload).encode("utf-8")
    status, _, raw = _request(port, "POST", path, body=body)
    return status, json.loads(raw.decode("utf-8"))


def test_force_config_api() -> None:
    """/api/force_config：白名单读写 + 实时生效 + 落盘 + 非法输入拒绝。"""
    with tempfile.TemporaryDirectory() as d:
        force_config = ForceControlConfig()
        dummy = _make_dummy_server(d)
        dummy.calibration_config.force_control = force_config  # type: ignore[attr-defined]
        config_path = os.path.join(d, "ft_calibration_config.json")

        monitor = web_monitor.start_web_server(
            dummy, data_dir=d, port=0, host="127.0.0.1",
            force_config_path=config_path,
        )
        port = monitor.port
        try:
            # GET：返回白名单字段的实时值
            status, payload = _get_json(port, "/api/force_config")
            assert status == 200
            for key in web_monitor.FORCE_CONFIG_FIELDS:
                assert key in payload["force_control"], f"GET 缺少字段 {key}"
            assert payload["force_control"]["chisel_lateral_deadband_n"] == 15.0

            # POST 合法值：实时生效 + 写入配置文件
            status, payload = _post_json(
                port, "/api/force_config",
                {"chisel_lateral_deadband_n": 20.0, "chisel_lateral_max_mm": 12.5},
            )
            assert status == 200 and payload["ok"] is True
            assert force_config.chisel_lateral_deadband_n == 20.0
            assert force_config.chisel_lateral_max_mm == 12.5
            with open(config_path, encoding="utf-8") as f:
                on_disk = json.load(f)
            assert on_disk["force_control"]["chisel_lateral_deadband_n"] == 20.0
            assert on_disk["force_control"]["chisel_lateral_max_mm"] == 12.5

            # 钻轴枚举参数可在线切换
            status, payload = _post_json(port, "/api/force_config", {"axis": "Z"})
            assert status == 200 and force_config.axis == "Z"
            status, _ = _post_json(port, "/api/force_config", {"axis": "X"})
            assert status == 200 and force_config.axis == "X"

            # 对中参数（含布尔开关）同样生效并落盘
            status, payload = _post_json(
                port, "/api/force_config",
                {"align_chisel_enable": True, "align_deadband_nm": 0.8, "align_max_deg": 2.5},
            )
            assert status == 200 and payload["ok"] is True
            assert force_config.align_chisel_enable is True
            assert force_config.align_deadband_nm == 0.8
            assert force_config.align_max_deg == 2.5
            with open(config_path, encoding="utf-8") as f:
                on_disk = json.load(f)
            assert on_disk["force_control"]["align_chisel_enable"] is True

            # 事件流有更新记录
            status, payload = _get_json(port, "/api/events?limit=20")
            assert any("已更新力控参数" in e["message"] for e in payload["events"])

            # 非法输入全部 400：白名单外 / 超范围 / 符号 0 / 布尔传字符串 / 空对象 / 坏 JSON / 空体
            for bad in (
                {"axis": "W"},
                {"chisel_lateral_max_mm": 50.0},
                {"chisel_lateral_sign": 0},
                {"align_chisel_enable": "yes"},
                {"align_sign": 0},
                {"align_max_deg": 9.0},
                {"align_deadband_nm": 0.01},
                {},
            ):
                status, payload = _post_json(port, "/api/force_config", bad)
                assert status == 400, f"{bad} 应被拒绝: {status}"
                assert "error" in payload
            status, _, _ = _request(
                port, "POST", "/api/force_config", body=b"{not json"
            )
            assert status == 400
            status, _, _ = _request(port, "POST", "/api/force_config")
            assert status == 400
            # 合法值未被非法请求破坏
            assert force_config.chisel_lateral_deadband_n == 20.0

            # 力控配置不可用 -> 409
            original_server = monitor.server
            monitor.server = SimpleNamespace(calibration_config=None)
            try:
                status, _ = _get_json(port, "/api/force_config")
                assert status == 409
                status, _ = _post_json(port, "/api/force_config", {"default_target_force_n": 60})
                assert status == 409
            finally:
                monitor.server = original_server
        finally:
            monitor.shutdown()
            monitor._http_thread.join(timeout=2)
            monitor._capacity_thread.join(timeout=2)
    print("force_config API 集成测试 OK")


def test_debug_override_api() -> None:
    """/api/debug_override：白名单读写 + 服务端限幅 + 停止归零 + 非力控模式回包生效。"""
    import udp_server as udp

    # --- 非力控模式下 generate_response 也必须使用固定输出 ---
    srv = udp.RSIServer(udp.RSIConfig())
    srv.debug_override = {"enabled": True, "x": 0.01, "y": 0.0, "z": 0.0,
                          "a": 0.0, "b": 0.0, "c": 0.0, "ov_pro": 55.0}
    parsed = srv.parse_rsi_xml(udp.SAMPLE_ROB_XML.encode("utf-8"))
    assert parsed is not None
    assert srv._apply_debug_override() is True  # 运行循环每拍先应用，再回包
    reply = srv.generate_response(parsed)
    assert 'X="0.0100"' in reply and "<OV_PRO>55.0000</OV_PRO>" in reply, reply
    assert srv.config.rkorr["RKorr.X"] == 0.01 and srv.config.ov_pro == 55.0
    srv.disable_debug_override("测试")
    assert srv.debug_override["enabled"] is False
    reply2 = srv.generate_response(parsed)
    assert 'X="0.0000"' in reply2 and "<OV_PRO>100.0000</OV_PRO>" in reply2, reply2
    print("debug 固定输出回包（非力控模式）OK")

    # --- HTTP API：读写 + 校验 ---
    with tempfile.TemporaryDirectory() as d:
        dummy = _make_dummy_server(d)
        disabled_calls: list[str] = []

        def _disable(reason: str) -> None:
            disabled_calls.append(reason)
            dummy.debug_override = {"enabled": False, "x": 0.0, "y": 0.0, "z": 0.0,
                                    "a": 0.0, "b": 0.0, "c": 0.0, "ov_pro": 100.0}

        dummy.debug_override = None
        dummy.disable_debug_override = _disable  # type: ignore[attr-defined]

        monitor = web_monitor.start_web_server(dummy, data_dir=d, port=0, host="127.0.0.1")
        port = monitor.port
        try:
            # GET 默认态 + 限幅（dummy 无 force_control -> 回退 0.08/0.05）
            status, payload = _get_json(port, "/api/debug_override")
            assert status == 200
            assert payload["debug"]["enabled"] is False
            assert payload["debug"]["ov_pro"] == 100.0
            assert payload["limits"]["trans_max"] == 0.08
            assert payload["limits"]["rot_max"] == 0.05

            # POST 启用：写入 server.debug_override
            status, payload = _post_json(
                port, "/api/debug_override",
                {"enabled": True, "x": 0.02, "y": -0.01, "a": -0.005, "ov_pro": 80.0},
            )
            assert status == 200 and payload["ok"] is True
            assert dummy.debug_override["enabled"] is True
            assert dummy.debug_override["x"] == 0.02
            assert dummy.debug_override["a"] == -0.005
            assert dummy.debug_override["ov_pro"] == 80.0

            # 超限全部 400：平移超 per_cycle_max_mm / 旋转超 0.05 / OV 超 100 / 非法类型 / 白名单外
            for bad in (
                {"enabled": True, "x": 0.5},
                {"enabled": True, "a": 0.1},
                {"enabled": True, "ov_pro": 150},
                {"enabled": "yes"},
                {"rkorr": 1},
                {},
            ):
                status, payload = _post_json(port, "/api/debug_override", bad)
                assert status == 400, f"{bad} 应被拒绝: {status}"
                assert "error" in payload
            assert dummy.debug_override["x"] == 0.02  # 非法请求不破坏已生效值

            # 停止：调用服务端 disabler 并归零
            status, payload = _post_json(port, "/api/debug_override", {"enabled": False})
            assert status == 200
            assert disabled_calls and "Web 调试台停止" in disabled_calls[-1]
            assert payload["debug"]["enabled"] is False
            assert payload["debug"]["ov_pro"] == 100.0
        finally:
            monitor.shutdown()
            monitor._http_thread.join(timeout=2)
            monitor._capacity_thread.join(timeout=2)
    print("debug_override API 集成测试 OK")


def test_server_mode_api() -> None:
    """/api/server_mode：运行时切换 monitor/force（免重启）。"""
    import udp_server as udp
    from calibration_io import load_calibration_result, load_config
    from calibration_runner import CalibrationRunner

    # --- 服务端 set_service_mode 直测 ---
    srv = udp.RSIServer(udp.RSIConfig())
    cfg = load_config("ft_calibration_config.json")
    srv.calibration_config = cfg
    srv.calibration_runner = CalibrationRunner(cfg)  # 暂无标定结果
    assert srv.service_mode() == "monitor"
    try:
        srv.set_service_mode("force")
        raise AssertionError("无标定结果时启用力控应被拒绝")
    except ValueError as error:
        assert "标定" in str(error)
    srv.set_service_mode("monitor")  # 幂等
    # 有标定结果后切 force：创建力控器并立即生效
    srv.calibration_runner.calibration_result = load_calibration_result("ft_calibration.json")
    srv.set_service_mode("force")
    assert srv.force_mode and srv.force_requested and not srv.force_suspended
    assert srv.force_controller is not None
    # 切回 monitor：停固定输出、修正清零、倍率回 100
    srv.config.rkorr["RKorr.X"] = 0.02
    srv.debug_override = {"enabled": True, "x": 0.01, "y": 0.0, "z": 0.0,
                          "a": 0.0, "b": 0.0, "c": 0.0, "ov_pro": 50.0}
    srv.set_service_mode("monitor")
    assert not srv.force_mode
    assert srv.debug_override["enabled"] is False
    assert srv.config.rkorr["RKorr.X"] == 0.0
    assert srv.config.ov_pro == 100.0
    print("set_service_mode 单测 OK")

    # --- HTTP API ---
    with tempfile.TemporaryDirectory() as d:
        dummy = _make_dummy_server(d)

        def _set(mode: str) -> None:
            if mode == "force" and getattr(dummy.calibration_runner, "calibration_result", None) is None:
                raise ValueError("无标定结果，无法启用力控（请先完成标定）")
            dummy.force_mode = mode == "force"

        dummy.set_service_mode = _set  # type: ignore[attr-defined]
        monitor = web_monitor.start_web_server(dummy, data_dir=d, port=0, host="127.0.0.1")
        port = monitor.port
        try:
            status, payload = _get_json(port, "/api/server_mode")
            assert status == 200 and payload["mode"] == "monitor"
            assert payload["has_calibration"] is False

            # 无标定 -> force 被拒（409）
            status, payload = _post_json(port, "/api/server_mode", {"mode": "force"})
            assert status == 409 and "标定" in payload["error"]

            # 挂上标定结果后切换成功
            dummy.calibration_runner = SimpleNamespace(calibration_result=object())
            status, payload = _post_json(port, "/api/server_mode", {"mode": "force"})
            assert status == 200 and payload["mode"] == "force" and dummy.force_mode

            # 非法请求 400：未知模式 / 多余键
            status, _ = _post_json(port, "/api/server_mode", {"mode": "debug"})
            assert status == 400
            status, _ = _post_json(port, "/api/server_mode", {"mode": "force", "x": 1})
            assert status == 400

            # 切回监控
            status, payload = _post_json(port, "/api/server_mode", {"mode": "monitor"})
            assert status == 200 and payload["mode"] == "monitor" and not dummy.force_mode
            # 事件流断言在真实 RSIServer 上覆盖（见 test_force_e2e 阶段6 / 单测部分）
        finally:
            monitor.shutdown()
            monitor._http_thread.join(timeout=2)
            monitor._capacity_thread.join(timeout=2)
    print("server_mode API 集成测试 OK")


def test_frame_convention_api() -> None:
    """/api/frame_convention：手型约定读写 + 排列/手性自洽校验 + 推导参考表。"""
    with tempfile.TemporaryDirectory() as d:
        from calibration_models import FrameConvention

        dummy = _make_dummy_server(d)
        dummy.calibration_config.frame_convention = FrameConvention()  # type: ignore[attr-defined]
        config_path = os.path.join(d, "ft_calibration_config.json")
        monitor = web_monitor.start_web_server(
            dummy, data_dir=d, port=0, host="127.0.0.1", force_config_path=config_path
        )
        port = monitor.port
        try:
            # GET：默认约定 + 推导表（右手 拇Y/食Z/中X：推食指(+Z) -> My 读负；推拇指(+Y) -> Mz 读正）
            status, payload = _get_json(port, "/api/frame_convention")
            assert status == 200
            assert payload["convention"] == {"hand": "right", "thumb": "Y", "index": "Z", "middle": "X"}
            pushes = {p["finger"]: p for p in payload["derived"]["pushes"]}
            assert pushes["食指"]["expect_moment"] == "My 读负"
            assert pushes["拇指"]["expect_moment"] == "Mz 读正"
            assert payload["derived"]["feed_finger"] == "中指"

            # POST：合法的右手标准分配（拇X/食Y/中Z）
            status, payload = _post_json(
                port, "/api/frame_convention",
                {"hand": "right", "thumb": "X", "index": "Y", "middle": "Z"},
            )
            assert status == 200 and payload["ok"] is True
            assert dummy.calibration_config.frame_convention.thumb == "X"
            with open(config_path, encoding="utf-8") as f:
                on_disk = json.load(f)
            assert on_disk["frame_convention"]["middle"] == "Z"
            # 新分配下推导表更新：X 在拇指 -> 推食指(+Y) -> cross(X,Y)=Z -> Mz 读正
            pushes = {p["finger"]: p for p in payload["derived"]["pushes"]}
            assert pushes["食指"]["expect_moment"] == "Mz 读正"
            assert payload["derived"]["feed_finger"] == "拇指"

            # 非法：轴重复 / 未知手 / 手性不自洽（右手上 X×Z=-Y 的分配）
            for bad in (
                {"hand": "right", "thumb": "X", "index": "X", "middle": "Y"},
                {"hand": "both", "thumb": "X", "index": "Y", "middle": "Z"},
                {"hand": "right", "thumb": "X", "index": "Z", "middle": "Y"},  # 该分配符合左手
                {"hand": "left", "thumb": "X", "index": "Y", "middle": "Z"},   # 该分配符合右手
            ):
                status, payload = _post_json(port, "/api/frame_convention", bad)
                assert status == 400, f"{bad} 应被拒绝: {status}"
                assert "error" in payload
            # 合法左手分配通过
            status, payload = _post_json(
                port, "/api/frame_convention",
                {"hand": "left", "thumb": "X", "index": "Z", "middle": "Y"},
            )
            assert status == 200 and payload["convention"]["hand"] == "left"
        finally:
            monitor.shutdown()
            monitor._http_thread.join(timeout=2)
            monitor._capacity_thread.join(timeout=2)
    print("frame_convention API 集成测试 OK")


if __name__ == "__main__":
    test_whitelist()
    test_list_files()
    test_lock_unlock_delete()
    test_enforce_capacity()
    test_read_series()
    test_web_monitor()
    test_force_config_api()
    test_debug_override_api()
    test_server_mode_api()
    test_frame_convention_api()
    print("\nWeb 监控层测试全部通过")
