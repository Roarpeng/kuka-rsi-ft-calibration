"""CSV 运行数据管理模块：文件列举、锁定、删除、容量控制与历史回放读取。

供 Web 监控层调用。CSV 文件位于 data/ 目录，命名 rsi_data_YYYYMMDD_HHMMSS.csv，
表头 36 列（见 udp_server.py 的 CSV_HEADER），首列 timestamp 为
"%Y-%m-%d %H:%M:%S.%f"（实际写入截断到毫秒）。
"""

from __future__ import annotations

import csv
import math
import os
import re
from datetime import datetime
from typing import Optional

# 白名单文件名（防路径穿越）：仅允许 rsi_data_YYYYMMDD_HHMMSS.csv
CSV_NAME_RE: re.Pattern[str] = re.compile(r"^rsi_data_\d{8}_\d{6}\.csv$")

# CSV 首列时间戳格式（%f 可匹配 1~6 位微秒，兼容截断到毫秒的写法）
_TIMESTAMP_FMT = "%Y-%m-%d %H:%M:%S.%f"
# read_series 的 start/end 过滤参数格式
_FILTER_FMT = "%Y-%m-%d %H:%M:%S"

# 始终保持字符串、不做 float 转换的列
_STRING_COLUMNS = frozenset({"sample_status"})
# 首列单独放入 timestamps，不进 series
_TIMESTAMP_COLUMN = "timestamp"


def is_valid_csv_name(name: str) -> bool:
    """仅允许白名单文件名，防止路径穿越。"""
    return bool(CSV_NAME_RE.match(name))


def _lock_path(data_dir: str, name: str) -> str:
    """<name>.lock sidecar 路径。"""
    return os.path.join(data_dir, name + ".lock")


def _basename_or_empty(active_name: Optional[str]) -> str:
    """active_name 可能是路径，取 basename 参与比较。"""
    return os.path.basename(active_name) if active_name else ""


def list_files(data_dir: str, active_name: Optional[str] = None) -> list[dict]:
    """扫描 data_dir 下匹配白名单的 csv，按修改时间倒序返回元信息列表。

    每项 {"name", "size_bytes", "mtime" (float), "locked", "active"}；
    data_dir 不存在时返回空列表。
    """
    if not os.path.isdir(data_dir):
        return []
    active_base = _basename_or_empty(active_name)
    entries: list[dict] = []
    with os.scandir(data_dir) as it:
        for entry in it:
            if not entry.is_file() or not is_valid_csv_name(entry.name):
                continue
            try:
                st = entry.stat()
            except OSError:
                continue  # 枚举期间被删/被移动，跳过
            entries.append({
                "name": entry.name,
                "size_bytes": st.st_size,
                "mtime": st.st_mtime,
                "locked": os.path.exists(_lock_path(data_dir, entry.name)),
                "active": entry.name == active_base,
            })
    entries.sort(key=lambda e: e["mtime"], reverse=True)
    return entries


def lock_file(data_dir: str, name: str) -> None:
    """创建 <name>.lock sidecar；name 不合法抛 ValueError。"""
    if not is_valid_csv_name(name):
        raise ValueError(f"非法 CSV 文件名：{name!r}")
    path = _lock_path(data_dir, name)
    with open(path, "w", encoding="utf-8"):
        pass  # 仅作为存在性标记，内容为空


def unlock_file(data_dir: str, name: str) -> None:
    """删除 <name>.lock sidecar；name 不合法抛 ValueError；sidecar 不存在视为成功。"""
    if not is_valid_csv_name(name):
        raise ValueError(f"非法 CSV 文件名：{name!r}")
    try:
        os.remove(_lock_path(data_dir, name))
    except FileNotFoundError:
        pass


def delete_file(data_dir: str, name: str, active_name: Optional[str] = None) -> None:
    """删除 csv 及其 .lock sidecar。

    name 不合法、文件不存在、或删除的是活动文件时抛 ValueError。
    """
    if not is_valid_csv_name(name):
        raise ValueError(f"非法 CSV 文件名：{name!r}")
    if name == _basename_or_empty(active_name):
        raise ValueError(f"不能删除正在写入的活动文件：{name}")
    path = os.path.join(data_dir, name)
    if not os.path.isfile(path):
        raise ValueError(f"文件不存在：{name}")
    os.remove(path)
    try:
        os.remove(_lock_path(data_dir, name))
    except FileNotFoundError:
        pass


def total_size_bytes(data_dir: str) -> int:
    """data_dir 下所有白名单 csv 的总字节数；目录不存在返回 0。"""
    if not os.path.isdir(data_dir):
        return 0
    total = 0
    with os.scandir(data_dir) as it:
        for entry in it:
            if not entry.is_file() or not is_valid_csv_name(entry.name):
                continue
            try:
                total += entry.stat().st_size
            except OSError:
                continue
    return total


def enforce_capacity(
    data_dir: str,
    cap_bytes: int,
    active_name: Optional[str] = None,
) -> list[str]:
    """总量超 cap_bytes 时按修改时间从最旧开始删除，返回被删除的文件名列表。

    跳过锁定与活动文件；只剩锁定/活动文件仍超额时停止，返回已删除的部分。
    """
    if not os.path.isdir(data_dir):
        return []
    total = total_size_bytes(data_dir)
    if total <= cap_bytes:
        return []
    # list_files 是倒序，翻转为最旧在前
    candidates = [e for e in reversed(list_files(data_dir, active_name))]
    deleted: list[str] = []
    for entry in candidates:
        if total <= cap_bytes:
            break
        if entry["locked"] or entry["active"]:
            continue
        try:
            delete_file(data_dir, entry["name"], active_name)
        except (OSError, ValueError):
            continue  # 与删除并发竞争，放弃这条
        total -= entry["size_bytes"]
        deleted.append(entry["name"])
    return deleted


def _parse_timestamp(text: str) -> Optional[datetime]:
    """解析 CSV 首列时间戳；失败返回 None。"""
    try:
        return datetime.strptime(text, _TIMESTAMP_FMT)
    except ValueError:
        return None


def _parse_filter_bound(text: str, label: str) -> datetime:
    """解析 start/end 过滤边界；格式非法抛 ValueError。"""
    try:
        return datetime.strptime(text, _FILTER_FMT)
    except ValueError:
        raise ValueError(f"{label} 格式应为 \"YYYY-MM-DD HH:MM:SS\"：{text!r}") from None


def _row_timestamp(row: list[str]) -> tuple[str, Optional[datetime]]:
    """取行首时间戳原文与解析结果（解析失败为 None）。"""
    text = row[0].strip()
    return text, _parse_timestamp(text)


def read_series(
    path: str,
    start: Optional[str] = None,
    end: Optional[str] = None,
    max_points: int = 2000,
) -> dict:
    """读 CSV 供历史回放，返回 {"columns", "timestamps", "series"}。

    start/end 为 "YYYY-MM-DD HH:MM:SS" 可选时间过滤（含边界）；
    时间戳无法解析的行不受过滤影响、始终保留。
    行数超 max_points 时按等距步长抽稀（始终保留首行/末行）。
    timestamp 列解析失败时保留原始字符串；数值列转 float，sample_status 保持 str。
    大文件友好：两遍扫描（第一遍计数算步长，第二遍抽取）。
    """
    start_dt = _parse_filter_bound(start, "start") if start else None
    end_dt = _parse_filter_bound(end, "end") if end else None
    if start_dt is not None and end_dt is not None and start_dt > end_dt:
        raise ValueError(f"start 晚于 end：{start!r} > {end!r}")
    max_points = max(1, max_points)

    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return {"columns": [], "timestamps": [], "series": {}}
        n_cols = len(header)

        def _in_range(ts: Optional[datetime]) -> bool:
            """时间戳解析失败的行不过滤（始终保留）。"""
            if ts is None:
                return True
            if start_dt is not None and ts < start_dt:
                return False
            if end_dt is not None and ts > end_dt:
                return False
            return True

        # 第一遍：数命中行数，算抽稀步长
        count = 0
        for row in reader:
            if len(row) != n_cols:
                continue
            _, ts = _row_timestamp(row)
            if _in_range(ts):
                count += 1

    step = max(1, math.ceil(count / max_points)) if count > 0 else 1

    columns = header
    data_columns = [c for c in columns if c != _TIMESTAMP_COLUMN]
    series: dict[str, list] = {c: [] for c in data_columns}
    timestamps: list[str] = []

    # 第二遍：按步长抽取，同时记住最后一行命中行，保证末行始终保留
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader, None)  # 跳过表头
        last_row: Optional[list[str]] = None
        hit = 0
        for row in reader:
            if len(row) != n_cols:
                continue
            ts_text, ts = _row_timestamp(row)
            if not _in_range(ts):
                continue
            if hit % step == 0:
                _append_row(row, columns, series)
                timestamps.append(ts_text)
            last_row = row
            hit += 1
        # 末行未被步长选中时补入（首行 hit=0 必然被选中）
        if count > 1 and (count - 1) % step != 0 and last_row is not None:
            ts_text, _ = _row_timestamp(last_row)
            _append_row(last_row, columns, series)
            timestamps.append(ts_text)

    return {"columns": columns, "timestamps": timestamps, "series": series}


def _append_row(row: list[str], columns: list[str], series: dict[str, list]) -> None:
    """把一行数据按列追加进 series；数值转 float，失败或 sample_status 保留原串。"""
    for name, cell in zip(columns, row):
        if name == _TIMESTAMP_COLUMN:
            continue
        value = cell.strip()
        if name in _STRING_COLUMNS:
            series[name].append(value)
            continue
        try:
            series[name].append(float(value))
        except ValueError:
            series[name].append(value)
