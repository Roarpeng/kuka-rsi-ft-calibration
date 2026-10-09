# -*- coding: utf-8 -*-
"""临时调试录制器：每 200ms 记录 /api/debug_override + /api/status 摘要。

用途：实机验证"RSI 固定输出是否生效"。per-frame CSV 已记录每拍 rkorr/Act/OV_PRO，
本录制器补记 debug_override 的启停时刻与设定值，便于对齐时间窗。
用法：python record_debug_session.py [时长秒，默认 1800]；到时自动结束。"""
from __future__ import annotations

import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8080"
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 1800.0
OUT = time.strftime("data/debug_record_%Y%m%d_%H%M%S.jsonl")


def get(path: str) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=1.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


last_line = None
last_beat = 0.0
end = time.time() + DURATION_S
with open(OUT, "w", encoding="utf-8") as handle:
    print(f"[录制] 写入 {OUT}，时长 {DURATION_S / 60:.0f} 分钟", flush=True)
    while time.time() < end:
        try:
            dbg = get("/api/debug_override")["debug"]
            st = get("/api/status")
            row = {
                "t": round(time.time(), 3),
                "debug_enabled": dbg["enabled"],
                "debug_set": {k: dbg[k] for k in ("x", "y", "z", "a", "b", "c", "ov_pro")},
                "packets": st["connection"]["packet_count"],
                "rkorr_x": st["rkorr"]["RKorr.X"],
                "rkorr_y": st["rkorr"]["RKorr.Y"],
                "rkorr_z": st["rkorr"]["RKorr.Z"],
                "rkorr_a": st["rkorr"]["RKorr.A"],
                "rkorr_b": st["rkorr"]["RKorr.B"],
                "rkorr_c": st["rkorr"]["RKorr.C"],
                "ov_pro": st["ov_pro"],
                "active_csv": st["active_csv"],
            }
        except Exception as error:
            row = {"t": round(time.time(), 3), "error": str(error)}
        line = json.dumps(row, ensure_ascii=False)
        now = time.time()
        if line != last_line or now - last_beat >= 2.0:  # 状态变化立即记 + 2s 心跳
            handle.write(line + "\n")
            handle.flush()
            last_line = line
            last_beat = now
        time.sleep(0.2)
print("[录制] 结束", flush=True)
