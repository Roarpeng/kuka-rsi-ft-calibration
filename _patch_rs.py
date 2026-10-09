# -*- coding: utf-8 -*-
"""一次性补丁 v2：RobotStatus BOOL -> INT。用后即删。"""
import ast

with open("udp_server.py", encoding="utf-8") as f:
    t = f.read()

edits = [
    # (old, new, tag)
    ('''    RobotStatus: bool = False      # FALSE=钻孔（控 $OV_PRO）；TRUE=凿击（X 恒力 OV_PRO + Y/Z 横向让位）''',
     '''    RobotStatus: int = 2           # INT：1=标定 2=钻孔（OV_PRO 力-速度）3=凿击（恒力+横向让位+对中）''', "a"),

    ('"<RobotStatus>FALSE</RobotStatus>"',
     '"<RobotStatus>2</RobotStatus>"', "b"),

    ('''        ("RobotStatus", "BOOL", 14),  # FALSE=钻孔控倍率；TRUE=凿击（X 恒力 + Y/Z 横向让位）''',
     '''        ("RobotStatus", "INT", 14),   # 1=标定；2=钻孔（OV_PRO 力-速度）；3=凿击（恒力+横向让位+对中）''', "c"),

    ('''                elif elem_type == "BOOL":
                    setattr(rsi_data, tag, False)
''',
     '''                elif elem_type == "BOOL":
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
''', "d"),

    ("            and parsed.RobotStatus is False",
     "            and parsed.RobotStatus == 2", "e"),

    ('''                1 if rsi_data.RobotStatus else 0,''',
     '''                rsi_data.RobotStatus,''', "f"),

    ('''            mode="chisel" if rsi_data.RobotStatus else "drill",''',
     '''            mode="chisel" if rsi_data.RobotStatus == 3 else "drill",''', "g"),

    ('''        if (
            not rsi_data.data_collection
            or self._last_data_collection
            or self.auto_calibrating
            or self.calibration_config is None
            or self.calibration_runner is None
            or self.calibration_config.mode == "calibration_collect"
        ):
            return''',
     '''        data_collection_rise = rsi_data.data_collection and not self._last_data_collection
        robot_status_enter_calib = rsi_data.RobotStatus == 1 and self._last_robot_status != 1
        if (
            not (data_collection_rise or robot_status_enter_calib)
            or self.auto_calibrating
            or self.calibration_config is None
            or self.calibration_runner is None
            or self.calibration_config.mode == "calibration_collect"
        ):
            return''', "h"),

    ('''        message = "[标定] 检测到标定程序信号（data_collection=TRUE），自动进入标定模式"''',
     '''        source = "RobotStatus=1" if robot_status_enter_calib else "data_collection=TRUE"
        message = f"[标定] 检测到标定程序信号（{source}），自动进入标定模式"''', "i"),

    ('''        self._last_data_collection = False  # 上一帧 data_collection，用于上升沿检测''',
     '''        self._last_data_collection = False  # 上一帧 data_collection，用于上升沿检测
        self._last_robot_status = 2         # 上一帧 RobotStatus（INT），用于进入标定的沿检测''', "j"),

    ('''        self._check_auto_calibration(rsi_data)
        self._last_data_collection = rsi_data.data_collection''',
     '''        self._check_auto_calibration(rsi_data)
        self._last_data_collection = rsi_data.data_collection
        self._last_robot_status = rsi_data.RobotStatus''', "k"),

    ('''                                f"RobotStatus={'凿击' if rsi_data.RobotStatus else '钻孔'}"''',
     '''                                f"RobotStatus={rsi_data.RobotStatus}"''', "l"),

    ('''        help="启动力控模式：重力补偿 + 恒力钻孔（RobotStatus=FALSE 控 OV_PRO；TRUE 凿击横向让位）"''',
     '''        help="启动力控模式（RobotStatus: 1=标定 2=钻孔控OV_PRO 3=凿击恒力+横向让位）"''', "m"),

    ('''        print(f"倍率：RobotStatus=FALSE 钻孔按力映射 $OV_PRO 0–100%（每拍 ≤{fc.ov_pro_slew_pct:.1f}%）")''',
     '''        print(f"倍率：RobotStatus=2 钻孔按力映射 $OV_PRO 0–100%（每拍 ≤{fc.ov_pro_slew_pct:.1f}%）；1=自动标定；3=凿击")''', "n"),
]

for old, new, tag in edits:
    assert old in t, f"anchor {tag}"
    t = t.replace(old, new)

ast.parse(t)
with open("udp_server.py", "w", encoding="utf-8") as f:
    f.write(t)
print("udp_server OK")

with open("web_monitor.py", encoding="utf-8") as f:
    t = f.read()
old = '''        "robot_status": bool(rsi_data.RobotStatus),'''
new = '''        "robot_status": int(getattr(rsi_data, "RobotStatus", 2)),'''
assert old in t
t = t.replace(old, new)
with open("web_monitor.py", "w", encoding="utf-8") as f:
    f.write(t)
print("web_monitor OK")

for path in ("web_static/app.js", "web_static/debug.js"):
    with open(path, encoding="utf-8") as f:
        t = f.read()
    old = '''    if (typeof frame.robot_status === "boolean" && frame.robot_status !== lastRobotStatus) {
      lastRobotStatus = frame.robot_status;
      setLamp("lamp-chisel", frame.robot_status ? "kuka" : null,
        frame.robot_status ? "凿击" : "钻孔");
    }'''
    new = '''    if (frame.robot_status !== undefined && frame.robot_status !== null
        && frame.robot_status !== lastRobotStatus) {
      lastRobotStatus = frame.robot_status;
      if (frame.robot_status === 1) setLamp("lamp-chisel", "warn", "标定");
      else if (frame.robot_status === 3) setLamp("lamp-chisel", "kuka", "凿击");
      else setLamp("lamp-chisel", null, "钻孔");
    }'''
    assert old in t, path
    t = t.replace(old, new)
    with open(path, "w", encoding="utf-8") as f:
        f.write(t)
    print(path, "OK")
