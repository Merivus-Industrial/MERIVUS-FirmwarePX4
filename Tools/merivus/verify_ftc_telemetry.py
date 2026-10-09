#!/usr/bin/env python3

from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
DEFINITION_ROOT = ROOT / "src/modules/mavlink/message_definitions/v1.0"
STREAM_ROOT = ROOT / "src/modules/mavlink/streams"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    wrapper = ET.parse(DEFINITION_ROOT / "merivus.xml").getroot()
    includes = [element.text for element in wrapper.findall("include")]
    require(includes == ["development.xml", "merivus_ftc.xml"], "固件方言必须先继承 development，再包含 FTC 定义")

    core_path = DEFINITION_ROOT / "merivus_ftc.xml"
    core = ET.parse(core_path).getroot()
    messages = {
        message.attrib["name"]: int(message.attrib["id"])
        for message in core.findall("./messages/message")
    }
    expected_messages = {
        "MERIVUS_FTC_MOTOR_STATUS": 60000,
        "MERIVUS_FTC_CONTROL_STATUS": 60001,
        "MERIVUS_FTC_EXTREME_STATUS": 60002,
        "MERIVUS_FTC_DIAGNOSTICS": 60003,
    }
    require(messages == expected_messages, f"消息 ID 不符合契约：{messages}")

    for message in core.findall("./messages/message"):
        fields = {field.attrib["name"] for field in message.findall("field")}
        require("protocol_version" in fields, f"{message.attrib['name']} 缺少 protocol_version")

    source_text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in STREAM_ROOT.glob("MERIVUS_FTC_*.hpp")
    )
    topics = {
        "motor_health_status",
        "ftc_model_status",
        "ftc_effectiveness_matrix",
        "ftc_allocation_shadow",
        "ftc_control_authority",
        "ftc_extreme_state",
        "ftc_recovery_status",
        "ftc_system_status",
        "ftc_simulation_status",
    }
    missing_topics = sorted(topic for topic in topics if f"ORB_ID({topic})" not in source_text)
    require(not missing_topics, f"未覆盖 uORB 主题：{', '.join(missing_topics)}")

    control_text = (STREAM_ROOT / "MERIVUS_FTC_CONTROL_STATUS.hpp").read_text(encoding="utf-8")
    require("if (_system.intervention_enabled)" in control_text, "ACTIVE 必须来自实际仲裁反馈")
    require("msg.control_mode = _system.mode" in control_text, "模式必须由 Supervisor 统一发布")

    main_text = (ROOT / "src/modules/mavlink/mavlink_main.cpp").read_text(encoding="utf-8")
    for stream_name, rate in {
        "MERIVUS_FTC_MOTOR_STATUS": "5.0f",
        "MERIVUS_FTC_CONTROL_STATUS": "5.0f",
        "MERIVUS_FTC_EXTREME_STATUS": "10.0f",
        "MERIVUS_FTC_DIAGNOSTICS": "1.0f",
    }.items():
        require(f'configure_stream_local("{stream_name}", {rate})' in main_text, f"{stream_name} 默认频率不正确")

    for board in ("boards/px4/sitl/default.px4board", "boards/px4/fmu-v6c/default.px4board"):
        require('CONFIG_MAVLINK_DIALECT="merivus"' in (ROOT / board).read_text(encoding="utf-8"), f"{board} 未启用 merivus 方言")

    ground_core = ROOT.parent / "GroundStation/schemas/mavlink/merivus_ftc.xml"
    digest = hashlib.sha256(core_path.read_bytes()).hexdigest()
    if ground_core.exists():
        require(hashlib.sha256(ground_core.read_bytes()).hexdigest() == digest, "固件与地面站 FTC XML 不一致")

    print(f"FTC telemetry contract OK: sha256={digest}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as error:
        print(f"FTC telemetry contract FAILED: {error}", file=sys.stderr)
        raise SystemExit(1)
