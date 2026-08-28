"""Record the agreed D455 and IWR6843ISK acquisition topics."""

from datetime import datetime
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


VALID_BAG_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+-]*$")


def _as_positive_float(value: str, name: str) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise RuntimeError(f"{name} must be a number: {value}") from error
    if parsed <= 0.0:
        raise RuntimeError(f"{name} must be positive: {value}")
    return parsed


def _as_positive_int(value: str, name: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise RuntimeError(f"{name} must be an integer: {value}") from error
    if parsed <= 0:
        raise RuntimeError(f"{name} must be positive: {value}")
    return parsed


def _load_topics(topics_file: Path) -> dict[str, str]:
    if not topics_file.is_file():
        raise RuntimeError(f"Bag topic config does not exist: {topics_file}")
    with topics_file.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    topics = config.get("topics") if isinstance(config, dict) else None
    if not isinstance(topics, dict) or not topics:
        raise RuntimeError(f"Bag topic config must contain a non-empty topics map: {topics_file}")

    result = {}
    for topic, type_name in topics.items():
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise RuntimeError(f"Invalid absolute topic name in {topics_file}: {topic}")
        if not isinstance(type_name, str) or "/msg/" not in type_name:
            raise RuntimeError(f"Invalid message type for {topic}: {type_name}")
        result[topic] = type_name
    return result


def _load_rate_monitor(rate_file: Path) -> list[str]:
    if not rate_file.is_file():
        raise RuntimeError(f"Rate monitor config does not exist: {rate_file}")
    with rate_file.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    topics = config.get("topics") if isinstance(config, dict) else None
    if not isinstance(topics, dict) or not topics:
        raise RuntimeError(
            f"Rate monitor config must contain a non-empty topics map: {rate_file}"
        )

    specifications = []
    for topic, properties in topics.items():
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise RuntimeError(f"Invalid monitor topic in {rate_file}: {topic}")
        if not isinstance(properties, dict):
            raise RuntimeError(f"Invalid monitor properties for {topic}")
        label = str(properties.get("label", "")).strip()
        type_name = str(properties.get("type", "")).strip()
        expected_hz = properties.get("expected_hz")
        if not label or "=" in label:
            raise RuntimeError(f"Invalid monitor label for {topic}: {label}")
        if "/msg/" not in type_name:
            raise RuntimeError(f"Invalid monitor type for {topic}: {type_name}")
        try:
            expected = float(expected_hz)
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"Invalid expected_hz for {topic}: {expected_hz}") from error
        if expected <= 0.0:
            raise RuntimeError(f"expected_hz must be positive for {topic}")
        specifications.append(f"{topic}={type_name}={label}={expected}")
    return specifications


def _launch_setup(context):
    output_root = Path(
        LaunchConfiguration("output_root").perform(context)
    ).expanduser().resolve()
    bag_name = LaunchConfiguration("bag_name").perform(context).strip() or "daq"
    topics_file = Path(
        LaunchConfiguration("topics_file").perform(context)
    ).expanduser().resolve()
    qos_file = Path(
        LaunchConfiguration("qos_overrides_file").perform(context)
    ).expanduser().resolve()
    rate_file = Path(
        LaunchConfiguration("rate_monitor_file").perform(context)
    ).expanduser().resolve()
    timeout_sec = _as_positive_float(
        LaunchConfiguration("preflight_timeout").perform(context),
        "preflight_timeout",
    )
    max_cache_size = _as_positive_int(
        LaunchConfiguration("max_cache_size").perform(context),
        "max_cache_size",
    )
    report_period_sec = _as_positive_float(
        LaunchConfiguration("rate_report_period").perform(context),
        "rate_report_period",
    )

    if not VALID_BAG_NAME.fullmatch(bag_name) or bag_name in {".", ".."}:
        raise RuntimeError(
            "bag_name may contain only letters, digits, '.', '_', '+', and '-', "
            "and must start with a letter or digit"
        )
    if not qos_file.is_file():
        raise RuntimeError(f"Bag QoS override file does not exist: {qos_file}")

    topics = _load_topics(topics_file)
    monitor_specifications = _load_rate_monitor(rate_file)
    now = datetime.now().astimezone()
    date_text = now.strftime("%Y%m%d")
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    day_directory = output_root / date_text
    run_directory = day_directory / f"{bag_name}_{timestamp}"
    suffix = 1
    while run_directory.exists():
        run_directory = day_directory / f"{bag_name}_{timestamp}_{suffix:02d}"
        suffix += 1
    run_directory.mkdir(parents=True)
    bag_path = run_directory / "bag"

    specifications = [f"{topic}={type_name}" for topic, type_name in topics.items()]
    preflight = Node(
        package="daq_bringup",
        executable="topic_ready_check.py",
        name="daq_topic_ready_check",
        parameters=[
            {
                "required_topics": specifications,
                "timeout_sec": timeout_sec,
            }
        ],
        output="screen",
    )
    rate_monitor = Node(
        package="daq_bringup",
        executable="topic_rate_monitor.py",
        name="daq_topic_rate_monitor",
        parameters=[
            {
                "monitored_topics": monitor_specifications,
                "report_period_sec": report_period_sec,
            }
        ],
        output="screen",
    )

    recorder_command = [
        "ros2",
        "bag",
        "record",
        "--output", str(bag_path),
        "--storage", "sqlite3",
        "--max-cache-size", str(max_cache_size),
        "--compression-mode", "none",
        "--qos-profile-overrides-path", str(qos_file),
        *topics.keys(),
    ]
    recorder = ExecuteProcess(
        cmd=recorder_command,
        name="daq_bag_recorder",
        output="screen",
        emulate_tty=True,
        sigterm_timeout="15",
        sigkill_timeout="5",
    )

    def _after_preflight(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "ERROR: DAQ recording was NOT started because required sensor "
                        "messages were missing."
                    )
                ),
                EmitEvent(
                    event=Shutdown(reason="DAQ topic preflight failed")
                ),
            ]
        return [
            LogInfo(msg=f"Recording DAQ bag to: {bag_path}"),
            LogInfo(msg="Stop safely with Ctrl-C and wait for metadata.yaml."),
            recorder,
        ]

    return [
        LogInfo(msg=f"Prepared DAQ run directory: {run_directory}"),
        LogInfo(msg="Rosbag storage compression: none"),
        rate_monitor,
        preflight,
        RegisterEventHandler(
            OnProcessExit(
                target_action=preflight,
                on_exit=_after_preflight,
            )
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("daq_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_root",
                default_value=str(Path.home() / "daq_bags"),
                description="Root directory for date/run/bag output folders",
            ),
            DeclareLaunchArgument(
                "bag_name",
                default_value="daq",
                description="Experiment label used before the timestamp",
            ),
            DeclareLaunchArgument(
                "topics_file",
                default_value=str(package_share / "config" / "bag" / "record_topics.yaml"),
                description="YAML map of required topic names and types",
            ),
            DeclareLaunchArgument(
                "qos_overrides_file",
                default_value=str(package_share / "config" / "bag" / "qos_overrides.yaml"),
                description="Rosbag2 recorder subscription QoS overrides",
            ),
            DeclareLaunchArgument(
                "rate_monitor_file",
                default_value=str(package_share / "config" / "bag" / "rate_monitor.yaml"),
                description="Topics and expected Hz shown while recording",
            ),
            DeclareLaunchArgument(
                "rate_report_period",
                default_value="2.0",
                description="Seconds per live sensor-rate report",
            ),
            DeclareLaunchArgument(
                "preflight_timeout",
                default_value="20.0",
                description="Seconds to wait for one message from every required topic",
            ),
            DeclareLaunchArgument(
                "max_cache_size",
                default_value="268435456",
                description="Rosbag2 cache bytes (256 MiB by default)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
