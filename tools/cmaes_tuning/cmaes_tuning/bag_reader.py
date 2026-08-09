"""Small ROS 2 bag deserialization boundary used by the offline evaluator."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


@dataclass(frozen=True)
class MessageRecord:
    timestamp_ns: int
    message: Any


@dataclass
class BagData:
    uri: Path
    topic_types: dict[str, str]
    records: dict[str, list[MessageRecord]]

    def topic(self, name: str) -> list[MessageRecord]:
        return self.records.get(name, [])


def read_bag(uri: str | Path, topics: Iterable[str] | None = None) -> BagData:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "rosbag2_py/rclpy are required; source ROS 2 Jazzy and this workspace"
        ) from error

    path = Path(uri).resolve()
    if not (path / "metadata.yaml").is_file():
        raise ValueError(f"bag metadata is missing: {path}")
    selected = set(topics) if topics is not None else None
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    unknown = set() if selected is None else selected - set(topic_types)
    if unknown:
        raise ValueError(f"required bag topics are absent: {sorted(unknown)}")
    message_classes = {
        name: get_message(type_name)
        for name, type_name in topic_types.items()
        if selected is None or name in selected
    }
    records = {name: [] for name in message_classes}
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic not in message_classes:
            continue
        records[topic].append(
            MessageRecord(
                timestamp_ns=int(timestamp),
                message=deserialize_message(serialized, message_classes[topic]),
            )
        )
    return BagData(uri=path, topic_types=topic_types, records=records)
