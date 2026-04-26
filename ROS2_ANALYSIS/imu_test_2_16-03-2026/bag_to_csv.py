import csv
import os
import re
from collections import defaultdict
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

MCAP_FILE = "imu_test_2_0.mcap"
OUTPUT_DIR = "split_topics_csv"


def safe_filename(topic_name: str) -> str:
    """
    Convert a ROS topic name into a safe filename.
    Example:
        /gps/fix -> gps_fix.csv
        /iridium/incoming_message -> iridium_incoming_message.csv
    """
    name = topic_name.strip("/")
    if not name:
        name = "root_topic"
    name = name.replace("/", "_")
    name = re.sub(r"[^A-Za-z0-9_\-]", "_", name)
    return f"{name}.csv"


#group rows by topic
topic_rows = defaultdict(list)
topic_types = {}

with open(MCAP_FILE, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])

    for schema, channel, message, decoded in reader.iter_decoded_messages():
        topic = channel.topic
        msg_type = schema.name if schema is not None else "unknown"

        topic_types[topic] = msg_type

        topic_rows[topic].append({
            "log_time_ns": message.log_time,
            "topic": topic,
            "type": msg_type,
            "message": str(decoded),
        })

#make output folder
os.makedirs(OUTPUT_DIR, exist_ok=True)

#write one CSV per topic
for topic, rows in topic_rows.items():
    filename = safe_filename(topic)
    filepath = os.path.join(OUTPUT_DIR, filename)

    with open(filepath, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(
            csvfile,
            fieldnames=["log_time_ns", "topic", "type", "message"]
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} rows -> {filepath}")

print("\nDone.")
print(f"Created {len(topic_rows)} CSV files in '{OUTPUT_DIR}'")
