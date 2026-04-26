import os
import pandas as pd

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


BAG_FILE = "pressure_test_0.mcap"  #change if needed
OUTPUT_DIR = "csv_output"


def topic_to_filename(topic_name: str) -> str:
    """Convert ROS topic name into a safe CSV filename."""
    return topic_name.replace("/", "_").strip("_") + ".csv"


def ros_message_to_row(ros_msg, log_time_ns: int) -> dict:
    """
    Convert a decoded ROS2 message into a flat dict row for CSV export.
    This handles common simple message types and some ROS log fields.
    """
    row = {
        "timestamp_ns": log_time_ns,
        "timestamp_s": log_time_ns / 1e9,
    }

    #std_msgs Float64 or String
    if hasattr(ros_msg, "data"):
        row["data"] = ros_msg.data

    #sensor_msgs/Temperature
    if hasattr(ros_msg, "temperature"):
        row["temperature"] = ros_msg.temperature
    if hasattr(ros_msg, "variance"):
        row["variance"] = ros_msg.variance

    #sensor_msgs/FluidPressure
    if hasattr(ros_msg, "fluid_pressure"):
        row["fluid_pressure"] = ros_msg.fluid_pressure
    if hasattr(ros_msg, "variance"):
        row["variance"] = ros_msg.variance

    #rcl_interfaces/msg/Log for /rosout
    if hasattr(ros_msg, "stamp"):
        if hasattr(ros_msg.stamp, "sec"):
            row["stamp_sec"] = ros_msg.stamp.sec
        if hasattr(ros_msg.stamp, "nanosec"):
            row["stamp_nanosec"] = ros_msg.stamp.nanosec

    if hasattr(ros_msg, "level"):
        row["level"] = ros_msg.level
    if hasattr(ros_msg, "name"):
        row["name"] = ros_msg.name
    if hasattr(ros_msg, "msg"):
        row["msg"] = ros_msg.msg
    if hasattr(ros_msg, "file"):
        row["file"] = ros_msg.file
    if hasattr(ros_msg, "function"):
        row["function"] = ros_msg.function
    if hasattr(ros_msg, "line"):
        row["line"] = ros_msg.line

    return row


def main():
    if not os.path.exists(BAG_FILE):
        raise FileNotFoundError(f"Bag file not found: {BAG_FILE}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    decoder_factory = DecoderFactory()
    topic_data = {}

    with open(BAG_FILE, "rb") as f:
        reader = make_reader(f, decoder_factories=[decoder_factory])

        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            topic = channel.topic

            if topic not in topic_data:
                topic_data[topic] = []

            row = ros_message_to_row(ros_msg, message.log_time)
            topic_data[topic].append(row)

    if not topic_data:
        print("No decoded messages found in the bag.")
        return

    print("Topics found:")
    for topic, rows in topic_data.items():
        print(f"  {topic}: {len(rows)} messages")

    print("\nSaving CSV files...")
    for topic, rows in topic_data.items():
        if not rows:
            continue

        df = pd.DataFrame(rows)
        output_file = os.path.join(OUTPUT_DIR, topic_to_filename(topic))
        df.to_csv(output_file, index=False)
        print(f"Saved: {output_file}")

    print("\nDone.")


if __name__ == "__main__":
    main()
