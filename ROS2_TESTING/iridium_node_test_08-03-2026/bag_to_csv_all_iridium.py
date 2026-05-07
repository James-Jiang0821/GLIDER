import csv
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

MCAP_FILE = "iridiumtest2_0.mcap"

signal_rows = []
incoming_rows = []
status_rows = []

with open(MCAP_FILE, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])

    for schema, channel, message, decoded in reader.iter_decoded_messages():
        topic = channel.topic

        if topic == "/iridium/signal_strength":
            signal_rows.append({
                "topic": topic,
                "log_time_ns": message.log_time,
                "signal_strength": decoded.data,
            })

        elif topic == "/iridium/incoming_message":
            incoming_rows.append({
                "topic": topic,
                "log_time_ns": message.log_time,
                "incoming_message": decoded.data,
            })

        #/iridium/status is std_msgs/String
        elif topic == "/iridium/status":
            status_rows.append({
                "topic": topic,
                "log_time_ns": message.log_time,
                "status": decoded.data,
            })


with open("iridium_signal_strength.csv", "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "topic", "log_time_ns", "signal_strength"
    ])
    writer.writeheader()
    writer.writerows(signal_rows)

with open("iridium_incoming_message.csv", "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "topic", "log_time_ns", "incoming_message"
    ])
    writer.writeheader()
    writer.writerows(incoming_rows)

with open("iridium_status.csv", "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "topic", "log_time_ns", "status"
    ])
    writer.writeheader()
    writer.writerows(status_rows)

print(f"/iridium/signal_strength rows: {len(signal_rows)}")
print(f"/iridium/incoming_message rows: {len(incoming_rows)}")
print(f"/iridium/status rows: {len(status_rows)}")
