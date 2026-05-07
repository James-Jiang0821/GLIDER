import csv
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

MCAP_FILE = "gnsstest_0.mcap"
CSV_FILE = "all_topics.csv"

rows = []

with open(MCAP_FILE, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])

    for schema, channel, message, decoded in reader.iter_decoded_messages():
        rows.append({
            "topic": channel.topic,
            "log_time_ns": message.log_time,
            "type": schema.name if schema is not None else "",
            "message": str(decoded),
        })

with open(CSV_FILE, "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.DictWriter(
        csvfile,
        fieldnames=["topic", "log_time_ns", "type", "message"]
    )
    writer.writeheader()
    writer.writerows(rows)

print(f"Wrote {len(rows)} messages to {CSV_FILE}")