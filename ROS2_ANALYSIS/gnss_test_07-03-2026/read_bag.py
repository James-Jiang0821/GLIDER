from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

MCAP_FILE = "gnsstest_0.mcap"

with open(MCAP_FILE, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])

    for schema, channel, message, decoded in reader.iter_decoded_messages():
        print("-" * 80)
        print("Topic:", channel.topic)

        if hasattr(decoded, "latitude") and hasattr(decoded, "longitude"):
            print("Latitude :", decoded.latitude)
            print("Longitude:", decoded.longitude)

        if hasattr(decoded, "altitude"):
            print("Altitude :", decoded.altitude)

        if hasattr(decoded, "header"):
            print("Frame ID :", decoded.header.frame_id)
            print("Stamp    :", decoded.header.stamp.sec, decoded.header.stamp.nanosec)

        print("Full message:")
        print(decoded)