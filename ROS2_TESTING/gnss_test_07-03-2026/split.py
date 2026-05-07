import csv
import re

INPUT_CSV = "all_topics.csv"

FIX_OUT = "gps_fix_clean.csv"
VEL_OUT = "gps_vel_clean.csv"
DIAG_OUT = "gps_diagnostics_clean.csv"


def extract(pattern, text, cast=str, default=""):
    match = re.search(pattern, text)
    if not match:
        return default
    value = match.group(1)
    try:
        return cast(value)
    except Exception:
        return default


fix_rows = []
vel_rows = []
diag_rows = []

with open(INPUT_CSV, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)

    for row in reader:
        topic = row["topic"]
        msg = row["message"]
        log_time_ns = row["log_time_ns"]

        if topic == "/gps/fix":
            fix_rows.append({
                "log_time_ns": log_time_ns,
                "stamp_sec": extract(r"stamp=Time\(sec=(\d+)", msg, int),
                "stamp_nanosec": extract(r"stamp=Time\(sec=\d+, nanosec=(\d+)\)", msg, int),
                "frame_id": extract(r"frame_id=([^,\)]+)", msg),
                "status": extract(r"status=NavSatStatus\(status=([-\d]+)", msg, int),
                "service": extract(r"service=([-\d]+)\)", msg, int),
                "latitude": extract(r"latitude=([-\d\.eE]+)", msg, float),
                "longitude": extract(r"longitude=([-\d\.eE]+)", msg, float),
                "altitude": extract(r"altitude=([-\d\.eE]+)", msg, float),
                "cov_xx": extract(r"position_covariance=\[([-\d\.eE]+)", msg, float),
                "cov_yy": extract(r"position_covariance=\[[^\]]*?, [^,\]]+, [^,\]]+, [^,\]]+, ([-\d\.eE]+)", msg, float),
                "cov_zz": extract(r"position_covariance=\[[^\]]*?, [^,\]]+, [^,\]]+, [^,\]]+, [^,\]]+, [^,\]]+, [^,\]]+, [^,\]]+, ([-\d\.eE]+)\]", msg, float),
                "covariance_type": extract(r"position_covariance_type=(\d+)", msg, int),
            })

        elif topic == "/gps/vel":
            vel_rows.append({
                "log_time_ns": log_time_ns,
                "stamp_sec": extract(r"stamp=Time\(sec=(\d+)", msg, int),
                "stamp_nanosec": extract(r"stamp=Time\(sec=\d+, nanosec=(\d+)\)", msg, int),
                "frame_id": extract(r"frame_id=([^,\)]+)", msg),
                "linear_x": extract(r"linear=Vector3\(x=([-\d\.eE]+)", msg, float),
                "linear_y": extract(r"linear=Vector3\(x=[-\d\.eE]+, y=([-\d\.eE]+)", msg, float),
                "linear_z": extract(r"linear=Vector3\(x=[-\d\.eE]+, y=[-\d\.eE]+, z=([-\d\.eE]+)\)", msg, float),
                "angular_x": extract(r"angular=Vector3\(x=([-\d\.eE]+)", msg, float),
                "angular_y": extract(r"angular=Vector3\(x=[-\d\.eE]+, y=([-\d\.eE]+)", msg, float),
                "angular_z": extract(r"angular=Vector3\(x=[-\d\.eE]+, y=[-\d\.eE]+, z=([-\d\.eE]+)\)", msg, float),
            })

        elif topic == "/gps/diagnostics":
            diag_rows.append({
                "log_time_ns": log_time_ns,
                "stamp_sec": extract(r"stamp=Time\(sec=(\d+)", msg, int),
                "stamp_nanosec": extract(r"stamp=Time\(sec=\d+, nanosec=(\d+)\)", msg, int),
                "level": extract(r"DiagnosticStatus\(level=(\d+)", msg, int),
                "name": extract(r"name=([^,]+)", msg),
                "diag_message": extract(r"message=([^,]+)", msg),
                "i2c_addr": extract(r"KeyValue\(key=i2c_addr, value=([^\)]+)\)", msg),
                "avail_bytes_last_poll": extract(r"KeyValue\(key=avail_bytes_last_poll, value=([^\)]+)\)", msg),
                "age_s_since_last_pvt": extract(r"KeyValue\(key=age_s_since_last_pvt, value=([^\)]+)\)", msg, float),
            })


with open(FIX_OUT, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "log_time_ns", "stamp_sec", "stamp_nanosec", "frame_id",
        "status", "service",
        "latitude", "longitude", "altitude",
        "cov_xx", "cov_yy", "cov_zz", "covariance_type"
    ])
    writer.writeheader()
    writer.writerows(fix_rows)

with open(VEL_OUT, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "log_time_ns", "stamp_sec", "stamp_nanosec", "frame_id",
        "linear_x", "linear_y", "linear_z",
        "angular_x", "angular_y", "angular_z"
    ])
    writer.writeheader()
    writer.writerows(vel_rows)

with open(DIAG_OUT, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "log_time_ns", "stamp_sec", "stamp_nanosec",
        "level", "name", "diag_message",
        "i2c_addr", "avail_bytes_last_poll", "age_s_since_last_pvt"
    ])
    writer.writeheader()
    writer.writerows(diag_rows)

print(f"Wrote {len(fix_rows)} rows to {FIX_OUT}")
print(f"Wrote {len(vel_rows)} rows to {VEL_OUT}")
print(f"Wrote {len(diag_rows)} rows to {DIAG_OUT}")