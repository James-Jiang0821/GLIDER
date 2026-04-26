import pandas as pd
import matplotlib.pyplot as plt

#Load CSV
df = pd.read_csv("iridium_signal_strength.csv")

#Clean column names
df.columns = df.columns.str.strip()

#Convert signal strength to numeric
df["signal_strength"] = pd.to_numeric(df["signal_strength"], errors="coerce")

#Drop bad rows
df = df.dropna(subset=["signal_strength"])

#ns to seconds from start
df["time_s"] = (df["log_time_ns"] - df["log_time_ns"].iloc[0]) / 1e9

#Plot
plt.figure()
plt.plot(df["time_s"], df["signal_strength"], marker="o")
plt.xlabel("Time (s)")
plt.ylabel("CSQ")
plt.title("Iridium Signal Strength Over Time")
plt.grid(True)
plt.ylim(bottom=0)

plt.show()
