import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("gps_fix_clean.csv")

#ensure numeric types
df["latitude"] = pd.to_numeric(df["latitude"], errors="coerce")
df["longitude"] = pd.to_numeric(df["longitude"], errors="coerce")

#drop invalid GPS rows
df = df.dropna(subset=["latitude", "longitude"]).copy()
df = df[
    (df["latitude"] >= -90) & (df["latitude"] <= 90) &
    (df["longitude"] >= -180) & (df["longitude"] <= 180)
].copy()

#filter to local test area
df = df[
    (df["latitude"] > 51.0) & (df["latitude"] < 52.0) &
    (df["longitude"] > -1.0) & (df["longitude"] < 0.5)
].copy()

df = df.reset_index(drop=True)

lat = df["latitude"].to_numpy()
lon = df["longitude"].to_numpy()

lat0 = lat[0]
lon0 = lon[0]

meters_per_deg_lat = 111320.0
meters_per_deg_lon = 111320.0 * np.cos(np.radians(lat0))

x = (lon - lon0) * meters_per_deg_lon
y = (lat - lat0) * meters_per_deg_lat

print("Points plotted:", len(df))
print("East range (m):", x.min(), "to", x.max())
print("North range (m):", y.min(), "to", y.max())

plt.figure(figsize=(8, 6))
plt.plot(x, y, marker="o", linestyle="-")
plt.scatter(x[0], y[0], label="Start")
plt.scatter(x[-1], y[-1], label="End")

plt.xlabel("East (meters)")
plt.ylabel("North (meters)")
plt.title("GPS Walking Path")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()
