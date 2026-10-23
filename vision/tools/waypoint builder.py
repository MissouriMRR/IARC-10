import math
import json


def main():
	# Starting corner coordinates (north‑west of the square)
	lat0 = 37.9485445
	lon0 = -91.7859372

	# Convert meters to degrees (approximate)
	lat_per_meter = 1 / 111111  # degrees latitude per meter
	lon_per_meter = 1 / (111111 * math.cos(math.radians(lat0)))  # degrees longitude per meter

	# Center of the first 4x4m cell (top‑left)
	start_lat = lat0 - 2 * lat_per_meter  # move 2m south
	start_lon = lon0 + 2 * lon_per_meter  # move 2m east

	waypoints = []
	count = 1
	# 5x5 grid of 4x4m cells covering a 20x20m square
	for i in range(5):  # rows (southward)
		row_lat = start_lat - i * 4 * lat_per_meter
		cols = range(5) if i % 2 == 0 else range(4, -1, -1)  # zig‑zag
		for j in cols:  # columns (eastward)
			col_lon = start_lon + j * 4 * lon_per_meter
			waypoints.append({
				"name": f"Waypoint {count} S1",
				"latitude": round(row_lat, 8),
				"longitude": round(col_lon, 8)
			})
			count += 1

	data = {"Set 1": waypoints}
	print(json.dumps(data, indent=4))


if __name__ == "__main__":
	main()
