# File: gps_trip_analysis.py
# Author: Keren Skeete and Lara Toklar
# Description: This script analyzes GPS trip data to calculate total distance traveled, average speed, and trip duration.
import sys
import os
import simplekml  # type: ignore
from math import radians, sin, cos, sqrt, atan2, degrees

###################################
# From instructions:
# 1: Write a program to read in the GPS data, parse the NEMA GPS lines, and emit a KML file.
###################################
def gather_all_files(args):
    """
    Takes command line arguments and returns a list of all GPS file paths.
    If a folder is passed, returns all .txt or .gps files inside it.
    """
    files = []

    for path in args:
        if os.path.isfile(path):
            files.append(path)
        elif os.path.isdir(path):
            for fname in os.listdir(path):
                if fname.lower().endswith((".txt", ".gps", ".nmea")):
                    files.append(os.path.join(path, fname))
        else:
            print(f"Warning: '{path}' does not exist. Skipped.")

    return files


def extract_points_from_file(filename):
    """
    Read a single GPS file and extract:
        • (lat, lon)
        • stop points (speed ≈ 0)
        • left-turn points (heading increases > 45°)

    Returns:
        points, stop_points, left_turn_points
    """

    points = []
    stop_points = []
    left_turn_points = []
    last_heading = None  # Used for left-turn detection

    with open(filename, "r", errors="ignore") as f:
        for line in f:

            # Detect Arduino "burp" (two sentences in one line)
            if line.count("$") > 1:
                # More than one GPS sentence → invalid line
                continue

            elif line.startswith("$GPRMC"):
                parts = line.strip().split(",")

                if len(parts) > 8 and parts[3] != "" and parts[5] != "":
                    lat, lon = parse_lat_lon(parts[3], parts[4], parts[5], parts[6])
                    points.append((lat, lon))

                    # Speed in knots
                    try:
                        speed_knots = float(parts[7]) if parts[7] else 0.0
                    except:
                        speed_knots = 0.0

                    # Heading (bearing)
                    try:
                        heading = float(parts[8]) if parts[8] else None
                    except:
                        heading = None

                    # ---------------------------
                    # A. STOP DETECTION
                    # ---------------------------
                    if speed_knots < 0.5:  # car basically stopped
                        stop_points.append((lat, lon))

                    # ---------------------------
                    # B. LEFT TURN DETECTION
                    # ---------------------------
                    if last_heading is not None and heading is not None:
                        if heading - last_heading > 45:
                            left_turn_points.append((lat, lon))

                    last_heading = heading

            elif line.startswith("$GPGGA"):
                parts = line.strip().split(",")
                if len(parts) > 5 and parts[2] != "" and parts[4] != "":
                    lat, lon = parse_lat_lon(parts[2], parts[3], parts[4], parts[5])
                    points.append((lat, lon))

    return points, stop_points, left_turn_points


def parse_lat_lon(nmea_lat, lat_dir, nmea_lon, lon_dir):
    """
    Convert NMEA formatted coordinates to decimal degrees.
    """

    # NMEA: DDMM.MMMM
    lat_deg = int(float(nmea_lat) / 100)
    lat_min = float(nmea_lat) - lat_deg * 100
    lat = lat_deg + lat_min / 60.0
    if lat_dir == 'S':
        lat = -lat

    # NMEA: DDDMM.MMMM
    lon_deg = int(float(nmea_lon) / 100)
    lon_min = float(nmea_lon) - lon_deg * 100
    lon = lon_deg + lon_min / 60.0
    if lon_dir == 'W':
        lon = -lon

    return lat, lon

###################################
# From instructions:
# Decorate the KML file with markers for:
#   A. A yellow line along the route of travel.
#   B. Do not worry about the altitude. You can set that a 3 meters or something fixed.
#   C. A red marker if the car stopped for a stop sign or traffic light.
#   D. A yellow marker if the car made a left turn.
###################################
def create_kml(points, stop_points, left_turn_points, output_filename="output.kml"):
    """
    Create a decorated KML:
        • Yellow route line
        • Altitude = 3 meters
        • Red markers for stops
        • Yellow markers for left turns
    """

    kml = simplekml.Kml()

    # A
    line = kml.newlinestring(name="Route")
    line.coords = [(lon, lat, 3) for (lat, lon) in points]  # altitude = 3 meters
    line.style.linestyle.color = simplekml.Color.yellow
    line.style.linestyle.width = 4
    line.altitudemode = simplekml.AltitudeMode.relativetoground

    # C
    for (lat, lon) in stop_points:
        p = kml.newpoint(coords=[(lon, lat, 3)])
        p.name = "Stop"
        p.snippet = simplekml.Snippet("")  # prevents duplicate label in Google Earth
        p.style.iconstyle.color = simplekml.Color.red
        p.style.iconstyle.scale = 1.2

    # D
    for (lat, lon) in left_turn_points:
        p = kml.newpoint(coords=[(lon, lat, 3)])
        p.name = "Left Turn"
        p.snippet = simplekml.Snippet("")  # prevents duplicate label
        p.style.iconstyle.color = simplekml.Color.yellow
        p.style.iconstyle.scale = 1.2


    kml.save(output_filename)
    print(f"KML created. File: {output_filename}")

def haversine(lat1, lon1, lat2, lon2):
    """Return distance in meters between two lat/lon points."""
    R = 6371000  # Earth radius in meters
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

###################################
# Clean duplicate stop points
###################################
def clean_gps_data(stop_points, cluster_radius_m=5.0):
    """
    Take a list of stop points [(lat, lon), ...] and
    merge consecutive points that are within cluster_radius_m
    of each other into a single averaged stop point.

    Result: one stop marker per physical stop.
    """
    if not stop_points:
        return []

    cleaned = []

    # start first cluster
    current_cluster = [stop_points[0]]
    cluster_lat, cluster_lon = stop_points[0]

    for lat, lon in stop_points[1:]:
        dist = haversine(cluster_lat, cluster_lon, lat, lon)

        if dist <= cluster_radius_m:
            # same stop → add to current cluster and update cluster center
            current_cluster.append((lat, lon))
            cluster_lat = sum(p[0] for p in current_cluster) / len(current_cluster)
            cluster_lon = sum(p[1] for p in current_cluster) / len(current_cluster)
        else:
            # new stop → finish old cluster
            avg_lat = sum(p[0] for p in current_cluster) / len(current_cluster)
            avg_lon = sum(p[1] for p in current_cluster) / len(current_cluster)
            cleaned.append((avg_lat, avg_lon))

            # start new cluster
            current_cluster = [(lat, lon)]
            cluster_lat, cluster_lon = lat, lon

    # flush last cluster
    avg_lat = sum(p[0] for p in current_cluster) / len(current_cluster)
    avg_lon = sum(p[1] for p in current_cluster) / len(current_cluster)
    cleaned.append((avg_lat, avg_lon))

    return cleaned

def remove_beginning_stop_points(points, min_move_m=1.0):
    """Remove points at the beginning until movement is detected."""
    if len(points) < 2:
        return points

    start_index = 0
    for i in range(1, len(points)):
        dist = haversine(points[i-1][0], points[i-1][1],
                         points[i][0], points[i][1])
        if dist >= min_move_m:
            start_index = i - 1
            break

    return points[start_index:]

def remove_ending_stop_points(points, min_move_m=1.0):
    """Remove points at the end after movement stops."""
    if len(points) < 2:
        return points

    end_index = len(points) - 1
    for i in range(len(points)-1, 0, -1):
        dist = haversine(points[i][0], points[i][1],
                         points[i-1][0], points[i-1][1])
        if dist >= min_move_m:
            end_index = i
            break

    return points[:end_index+1]


def main():
    if len(sys.argv) < 2:
        print("Usage: python gps_to_kml.py <gps files or folder>")
        sys.exit(1)

    files = gather_all_files(sys.argv[1:])

    if not files:
        print("No valid GPS files found.")
        sys.exit(1)

    all_points = []
    all_stops = []
    all_turns = []

    for gps_file in files:
        print(f"[Reading] {gps_file}")
        pts, stops, turns = extract_points_from_file(gps_file)
        all_points.extend(pts)
        all_stops.extend(stops)
        all_turns.extend(turns)

    if not all_points:
        print("No GPS coordinates found in files.")
        sys.exit(1)


    all_points = remove_beginning_stop_points(all_points)
    all_points = remove_ending_stop_points(all_points)

    # ---------- NEW: CLEANING STEP ----------
    print(f"Raw points: {len(all_points)}")
    all_points = clean_gps_data(all_points)
    all_stops  = clean_gps_data(all_stops)
    all_turns  = clean_gps_data(all_turns)
    print(f"Cleaned points: {len(all_points)}")
    # ----------------------------------------

    create_kml(all_points, all_stops, all_turns, "gps_output.kml")


if __name__ == "__main__":
    main()
