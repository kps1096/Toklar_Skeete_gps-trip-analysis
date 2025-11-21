# File: gps_trip_analysis.py
# Author: Keren Skeete
# Description: This script analyzes GPS trip data to calculate total distance traveled, average speed, and trip duration.
import sys
import os
import simplekml

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
    Read a single GPS file and extract the track points - latitude (lat) & longitude (lon)
    Supports NMEA sentences: GPRMC and GPGGA.
    """

    points = []
    with open(filename, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("$GPRMC"):
                parts = line.strip().split(",")
                # RMC format: index 3 = latitude, 4 = N/S, 5 = longitude, 6 = E/W
                if len(parts) > 6 and parts[3] != "" and parts[5] != "":
                    lat, lon = parse_lat_lon(parts[3], parts[4], parts[5], parts[6])
                    points.append((lat, lon))

            elif line.startswith("$GPGGA"):
                parts = line.strip().split(",")
                # GGA format: index 2 = latitude, 3 = N/S, 4 = longitude, 5 = E/W
                if len(parts) > 5 and parts[2] != "" and parts[4] != "":
                    lat, lon = parse_lat_lon(parts[2], parts[3], parts[4], parts[5])
                    points.append((lat, lon))

    return points

def parse_lat_lon(nmea_lat, lat_dir, nmea_lon, lon_dir):
    """
    Convert NMEA formatted coordinates to decimal degrees.
    Example:
        Latitude: 4250.5589 N  ->  42.8426483
        Longitude: 07736.1234 W -> -77.6020567
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

def create_kml(points, output_filename="output.kml"):
    """
    Given a list of (lat, lon) points, create a KML line path.
    """

    kml = simplekml.Kml()
    linestring = kml.newlinestring(name="GPS Track")

    # Convert to (lon, lat) because KML requires (longitude, latitude)
    linestring.coords = [(lon, lat) for lat, lon in points]

    linestring.style.linestyle.width = 3
    linestring.style.linestyle.color = simplekml.Color.red

    kml.save(output_filename)
    print(f"[✓] KML created: {output_filename}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python gps_to_kml.py <gps files or folder>")
        sys.exit(1)

    files = gather_all_files(sys.argv[1:])

    if not files:
        print("No valid GPS files found.")
        sys.exit(1)

    all_points = []

    for gps_file in files:
        print(f"[Reading] {gps_file}")
        pts = extract_points_from_file(gps_file)
        all_points.extend(pts)

    if not all_points:
        print("No GPS coordinates found in files.")
        sys.exit(1)

    create_kml(all_points, "gps_track_output.kml")


if __name__ == "__main__":
    main()
