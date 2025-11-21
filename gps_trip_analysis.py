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



