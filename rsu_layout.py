"""QuickStart roadside coordinates and deterministic nearest-RSU policy."""

import csv
import math
import xml.etree.ElementTree as ET


ROAD_SITES = (("D2", 2), ("L2", 2), ("L12", 4), ("L10", 4), ("L7", 2), ("D7", 2),
              ("L4", 1), ("L14", 2), ("L18", 2), ("L5", 1), ("L15", 2))


def point_along(shape, fraction, roadside=10):
    lengths = [math.dist(a, b) for a, b in zip(shape, shape[1:])]
    total = sum(lengths)
    if total <= 0 or not 0 <= fraction <= 1:
        raise ValueError("Invalid road polyline or position")
    remaining = fraction * total
    for a, b, length in zip(shape, shape[1:], lengths):
        if length <= 0:
            continue
        if remaining <= length + 1e-9:
            ratio = min(1, remaining / length)
            dx, dy = b[0] - a[0], b[1] - a[1]
            return (a[0] + ratio * dx - roadside * dy / length,
                    a[1] + ratio * dy + roadside * dx / length)
        remaining -= length
    raise ValueError("Road position outside polyline")


def build_layout(network):
    edges = {edge.attrib["id"]: edge for edge in ET.parse(network).getroot().findall("edge")}
    stations = []
    for edge_id, count in ROAD_SITES:
        if edge_id not in edges:
            raise ValueError(f"QuickStart layout requires road {edge_id}")
        lane = edges[edge_id].find("lane")
        shape = [tuple(map(float, p.split(","))) for p in lane.attrib["shape"].split()]
        for index in range(count):
            fraction = (index + 0.5) / count
            x, y = point_along(shape, fraction)
            stations.append(dict(rsu_id=len(stations) + 1, x_m=x, y_m=y, z_m=3.0,
                                 edge_id=edge_id, lane_position_m=float(lane.attrib["length"]) * fraction))
    return stations


def write_layout(path, stations):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(stations[0]))
        writer.writeheader()
        writer.writerows(stations)


def read_layout(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    stations = [dict(rsu_id=int(row["rsu_id"]),
                     **{axis: float(row[axis]) for axis in ("x_m", "y_m", "z_m")},
                     edge_id=row["edge_id"], lane_position_m=float(row["lane_position_m"])) for row in rows]
    if not 2 <= len(stations) <= 32:
        raise ValueError("RSU count must be 2..32")
    for index, row in enumerate(stations, 1):
        if row["rsu_id"] != index or not all(math.isfinite(row[key]) for key in ("x_m", "y_m", "z_m")):
            raise ValueError("Invalid RSU identifiers or coordinates")
    if len({(s["x_m"], s["y_m"], s["z_m"]) for s in stations}) != len(stations):
        raise ValueError("Duplicate RSU locations")
    return stations


def nearest_station(state, stations):
    best, distance = None, math.inf
    for station in stations:
        candidate = math.sqrt(sum((state[key] - station[key]) ** 2 for key in ("x_m", "y_m", "z_m")))
        if candidate < distance - 1e-9:  # Stable ascending-ID tie break.
            best, distance = station["rsu_id"], candidate
    if best is None:
        raise ValueError("No candidate RSUs")
    return best, distance


def check_handover_settings(duration, speed):
    if not math.isfinite(duration) or not 5 <= duration <= 390 or not duration.is_integer():
        raise ValueError("Handover duration must be an integer from 5 to 390 seconds")
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("Handover speed must be in (0,10] m/s")
