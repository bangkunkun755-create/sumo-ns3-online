"""Independently check nearest geometry, C++ decisions, switch events, and packet targets."""

import argparse
import json
import math
from pathlib import Path

from rsu_layout import read_layout
from validate_online import close, csv_rows, json_lines, require, validate


def validate_handover(directory):
    directory = Path(directory)
    report = validate(directory)
    metadata = json.loads((directory / "sumo_metadata.json").read_text())
    result = json.loads((directory / "results.json").read_text())
    require(metadata["association_mode"] == result["association_mode"] == "nearest", "Not nearest mode")
    layout = read_layout(directory / "rsu_layout.csv")
    require(layout == metadata["rsus"], "RSU layout differs from bridge metadata")
    for given, actual in zip(layout, result["rsus"]):
        for axis in ("x_m", "y_m", "z_m"):
            close(given[axis], actual[axis], "ns-3 RSU coordinate differs from layout")
    trajectory = csv_rows(directory / "sumo_trajectory.csv")
    synced = csv_rows(directory / "ns3_sync.csv")
    selections = csv_rows(directory / "association.csv")
    switches = json_lines(directory / "handover_events.jsonl")
    require(len(selections) == len(trajectory), "Missing C++ association decision")
    expected_switches = []
    previous = 0
    for index, (truth, actual, row) in enumerate(zip(trajectory, synced, selections)):
        distances = [math.sqrt(sum((float(truth[axis]) - station[axis]) ** 2
                                  for axis in ("x_m", "y_m", "z_m"))) for station in layout]
        shortest = min(distances)
        target = next(i + 1 for i, distance in enumerate(distances) if distance <= shortest + 1e-9)
        require(int(truth["serving_rsu"]) == int(actual["serving_rsu"]) == int(row["to_rsu"]) == target,
                "Target is not the nearest RSU")
        require(int(row["seq"]) == index and int(row["from_rsu"]) == previous, "Invalid association order")
        close(row["ns3_time_s"], index, "Association timestamp differs")
        close(row["distance_m"], distances[target - 1], "Nearest distance differs")
        close(truth["nearest_distance_m"], distances[target - 1], "Bridge nearest distance differs")
        require(int(row["changed"]) == int(target != previous), "Association change flag differs")
        if previous != target:
            expected_switches.append((index, previous, target))
        previous = target
    require(len(expected_switches) == len(switches), "Missing handover event")
    for expected, row in zip(expected_switches, switches):
        require((row["ns3_time_s"], row["from_rsu"], row["to_rsu"]) == expected, "Wrong handover event")
        require(row["kind"] == ("initial_selection" if expected[1] == 0 else "handover"), "Initial selection is not a handover")
    require(result["handover_count"] == len(expected_switches) - 1, "Handover count differs")
    events = json_lines(directory / "packets.jsonl")
    display_path = directory / "gui_packet_events.jsonl"
    if display_path.exists():
        shown = json_lines(display_path)
        require([row["packet"] for row in shown] == events, "GUI packet events differ")
        require(all((row["packet"]["send_time"] if row["packet"]["event"] == "tx"
                     else row["packet"]["arrive_time"]) <= row["barrier_s"] + 1e-8 for row in shown),
                "GUI displayed a future packet event")
    gui_path = directory / "gui_feedback.jsonl"
    if gui_path.exists():
        gui = json_lines(gui_path)
        feedback = csv_rows(directory / "network_feedback.csv")
        require(len(gui) == len(trajectory), "GUI sync count differs")
        previous_rx = [0] * len(layout)
        for shown, state, returned in zip(gui, trajectory, feedback):
            require(shown["serving_rsu"] == int(state["serving_rsu"]), "GUI target differs")
            received = [int(returned[f"rsu{i}_rx"]) for i in range(1, len(layout) + 1)]
            delta = [now - old for now, old in zip(received, previous_rx)]
            require(shown["received"] == received and shown["newly_received"] == delta
                    and shown["link_visible"] == [value > 0 for value in delta], "GUI receipt status differs")
            previous_rx = received
    report.update(rsu_count=len(layout), handover_count=len(expected_switches) - 1,
                  serving_sequence=[row[2] for row in expected_switches],
                  unused_rsus=[row["rsu_id"] for row in result["rsus"] if not row["generated_packets"]],
                  selected_without_traffic=[row["rsu_id"] for row in result["rsus"]
                                            if not row["generated_packets"] and row["rsu_id"] in {s[2] for s in expected_switches}],
                  nearest_selection_verified=True, single_destination_schedule_verified=True,
                  handover_definition="application UDP target change, NOT L2/cellular handover")
    report["acceptance"]["A2.2_multi_rsu_packet_times"] = report["acceptance"].pop("A2.2_two_rsus_packet_times")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    print(json.dumps(validate_handover(parser.parse_args().directory), indent=2))
