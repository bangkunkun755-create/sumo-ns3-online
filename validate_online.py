"""Independently reconcile live SUMO states, socket requests, and UDP events."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def csv_rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def json_lines(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def close(a, b, message, tolerance=1e-8):
    require(math.isfinite(float(a)) and math.isfinite(float(b))
            and math.isclose(float(a), float(b), rel_tol=0, abs_tol=tolerance), message)


def resolve_network(metadata, directory):
    """Validate a moved run against its hash-checked, local network snapshot."""
    directory = Path(directory).resolve()
    if "network_snapshot" in metadata:
        path = (directory / metadata["network_snapshot"]).resolve()
        require(path.is_relative_to(directory), "Network snapshot must stay inside the run directory")
        require(path.is_file(), "Network snapshot is missing")
        return path
    recorded = Path(metadata["network"])
    return recorded if recorded.is_absolute() else directory / recorded


def validate(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "sumo_metadata.json").read_text())
    result = json.loads((directory / "results.json").read_text())
    bridge = json.loads((directory / "bridge_result.json").read_text())
    trajectory = csv_rows(directory / "sumo_trajectory.csv")
    synced = csv_rows(directory / "ns3_sync.csv")
    feedback = csv_rows(directory / "network_feedback.csv")
    exchanges = json_lines(directory / "socket_exchange.jsonl")
    events = json_lines(directory / "packets.jsonl")
    summary = csv_rows(directory / "packet_summary.csv")
    duration = metadata["duration_s"]
    nearest = metadata.get("association_mode") == "nearest"
    rsu_ids = tuple(range(1, len(result["rsus"]) + 1))
    require(2 <= len(rsu_ids) <= 32 and [s["rsu_id"] for s in result["rsus"]] == list(rsu_ids),
            "Invalid RSU layout identities")
    count = int(duration) + 1
    require(metadata["mode"] == "online" and bridge["completed"], "Bridge did not complete online")
    require(len(trajectory) == len(synced) == len(feedback) == len(exchanges)
            == bridge["updates"] == result["sync_updates"] == count, "Missing synchronization step")
    require(hashlib.sha256(resolve_network(metadata, directory).read_bytes()).hexdigest()
            == metadata["network_sha256"], "Original QuickStart network has changed")
    require(result["duplicates"] == result["malformed"] == 0, "Malformed or duplicate UDP packets")
    close(result["duration_s"], duration, "Duration differs")
    tx = {}
    rx = {}
    for row in events:
        seq = row["seq"]
        require(row["bytes"] == 256 and row["rsu_id"] in rsu_ids, "Invalid UDP payload or RSU")
        if row["event"] == "tx":
            require(seq not in tx, "Duplicate transmission sequence")
            require(row["arrive_time"] is None and row["delay_ms"] is None, "TX must not invent arrival")
            require(row["socket_accepted"] is True, "UDP send failed")
            tx[seq] = row
        elif row["event"] == "rx":
            require(seq in tx and seq not in rx and row["unique"] is True, "Unknown/duplicate reception")
            sent = tx[seq]
            require(row["rsu_id"] == sent["rsu_id"], "Packet delivered to the wrong RSU")
            close(row["send_time"], sent["send_time"], "Packet send timestamp mismatch")
            require(row["send_time"] <= row["arrive_time"] <= duration, "Invalid arrival time")
            close(row["delay_ms"], (row["arrive_time"] - row["send_time"]) * 1000,
                  "Arrival time and delay disagree", 1e-7)
            rx[seq] = row
        else:
            raise ValueError("Unknown packet event")
    expected = []
    index = 0
    while 1 + index * result["interval_s"] < duration - 1 - 1e-9:
        when = 1 + index * result["interval_s"]
        if nearest:
            expected.append((index, int(trajectory[int(math.floor(when + 1e-9))]["serving_rsu"]), when))
        else:
            for rsu in rsu_ids:
                expected.append((len(rsu_ids) * index + rsu - 1, rsu, when))
        index += 1
    require(set(tx) == {row[0] for row in expected}, "UDP send schedule is incomplete")
    for seq, rsu, time in expected:
        require(tx[seq]["rsu_id"] == rsu, "Incorrect destination schedule")
        close(tx[seq]["send_time"], time, "Incorrect send time")
    require(len(summary) == len(tx), "Packet summary length differs")
    summary_by_seq = {int(row["seq"]): row for row in summary}
    require(set(summary_by_seq) == set(tx), "Packet summary sequence mismatch")
    max_packet_distance_error = 0.0
    for seq, sent in tx.items():
        row = summary_by_seq[seq]
        require(int(row["rsu_id"]) == sent["rsu_id"] and row["socket_accepted"] == "1",
                "Summary destination or send status differs")
        close(row["send_time"], sent["send_time"], "Summary send time differs")
        close(row["distance_m"], sent["distance_m"], "Summary distance differs")
        require(row["received"] == str(int(seq in rx)), "Summary delivery status differs")
        if seq in rx:
            close(row["arrive_time"], rx[seq]["arrive_time"], "Summary arrival time differs")
            close(row["delay_ms"], rx[seq]["delay_ms"], "Summary delay differs", 1e-7)
        else:
            require(row["arrive_time"] == row["delay_ms"] == "", "Missing reception must have blank arrival")
        # Independently verify the geometry used at every UDP transmission,
        # including fractional times between the one-second SUMO barriers.
        second = int(math.floor(sent["send_time"] + 1e-9))
        state = trajectory[second]
        delta = sent["send_time"] - second
        x = float(state["x_m"]) + delta * float(state["vx_mps"])
        y = float(state["y_m"]) + delta * float(state["vy_mps"])
        station = result["rsus"][sent["rsu_id"] - 1]
        expected_distance = math.sqrt((x - station["x_m"]) ** 2 + (y - station["y_m"]) ** 2
                                      + (float(state["z_m"]) - station["z_m"]) ** 2)
        close(sent["distance_m"], expected_distance, "UDP used incorrect live mobility", 1e-7)
        max_packet_distance_error = max(max_packet_distance_error, abs(sent["distance_m"] - expected_distance))
    max_position_error = 0.0
    max_clock_error = 0.0
    max_speed_error = 0.0
    max_correction = 0.0
    for seq, (truth, actual, returned, exchange) in enumerate(zip(trajectory, synced, feedback, exchanges)):
        require(int(truth["seq"]) == int(actual["seq"]) == int(returned["seq"]) == seq,
                "Non-monotonic step sequence")
        require(truth["vehicle_id"] == "online_car" and int(truth["vehicle_count"]) == 1,
                "Single-vehicle requirement violated")
        for row in (truth, actual, returned):
            close(row["ns3_time_s"], seq, "Synchronization time mismatch")
        close(truth["sumo_time_s"], float(truth["time_offset_s"]) + seq, "SUMO clock drift")
        close(actual["sumo_time_s"], truth["sumo_time_s"], "Bridge SUMO timestamp differs")
        close(actual["time_offset_s"], metadata["time_offset_s"], "Time origin differs")
        request = exchange["request"].split()
        require(request[:2] == ["STEP", str(seq)], "Missing ns-3-driven request")
        close(request[2], seq, "Socket request time differs")
        state = exchange["state"]
        for field in ("x_m", "y_m", "speed_mps", "sumo_time_s", "ns3_time_s"):
            close(state[field], truth[field], "Socket state differs from TraCI trajectory")
        for axis in ("x", "y"):
            close(actual[f"sumo_{axis}_m"], truth[f"{axis}_m"], "SUMO CSV vs bridge mismatch")
        position_error = math.hypot(float(actual["ns3_x_m"]) - float(truth["x_m"]),
                                    float(actual["ns3_y_m"]) - float(truth["y_m"]))
        clock_error = abs(float(truth["sumo_time_s"]) - metadata["time_offset_s"] - seq)
        speed_error = abs(float(truth["speed_mps"]) - metadata["speed_mps"])
        close(position_error, actual["position_error_m"], "Position error statistic differs")
        close(clock_error, actual["clock_error_s"], "Clock error statistic differs")
        require(position_error <= 1e-8 and clock_error <= 1e-8 and speed_error <= 1e-6,
                "Live position/time/speed acceptance failed")
        max_position_error = max(max_position_error, position_error)
        max_clock_error = max(max_clock_error, clock_error)
        max_speed_error = max(max_speed_error, speed_error)
        if seq:
            previous = trajectory[seq - 1]
            predicted_x = float(previous["x_m"]) + float(previous["vx_mps"])
            predicted_y = float(previous["y_m"]) + float(previous["vy_mps"])
            correction = math.hypot(predicted_x - float(truth["x_m"]), predicted_y - float(truth["y_m"]))
            close(correction, actual["prediction_correction_m"], "Between-step prediction correction differs")
            max_correction = max(max_correction, correction)
        for rsu in rsu_ids:
            station = result["rsus"][rsu - 1]
            require(station["rsu_id"] == rsu, "RSU index mismatch")
            distance = math.sqrt((float(truth["x_m"]) - station["x_m"]) ** 2
                                 + (float(truth["y_m"]) - station["y_m"]) ** 2
                                 + (float(truth["z_m"]) - station["z_m"]) ** 2)
            close(distance, actual[f"distance_rsu{rsu}_m"], "RSU distance differs")
            # All synchronization callbacks are enqueued before equal-time TX callbacks.
            sent_count = sum(row["rsu_id"] == rsu and row["send_time"] < seq - 1e-9 for row in tx.values())
            received_count = sum(row["rsu_id"] == rsu and row["arrive_time"] < seq - 1e-9 for row in rx.values())
            for kind, expected_count in (("tx", sent_count), ("rx", received_count)):
                key = f"rsu{rsu}_{kind}"
                require(int(returned[key]) == int(actual[key]) == expected_count, "Network feedback does not match events")
                request_index = 3 + (rsu - 1) * 2 + (kind == "rx")
                require(int(request[request_index]) == expected_count, "Socket feedback counter differs")
    for station in result["rsus"]:
        rsu = station["rsu_id"]
        sent = [row for row in tx.values() if row["rsu_id"] == rsu]
        arrived = [row for row in rx.values() if row["rsu_id"] == rsu]
        if not nearest:
            require(arrived, f"RSU {rsu} received no UDP packets; A2.2 cannot be accepted")
        require(len(sent) == station["generated_packets"] == station["socket_accepted_packets"], "TX count differs")
        require(len(arrived) == station["received_packets"], "RX count differs")
        if sent:
            close(station["pdr"], len(arrived) / len(sent), "PDR differs")
        else:
            require(station["pdr"] is None, "Unused RSU must have undefined PDR")
        if arrived:
            close(station["mean_delay_ms"], sum(row["delay_ms"] for row in arrived) / len(arrived),
                  "Mean delay differs", 1e-7)
        else:
            require(station["mean_delay_ms"] is None, "No RX: delay must be undefined")
    close(result["max_prediction_correction_m"], max_correction, "Correction summary differs")
    close(result["max_position_error_m"], max_position_error, "Position summary differs")
    close(result["max_clock_error_s"], max_clock_error, "Clock summary differs")
    if nearest:
        require(rx, "Nearest-RSU experiment received no packets")
    return {"passed": True, "sync_updates": count, "max_position_error_m": max_position_error,
            "max_clock_error_s": max_clock_error, "max_speed_error_mps": max_speed_error,
            "max_between_step_prediction_correction_m": max_correction,
            "max_packet_distance_error_m": max_packet_distance_error,
            "generated_packets": len(tx), "received_packets": len(rx),
            "network_feedback_verified": True, "network_unchanged": True,
            "acceptance": {"A2.1_single_vehicle_csv": True, "A2.2_two_rsus_packet_times": True,
                           "A2.3_live_socket_exchange": True, "A2.4_ns3_driven_1s_sync": True}}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    print(json.dumps(validate(parser.parse_args().directory), indent=2))
