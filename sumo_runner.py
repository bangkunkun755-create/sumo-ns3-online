"""One SUMO vehicle, driven by a live ns-3 TCP step barrier (or standalone)."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import socket
import sys
import xml.etree.ElementTree as ET

from gui_demo import GuiDemo, write_gui_settings
from rsu_layout import check_handover_settings, nearest_station, read_layout
from project_config import DEFAULT_NETWORK

PROJECT = Path(__file__).resolve().parent
VEHICLE = "online_car"
ROUTE = "D2 L2 L12 L10 L7 D7"


def load_traci():
    candidates = [Path(os.environ.get("SUMO_HOME", "/usr/share/sumo")) / "tools",
                  Path("/usr/share/sumo/tools")]
    for candidate in candidates:
        if candidate.is_dir() and str(candidate) not in sys.path:
            sys.path.append(str(candidate))
    import traci
    return traci


def check_settings(duration, speed):
    if (not math.isfinite(duration) or duration < 5 or duration > 120
            or not duration.is_integer()):
        raise ValueError("duration must be an integer from 5 to 120 seconds")
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("speed must be in (0, 10] m/s for this QuickStart route")


def prepare_scenario(directory, network, duration, speed):
    if not network.is_file():
        raise FileNotFoundError(network)
    routes = ET.Element("routes")
    ET.SubElement(routes, "vType", id="constant_car", accel="3", decel="6",
                  sigma="0", length="5", minGap="2.5", maxSpeed="10",
                  speedFactor="1", speedDev="0", lcStrategic="0",
                  lcCooperative="0", lcSpeedGain="0", lcKeepRight="0")
    ET.SubElement(routes, "route", id="straight", edges=ROUTE)
    ET.SubElement(routes, "vehicle", id=VEHICLE, type="constant_car", route="straight",
                  depart="0", departLane="0", departPos="base", departSpeed=str(speed))
    ET.indent(routes)
    ET.ElementTree(routes).write(directory / "single_car.rou.xml", encoding="utf-8",
                                xml_declaration=True)
    config = ET.Element("configuration")
    inputs = ET.SubElement(config, "input")
    ET.SubElement(inputs, "net-file", value=str(network))
    ET.SubElement(inputs, "route-files", value="single_car.rou.xml")
    time = ET.SubElement(config, "time")
    ET.SubElement(time, "begin", value="0")
    ET.SubElement(time, "end", value=str(duration + 2))
    ET.SubElement(time, "step-length", value="1")
    processing = ET.SubElement(config, "processing")
    ET.SubElement(processing, "time-to-teleport", value="-1")
    ET.indent(config)
    ET.ElementTree(config).write(directory / "single_car.sumocfg", encoding="utf-8",
                                xml_declaration=True)


def parse_step(line, expected_seq, duration, rsu_count=2):
    fields = line.split()
    if len(fields) != 3 + 2 * rsu_count or fields[0] != "STEP":
        raise ValueError("Expected STEP seq elapsed_s and one TX/RX pair per RSU")
    seq = int(fields[1])
    elapsed = float(fields[2])
    counters = [int(value) for value in fields[3:]]
    if (seq != expected_seq or not math.isfinite(elapsed)
            or abs(elapsed - seq) > 1e-9 or elapsed > duration
            or any(value < 0 for value in counters)
            or any(counters[i + 1] > counters[i] for i in range(0, len(counters), 2))):
        raise ValueError("Out-of-order step, invalid simulation time, or invalid counters")
    return seq, elapsed, counters


def state_at(traci, seq, elapsed, offset, speed):
    # ns-3 t=0 maps to the first inserted vehicle state at SUMO getTime()=1.
    # All subsequent advancement happens only in response to a STEP request.
    if seq:
        traci.vehicle.setSpeed(VEHICLE, speed)
        traci.simulationStep(offset + elapsed)
    ids = traci.vehicle.getIDList()
    if tuple(ids) != (VEHICLE,):
        raise RuntimeError(f"Expected exactly one active vehicle, got {ids}")
    sumo_time = traci.simulation.getTime()
    if abs(sumo_time - offset - elapsed) > 1e-8:
        raise RuntimeError("SUMO/ns-3 clocks diverged")
    x, y = traci.vehicle.getPosition(VEHICLE)
    actual_speed = traci.vehicle.getSpeed(VEHICLE)
    angle = traci.vehicle.getAngle(VEHICLE)
    if abs(actual_speed - speed) > 1e-6:
        raise RuntimeError(f"Constant-speed requirement failed: {actual_speed} != {speed}")
    # SUMO angles: clockwise from north. ns-3 uses Cartesian x/y velocity.
    radians = math.radians(angle)
    return {"seq": seq, "ns3_time_s": elapsed, "sumo_time_s": sumo_time,
            "time_offset_s": offset, "vehicle_id": VEHICLE, "vehicle_count": len(ids),
            "x_m": x, "y_m": y, "z_m": 1.5, "speed_mps": actual_speed,
            "vx_mps": actual_speed * math.sin(radians),
            "vy_mps": actual_speed * math.cos(radians),
            "angle_deg": angle, "edge_id": traci.vehicle.getRoadID(VEHICLE)}


def encode_state(state):
    keys = ("seq", "ns3_time_s", "sumo_time_s", "time_offset_s", "x_m", "y_m",
            "z_m", "speed_mps", "vx_mps", "vy_mps", "vehicle_count")
    if "serving_rsu" in state:
        keys += ("serving_rsu",)
    return "STATE " + " ".join(format(state[key], ".17g") for key in keys) + "\n"


def run(args):
    stations = read_layout(args.rsu_layout) if args.rsu_layout else None
    (check_handover_settings if stations else check_settings)(args.duration, args.speed)
    rsu_count = len(stations) if stations else 2
    if not math.isfinite(args.gui_delay_ms) or not 0 <= args.gui_delay_ms <= 1000:
        raise ValueError("gui-delay-ms must be from 0 to 1000")
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if (directory / "sumo_trajectory.csv").exists():
        raise FileExistsError("Refusing to overwrite an existing trajectory")
    network = args.network.resolve()
    original_hash = hashlib.sha256(network.read_bytes()).hexdigest()
    snapshot = directory / "network_snapshot.net.xml"
    shutil.copyfile(network, snapshot)
    prepare_scenario(directory, network, args.duration, args.speed)
    binary = "sumo-gui" if args.gui else "sumo"
    sumo = shutil.which(binary)
    if not sumo:
        raise RuntimeError(f"{binary} is missing from the Linux PATH")
    traci = load_traci()
    command = [sumo, "-c", str(directory / "single_car.sumocfg"), "--seed", "23423",
               "--no-step-log", "true", "--duration-log.disable", "true",
               "--log", str(directory / "sumo.log")]
    if args.gui:
        if not os.environ.get("DISPLAY"):
            raise RuntimeError("SUMO GUI needs WSLg DISPLAY; run from the Ubuntu WSL interpreter")
        # Scope the WSL/OpenGL compatibility workaround to this bridge and its child process.
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
        os.environ["GALLIUM_DRIVER"] = "llvmpipe"
        settings = directory / "gui.view.xml"
        write_gui_settings(settings)
        command += ["--start", "--quit-on-end", "--delay", "0", "--window-size", "1280,800",
                    "--gui-settings-file", str(settings)]
    started = False
    gui = None
    try:
        traci.start(command)
        started = True
        traci.simulationStep()  # Insert the vehicle. This one startup step precedes ns-3 t=0.
        traci.vehicle.setSpeed(VEHICLE, args.speed)
        traci.vehicle.setLaneChangeMode(VEHICLE, 0)
        offset = traci.simulation.getTime()
        initial = state_at(traci, 0, 0, offset, args.speed)
        if stations:
            initial["serving_rsu"], initial["nearest_distance_m"] = nearest_station(initial, stations)
        if args.gui:
            if stations:
                from handover_gui import HandoverGui
                gui = HandoverGui(traci, initial, stations, args.duration, args.gui_delay_ms,
                                  directory, overview=args.gui_overview,
                                  packet_demo=args.gui_packets and not args.standalone)
            else:
                gui = GuiDemo(traci, initial, (args.rsu1_offset, args.rsu2_offset),
                              args.duration, args.gui_delay_ms, directory, overview=args.gui_overview,
                              packet_demo=args.gui_packets and not args.standalone)
        metadata = {"protocol_version": 1, "mode": "standalone" if args.standalone else "online",
                    "sumo_version": traci.getVersion(), "traci_module": traci.__file__,
                    "sumo_command": command, "network": str(network), "network_snapshot": snapshot.name,
                    "network_sha256": original_hash, "vehicle_id": VEHICLE,
                    "route_edges": ROUTE.split(), "duration_s": args.duration,
                    "speed_mps": args.speed, "step_s": 1, "time_offset_s": offset,
                    "time_definition": "ns3_time_s = TraCI simulation.getTime() - time_offset_s",
                    "initial_state": initial}
        metadata["gui"] = {"enabled": args.gui, "delay_ms": args.gui_delay_ms if args.gui else 0,
                           "overview": args.gui_overview,
                           "packet_animation": args.gui_packets and not args.standalone,
                           "packet_animation_definition": "previous-interval real TX/RX replay; dots are schematic, not RF paths or delay scale",
                           "rsu_offsets_m": [args.rsu1_offset, args.rsu2_offset],
                           "colors": "green means received since previous sync; not instantaneous RF state"}
        if stations:
            metadata.update(protocol_version=2, association_mode="nearest", rsus=stations,
                            selection_step_s=1, selection_distance="3D Euclidean, lower-ID tie break",
                            handover_definition="instant application UDP destination selection, NOT L2/cellular handover")
        (directory / "sumo_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        with (directory / "sumo_trajectory.csv").open("w", newline="") as trajectory, \
                (directory / "network_feedback.csv").open("w", newline="") as feedback, \
                (directory / "socket_exchange.jsonl").open("w") as exchanges:
            writer = csv.DictWriter(trajectory, fieldnames=list(initial))
            writer.writeheader()
            feedback_writer = csv.writer(feedback)
            feedback_writer.writerow(["seq", "ns3_time_s", *[f"rsu{i}_{kind}"
                                      for i in range(1, rsu_count + 1) for kind in ("tx", "rx")]])
            previous_serving = 0
            handovers = []

            def respond(seq, elapsed, counters):
                nonlocal previous_serving
                state = state_at(traci, seq, elapsed, offset, args.speed)
                if stations:
                    state["serving_rsu"], state["nearest_distance_m"] = nearest_station(state, stations)
                    if state["serving_rsu"] != previous_serving:
                        handovers.append(dict(ns3_time_s=elapsed, from_rsu=previous_serving,
                                              to_rsu=state["serving_rsu"], x_m=state["x_m"],
                                              y_m=state["y_m"], distance_m=state["nearest_distance_m"],
                                              kind="handover" if previous_serving else "initial_selection"))
                        # Dedicated new-run file; never overwrite previous experiment outputs.
                        (directory / "handover_events.jsonl").write_text(
                            "".join(json.dumps(row) + "\n" for row in handovers))
                        previous_serving = state["serving_rsu"]
                if gui is not None:
                    gui.update(state, counters)
                writer.writerow(state)
                trajectory.flush()
                feedback_writer.writerow([seq, elapsed, *counters])
                feedback.flush()
                return state

            if args.standalone:
                for seq in range(int(args.duration) + 1):
                    respond(seq, float(seq), [0] * (2 * rsu_count))
            else:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    listener.bind(("127.0.0.1", args.port))
                    listener.listen(1)
                    listener.settimeout(60)
                    ready = {"host": "127.0.0.1", "port": listener.getsockname()[1],
                             "protocol_version": 2 if stations else 1, "pid": os.getpid()}
                    temporary = directory / "ready.tmp"
                    temporary.write_text(json.dumps(ready) + "\n")
                    temporary.replace(directory / "ready.json")
                    print(f"SUMO bridge ready on 127.0.0.1:{ready['port']}", flush=True)
                    connection, _ = listener.accept()
                    with connection:
                        connection.settimeout(30)
                        with connection.makefile("rwb") as stream:
                            for expected in range(int(args.duration) + 1):
                                raw = stream.readline(4097)
                                if not raw or len(raw) > 4096 or not raw.endswith(b"\n"):
                                    raise RuntimeError("Socket disconnected or invalid frame")
                                request = raw.decode("ascii").strip()
                                seq, elapsed, counters = parse_step(request, expected, args.duration, rsu_count)
                                state = respond(seq, elapsed, counters)
                                response = encode_state(state)
                                exchanges.write(json.dumps({"request": request, "state": state}) + "\n")
                                exchanges.flush()
                                stream.write(response.encode("ascii"))
                                stream.flush()
                            if stream.readline(4097) != b"CLOSE\n":
                                raise RuntimeError("Expected CLOSE after final synchronized state")
                            stream.write(b"DONE\n")
                            stream.flush()
        if hashlib.sha256(network.read_bytes()).hexdigest() != original_hash:
            raise RuntimeError("Source network changed during the experiment")
        (directory / "bridge_result.json").write_text(json.dumps({
            "completed": True, "updates": int(args.duration) + 1,
            "time_offset_s": offset, "network_unchanged": True}, indent=2) + "\n")
        print(f"SUMO completed: {int(args.duration) + 1} synchronized states", flush=True)
    finally:
        if gui is not None:
            gui.close()
        if started:
            traci.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=DEFAULT_NETWORK)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=100.0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--standalone", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--gui-delay-ms", type=float, default=250)
    parser.add_argument("--gui-overview", action="store_true")
    parser.add_argument("--gui-packets", action="store_true")
    parser.add_argument("--rsu1-offset", type=float, default=250)
    parser.add_argument("--rsu2-offset", type=float, default=750)
    parser.add_argument("--rsu-layout", type=Path,
                        help="Enable multi-RSU nearest-target mode using this CSV layout")
    args = parser.parse_args()
    try:
        run(args)
    except Exception as error:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "bridge_error.json").write_text(json.dumps({"error": str(error)}) + "\n")
        raise


if __name__ == "__main__":
    main()
