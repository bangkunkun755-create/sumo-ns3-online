"""24 roadside RSUs on QuickStart; one nearest application destination per second."""

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from rsu_layout import build_layout, check_handover_settings, write_layout
from run_online import DEFAULT_NS3, PROJECT, stop_process, wait_with_progress
from sumo_runner import DEFAULT_NETWORK
from validate_handover import validate_handover
from validate_online import csv_rows, resolve_network
from project_config import check_deployed_scene, ns3_run_command

TARGET = "sumo-nearest-rsu"


def plot_handover(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    trajectory = csv_rows(directory / "sumo_trajectory.csv")
    results = json.loads((directory / "results.json").read_text())
    metadata = json.loads((directory / "sumo_metadata.json").read_text())
    fig, axes = plt.subplots(3, 1, figsize=(12, 12), constrained_layout=True)
    for edge in ET.parse(resolve_network(metadata, directory)).getroot().findall("edge"):
        if edge.attrib["id"].startswith(":"):
            continue
        lane = edge.find("lane")
        shape = [tuple(map(float, p.split(","))) for p in lane.attrib["shape"].split()]
        axes[0].plot([p[0] for p in shape], [p[1] for p in shape], color="0.8", linewidth=1)
    axes[0].plot([float(r["x_m"]) for r in trajectory], [float(r["y_m"]) for r in trajectory],
                 color="tab:green", linewidth=3, label="Live SUMO path")
    for station in results["rsus"]:
        active = station["generated_packets"] > 0
        axes[0].scatter(station["x_m"], station["y_m"], marker="^", s=55,
                        color="tab:purple" if active else "tab:gray")
        axes[0].annotate(str(station["rsu_id"]), (station["x_m"], station["y_m"]),
                         xytext=(2, 5), textcoords="offset points", fontsize=8)
    axes[0].set(title="Original QuickStart roads: 24 RSUs (purple=used, gray=unused)", xlabel="x (m)", ylabel="y (m)")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].legend()
    axes[1].step([float(r["ns3_time_s"]) for r in trajectory],
                 [int(r["serving_rsu"]) for r in trajectory], where="post", color="tab:purple")
    axes[1].set(title="Nearest application destination (selected every second; NOT L2/cellular handover)",
                xlabel="ns-3 simulation time (s)", ylabel="Selected RSU ID")
    ids = [r["rsu_id"] for r in results["rsus"]]
    axes[2].bar([i - 0.2 for i in ids], [r["generated_packets"] for r in results["rsus"]], width=.4, label="TX attempts")
    axes[2].bar([i + 0.2 for i in ids], [r["received_packets"] for r in results["rsus"]], width=.4, label="Unique UDP RX")
    axes[2].set(title="Packets by destination; unused RSUs have zero traffic, not 100% loss", xlabel="RSU ID", ylabel="Packets", xticks=ids)
    axes[2].legend()
    for axis in axes:
        axis.grid(alpha=.2)
    fig.savefig(directory / "handover_overview.png", dpi=160)
    plt.close(fig)


def run(args):
    if sys.platform != "linux":
        raise RuntimeError("Use WSL Ubuntu-22.04 /usr/bin/python3")
    check_handover_settings(args.duration, args.speed)
    if not math.isfinite(args.gui_delay_ms) or not 0 <= args.gui_delay_ms <= 1000:
        raise ValueError("GUI delay must be 0..1000 milliseconds")
    if not math.isfinite(args.interval) or not .01 <= args.interval <= 1 or not math.isfinite(args.tx_power_dbm):
        raise ValueError("Invalid packet interval or transmit power")
    directory = PROJECT / "runs" / f"handover-{datetime.now():%Y%m%d-%H%M%S-%f}"
    directory.mkdir(parents=True, exist_ok=False)
    network, ns3_dir = args.network.resolve(), args.ns3_dir.resolve()
    check_deployed_scene(ns3_dir, TARGET)
    stations = build_layout(network)
    layout = directory / "rsu_layout.csv"
    write_layout(layout, stations)
    print(f"结果目录：{directory}", flush=True)
    print(f"[1/4] 原 QuickStart 路网上部署 {len(stations)} 个 RSU；编译最近节点选择场景……", flush=True)
    with (directory / "ns3_build.log").open("w") as log:
        subprocess.run([sys.executable, "ns3", "build", TARGET, "-j", "2"], cwd=ns3_dir,
                       stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
    bridge = process = None
    try:
        command = [sys.executable, str(PROJECT / "sumo_runner.py"), "--output-dir", str(directory),
                   "--network", str(network), "--duration", str(args.duration), "--speed", str(args.speed),
                   "--rsu-layout", str(layout)]
        if args.gui:
            command += ["--gui", "--gui-delay-ms", str(args.gui_delay_ms)]
            if args.gui_packets:
                command.append("--gui-packets")
            if args.gui_overview:
                command.append("--gui-overview")
        with (directory / "bridge.log").open("w") as blog, (directory / "ns3.log").open("w") as nlog:
            print("[2/4] TraCI 单车在线步进；每秒按三维距离选择最近 RSU（同距选较小编号）。", flush=True)
            print("紫色=发送目标；绿色=实际收包。切换只改新包目的 IP，不模拟蜂窝/L2切换。", flush=True)
            bridge = subprocess.Popen(command, stdout=blog, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 30
            while not (directory / "ready.json").exists():
                if bridge.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"Bridge startup failed; see {directory / 'bridge.log'}")
                time.sleep(.05)
            ready = json.loads((directory / "ready.json").read_text())
            scenario = [f"--outputDir={directory}", f"--bridgePort={ready['port']}",
                                   f"--rsuFile={layout}", f"--duration={args.duration}",
                                   f"--interval={args.interval}", f"--txPowerDbm={args.tx_power_dbm}",
                                   "--seed=23423", "--run=1", f"--progress={args.progress}"]
            ns3_command = ns3_run_command(TARGET, scenario)
            print("[3/4] 新包仅发送给当前最近节点；切换信息显示为 [HANDOVER]……", flush=True)
            process = subprocess.Popen(ns3_command, cwd=ns3_dir, stdout=nlog, stderr=subprocess.STDOUT, start_new_session=True)
            timeout = max(180, args.duration * (args.gui_delay_ms / 1000 if args.gui else 0) + 150)
            if wait_with_progress(process, directory / "ns3.log", timeout):
                raise RuntimeError(f"ns-3 failed; see {directory / 'ns3.log'}")
            if bridge.wait(timeout=15):
                raise RuntimeError(f"Bridge failed; see {directory / 'bridge.log'}")
        files = [ns3_dir / "scratch" / f"{TARGET}.cc", *[PROJECT / name for name in
                 ("run_handover.py", "sumo_runner.py", "rsu_layout.py", "handover_gui.py", "packet_demo.py",
                  "gui_demo.py", "validate_handover.py", "validate_online.py", "run_online.py", "project_config.py")]]
        metadata = dict(mode="nearest_application_target", rsu_count=len(stations), duration_s=args.duration,
                        speed_mps=args.speed, interval_s=args.interval, tx_power_dbm=args.tx_power_dbm,
                        gui=args.gui, gui_delay_ms=args.gui_delay_ms if args.gui else 0, gui_packets=args.gui_packets,
                        gui_overview=args.gui_overview, sumo_bridge_command=command, ns3_driver_command=ns3_command,
                        presentation=dict(camera="fixed-overview" if args.gui_overview else "legacy-near-follow",
                                          target_fps=30, rx_hold_wall_ms=180, detail_update="once_per_sync"),
                        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                        rng_seed=23423, rng_run=1, ns3_version=(ns3_dir / "VERSION").read_text().strip(),
                        selection_step_s=1, selection="nearest Euclidean 3D, lower-ID tie break",
                        limitations="No RSSI selection, hysteresis, authentication, L2 roaming, cellular handover cost, or backhaul")
        (directory / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print("[4/4] 独立核对逐秒距离、目标选择、切换和每个包的目的节点……", flush=True)
        report = validate_handover(directory)
        (directory / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
        if not args.no_plot:
            plot_handover(directory)
        results = json.loads((directory / "results.json").read_text())
        print(f"验收通过：{report['sync_updates']} 次同步，{report['handover_count']} 次切换。", flush=True)
        print("目标选择序列：" + " -> ".join(f"RSU{i}" for i in report["serving_sequence"]), flush=True)
        print(f"总发包 {report['generated_packets']}，总接收 {report['received_packets']}；"
              f"接收率 {report['received_packets'] / report['generated_packets']:.2%}", flush=True)
        for station in results["rsus"]:
            if station["generated_packets"]:
                print(f"RSU{station['rsu_id']}: TX={station['generated_packets']} RX={station['received_packets']} "
                      f"PDR={station['pdr']:.2%}", flush=True)
        print(f"没有发包业务的候选节点：{report['unused_rsus']}（PDR 未定义）。", flush=True)
        if report["selected_without_traffic"]:
            print(f"其中 {report['selected_without_traffic']} 在发包计划结束后才被选中，尚无新包。", flush=True)
        print(f"完成：{directory}", flush=True)
        return directory
    finally:
        stop_process(process)
        stop_process(bridge)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=DEFAULT_NETWORK)
    parser.add_argument("--ns3-dir", type=Path, default=DEFAULT_NS3)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--interval", type=float, default=.1)
    parser.add_argument("--tx-power-dbm", type=float, default=20.0)
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gui-packets", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gui-overview", action=argparse.BooleanOptionalAction, default=True,
                        help="Fixed full-network camera by default; --no-gui-overview restores near follow")
    parser.add_argument("--gui-delay-ms", type=float, default=250)
    parser.add_argument("--progress", choices=("summary", "packets", "off"), default="summary")
    parser.add_argument("--no-plot", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
