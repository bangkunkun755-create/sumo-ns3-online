"""Run the minimal live SUMO -> TCP -> ns-3 V2I experiment, validate, and plot."""

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from sumo_runner import DEFAULT_NETWORK, check_settings
from validate_online import csv_rows, validate
from project_config import DEFAULT_NS3, check_deployed_scene, ns3_run_command

PROJECT = Path(__file__).resolve().parent
TARGET = "sumo-online-v2i"


def stop_process(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def wait_with_progress(process, log_path, timeout):
    """Forward a growing ns-3 log while keeping a bounded wait and the full log."""
    deadline = time.monotonic() + timeout
    pending = ""
    with log_path.open(encoding="utf-8", errors="replace") as log:
        while True:
            pending += log.read()
            lines = pending.split("\n")
            pending = lines.pop()
            for line in lines:
                print(line, flush=True)
            code = process.poll()
            if code is not None:
                # The child can finish between the read above and poll().
                pending += log.read()
                if pending:
                    print(pending, end="" if pending.endswith("\n") else "\n", flush=True)
                return code
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(process.args, timeout)
            time.sleep(0.03)


def plot_results(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    trajectory = csv_rows(directory / "sumo_trajectory.csv")
    synced = csv_rows(directory / "ns3_sync.csv")
    packets = csv_rows(directory / "packet_summary.csv")
    results = json.loads((directory / "results.json").read_text())
    duration = results["duration_s"]
    times = [float(row["ns3_time_s"]) for row in trajectory]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), constrained_layout=True)
    x0 = float(trajectory[0]["x_m"])
    y0 = float(trajectory[0]["y_m"])
    axes[0].plot([float(row["x_m"]) - x0 for row in trajectory],
                 [float(row["y_m"]) - y0 for row in trajectory], label="SUMO car")
    for station in results["rsus"]:
        axes[0].scatter(station["x_m"] - x0, station["y_m"] - y0,
                        marker="^", s=70, label=f"RSU {station['rsu_id']}")
    axes[0].set(title="Online SUMO car and fixed RSUs (x/y scales differ)",
                xlabel="x relative to car start (m)", ylabel="y relative to car start (m)")
    for rsu, color in ((1, "tab:blue"), (2, "tab:orange")):
        axes[1].plot(times, [float(row[f"distance_rsu{rsu}_m"]) for row in synced],
                     color=color, label=f"RSU {rsu}")
        subset = [row for row in packets if int(row["rsu_id"]) == rsu]
        starts = list(range(0, int(duration), 5))
        pdr = []
        for start in starts:
            group = [row for row in subset if start <= float(row["send_time"]) < start + 5]
            pdr.append(100 * sum(row["received"] == "1" for row in group) / len(group)
                       if group else float("nan"))
        axes[2].stairs(pdr, starts + [duration], color=color, label=f"RSU {rsu}")
        delivered = [row for row in subset if row["received"] == "1"]
        axes[3].scatter([float(row["arrive_time"]) for row in delivered],
                        [float(row["delay_ms"]) for row in delivered],
                        s=8, alpha=0.8, color=color, label=f"RSU {rsu}")
    axes[1].set(title="Distance from the live vehicle to each RSU", ylabel="Distance (m)")
    axes[2].set(title="UDP delivery in 5-second bins, grouped by send_time", ylabel="PDR (%)", ylim=(-3, 105))
    axes[3].set(title="Successfully received UDP packets only; blank means no arrivals",
                ylabel="One-way delay (ms)")
    for index, axis in enumerate(axes):
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
        if index:
            axis.set_xlim(0, duration)
            axis.set_xlabel("ns-3 simulation time (s)")
    fig.savefig(directory / "online_overview.png", dpi=160)
    plt.close(fig)


def run(args):
    if sys.platform != "linux":
        raise RuntimeError("Select the Ubuntu-22.04 /usr/bin/python3 interpreter in PyCharm")
    check_settings(args.duration, args.speed)
    if not math.isfinite(args.gui_delay_ms) or not 0 <= args.gui_delay_ms <= 1000:
        raise ValueError("gui-delay-ms must be from 0 to 1000")
    if not math.isfinite(args.interval) or not 0.01 <= args.interval <= 1:
        raise ValueError("interval must be 0.01..1 second")
    if not all(math.isfinite(value) for value in (args.tx_power_dbm, args.rsu1_offset, args.rsu2_offset)):
        raise ValueError("Power and RSU offsets must be finite")
    args.network = args.network.resolve()
    args.ns3_dir = args.ns3_dir.resolve()
    source = check_deployed_scene(args.ns3_dir, TARGET)
    directory = PROJECT / "runs" / f"online-{datetime.now():%Y%m%d-%H%M%S-%f}"
    directory.mkdir(parents=True, exist_ok=False)
    print(f"结果目录：{directory}", flush=True)
    print("[1/4] 编译 ns-3 在线 V2I 场景……", flush=True)
    with (directory / "ns3_build.log").open("w") as log:
        subprocess.run([sys.executable, "ns3", "build", TARGET, "-j", "2"],
                       cwd=args.ns3_dir, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
    bridge_process = None
    network_process = None
    try:
        with (directory / "bridge.log").open("w") as bridge_log, \
                (directory / "ns3.log").open("w") as ns3_log:
            print("[2/4] 启动 SUMO 单车和 Python Socket 服务……", flush=True)
            bridge_command = [sys.executable, str(PROJECT / "sumo_runner.py"),
                              "--output-dir", str(directory), "--network", str(args.network),
                              "--duration", str(args.duration), "--speed", str(args.speed),
                              "--rsu1-offset", str(args.rsu1_offset), "--rsu2-offset", str(args.rsu2_offset)]
            if args.gui:
                bridge_command += ["--gui", "--gui-delay-ms", str(args.gui_delay_ms)]
                if args.gui_overview:
                    bridge_command.append("--gui-overview")
                if args.gui_packets:
                    bridge_command.append("--gui-packets")
                    print("逐包示意：蓝点=真实 TX 尝试；绿闪=真实 RX；逐秒回放上一秒事件，不表示电波路径或实际时延比例。", flush=True)
                print("SUMO 图形演示：黄色车辆；绿色连线表示上一同步区间实际收到数据包。", flush=True)
                print("车辆按 ns-3 同步请求前进；演示结束后窗口自动关闭，结果保存在本次目录。", flush=True)
            bridge_process = subprocess.Popen(bridge_command, stdout=bridge_log,
                                               stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 30
            while not (directory / "ready.json").exists():
                if bridge_process.poll() is not None:
                    raise RuntimeError(f"SUMO bridge exited: see {directory / 'bridge.log'}")
                if time.monotonic() > deadline:
                    raise TimeoutError("SUMO bridge startup timed out")
                time.sleep(0.05)
            ready = json.loads((directory / "ready.json").read_text())
            scenario = [f"--outputDir={directory}", f"--bridgePort={ready['port']}",
                                   f"--duration={args.duration}", f"--interval={args.interval}",
                                   f"--txPowerDbm={args.tx_power_dbm}", f"--rsu1Offset={args.rsu1_offset}",
                                   f"--rsu2Offset={args.rsu2_offset}", "--seed=23423", "--run=1",
                                   f"--progress={args.progress}"]
            command = ns3_run_command(TARGET, scenario)
            print("[3/4] ns-3 每秒驱动 TraCI，车辆向两个 RSU 发送 UDP……", flush=True)
            network_process = subprocess.Popen(command, cwd=args.ns3_dir, stdout=ns3_log,
                                                stderr=subprocess.STDOUT, start_new_session=True)
            if args.progress != "off":
                print("接收进度：sent=累计发包，received=累计收包，(+N)=上个同步点以来新增收包。", flush=True)
                print("NO_RX_IN_INTERVAL=该区间没有收到包；单包时刻使用 ns-3 仿真时间。", flush=True)
            code = wait_with_progress(network_process, directory / "ns3.log", timeout=180)
            if code:
                raise RuntimeError(f"ns-3 failed with code {code}: see {directory / 'ns3.log'}")
            if bridge_process.wait(timeout=15):
                raise RuntimeError(f"SUMO bridge failed: see {directory / 'bridge.log'}")
        files = [source, *[PROJECT / name for name in ("sumo_runner.py", "run_online.py", "validate_online.py",
                                                     "gui_demo.py", "packet_demo.py", "run_experiment.py", "project_config.py")]]
        metadata = {"mode": "online_lockstep", "ns3_driver_command": command,
                    "sumo_bridge_command": bridge_command, "duration_s": args.duration,
                    "speed_mps": args.speed, "sync_step_s": 1, "interval_s": args.interval,
                    "tx_power_dbm": args.tx_power_dbm, "rng_seed": 23423, "rng_run": 1,
                    "host_socket": ready, "source_sha256": {
                        str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
                    "ns3_version": (args.ns3_dir / "VERSION").read_text().strip(),
                    "console_progress": args.progress,
                    "gui": args.gui, "gui_delay_ms": args.gui_delay_ms if args.gui else 0,
                    "gui_overview": args.gui_overview,
                    "gui_packet_animation": args.gui and args.gui_packets,
                    "mobility_between_syncs": "causal constant-velocity prediction; corrected every second",
                    "feedback": "ns-3 TX/RX counters returned to Python; constant speed is retained"}
        (directory / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print("[4/4] 独立核对时间、坐标、Socket反馈和收发包日志……", flush=True)
        report = validate(directory)
        (directory / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
        if not args.no_plot:
            plot_results(directory)
        results = json.loads((directory / "results.json").read_text())
        print(f"同步通过：{report['sync_updates']} 次，位置误差 {report['max_position_error_m']:.3g} m，"
              f"时钟误差 {report['max_clock_error_s']:.3g} s", flush=True)
        print(f"车速误差：{report['max_speed_error_mps']:.3g} m/s；网络统计已回传并核对", flush=True)
        for station in results["rsus"]:
            mean = station["mean_delay_ms"]
            delay = "无成功接收" if mean is None else f"{mean:.6f} ms"
            print(f"RSU {station['rsu_id']}：生成 {station['generated_packets']}，"
                  f"收到 {station['received_packets']}，PDR {station['pdr']:.2%}，时延 {delay}", flush=True)
        print(f"A2.1～A2.4 验收通过。结果：{directory}", flush=True)
        return directory
    finally:
        stop_process(network_process)
        stop_process(bridge_process)


def main(default_gui=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=DEFAULT_NETWORK)
    parser.add_argument("--ns3-dir", type=Path, default=DEFAULT_NS3)
    parser.add_argument("--duration", type=float, default=100.0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--tx-power-dbm", type=float, default=20.0)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--rsu1-offset", type=float, default=250.0)
    parser.add_argument("--rsu2-offset", type=float, default=750.0)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--gui", action="store_true", default=default_gui,
                        help="Show the live QuickStart road and receive-feedback overlays in SUMO GUI")
    parser.add_argument("--gui-delay-ms", type=float, default=1000 if default_gui else 250,
                        help="Presentation wait per simulation second, 0..1000 ms; GUI entry defaults to 1000")
    parser.add_argument("--gui-overview", action="store_true",
                        help="Initially show the complete QuickStart network instead of the communication road")
    parser.add_argument("--gui-packets", action=argparse.BooleanOptionalAction, default=True,
                        help="In GUI mode animate every real TX/RX from the previous sync interval; --no-gui-packets disables it")
    parser.add_argument("--progress", choices=("summary", "packets", "off"), default="summary",
                        help="summary: per-second receive counts; packets: also every received packet; off: final results only")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
