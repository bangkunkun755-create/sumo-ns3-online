"""Read-only prerequisite check. Does not install packages or restart WSLg."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_config import DEFAULT_NETWORK, TARGETS, check_deployed_scene, default_ns3_dir


def inspect(ns3_dir, gui=False):
    failures, warnings = [], []
    report = {"python": sys.version.split()[0], "platform": sys.platform}
    if sys.platform != "linux" or sys.version_info < (3, 10):
        failures.append("Use Linux/WSL Python >=3.10 (reference: Ubuntu22.04 Python3.10)")
    if not DEFAULT_NETWORK.is_file():
        failures.append("Bundled scenarios/quickstart.net.xml is missing")
    for name in ("sumo", "cmake", "g++"):
        binary = shutil.which(name)
        report[name] = binary
        if not binary:
            failures.append(f"Missing executable: {name}")
    if report.get("sumo"):
        report["sumo_version"] = subprocess.check_output([report["sumo"], "--version"], text=True).splitlines()[0]
        if "1.12.0" not in report["sumo_version"]:
            warnings.append("SUMO differs from reference1.12.0; validate outcomes rather than assuming identical PDR")
    try:
        from sumo_runner import load_traci
        report["traci"] = load_traci().__file__
    except ImportError as exc:
        failures.append(f"TraCI unavailable: {exc}; install sumo-tools or set SUMO_HOME")
    try:
        import matplotlib
        import numpy
        report.update(matplotlib=matplotlib.__version__, numpy=numpy.__version__)
        if (matplotlib.__version__, numpy.__version__) != ("3.5.1", "1.21.5"):
            warnings.append("Plotting dependencies differ from the reference; see requirements.txt")
    except ImportError as exc:
        failures.append(f"Plot dependency unavailable: {exc}")
    try:
        for target in TARGETS:
            check_deployed_scene(ns3_dir, target)
        report["ns3_version"] = "3.47"
    except (ValueError, OSError) as exc:
        failures.append(str(exc))
    if gui and (not shutil.which("sumo-gui") or not os.environ.get("DISPLAY")):
        failures.append("GUI requires sumo-gui and a working DISPLAY/WSLg; first try --no-gui")
    report.update(passed=not failures, failures=failures, warnings=warnings,
                  note="Executable/DISPLAY checks do not prove WSLg rendering works or that C++ has been compiled")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ns3-dir", type=Path, default=default_ns3_dir())
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    report = inspect(args.ns3_dir, args.gui)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    sys.exit(0 if report["passed"] else 1)
