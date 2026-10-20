"""Deploy two scenes without silent overwrite; configure/build a user-owned ns-3 tree."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_config import PROJECT, TARGETS, check_ns3_dir, default_ns3_dir


def deploy(ns3_dir, replace=False):
    ns3_dir = check_ns3_dir(ns3_dir)
    plans = []
    for target in TARGETS:
        source = PROJECT / "ns3" / "scratch" / (target + ".cc")
        destination = ns3_dir / "scratch" / source.name
        same = destination.is_file() and destination.read_bytes() == source.read_bytes()
        if destination.exists() and not same and not replace:
            raise FileExistsError(f"Refusing to overwrite {destination}; inspect it first, then opt into --replace-scenes to keep a backup")
        plans.append((source, destination, same))
    for source, destination, same in plans:
        if same:
            print(f"UNCHANGED {destination.name}", flush=True)
            continue
        if destination.exists():
            backup = destination.with_name(destination.name + f".backup-{time.time_ns()}")
            shutil.copy2(destination, backup)
            print(f"BACKUP {backup}", flush=True)
        shutil.copy2(source, destination)
        print(f"DEPLOY {destination.name} SHA256={hashlib.sha256(source.read_bytes()).hexdigest()}", flush=True)
    return ns3_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ns3-dir", type=Path, default=default_ns3_dir())
    parser.add_argument("--replace-scenes", action="store_true")
    parser.add_argument("--configure", action="store_true", help="Configure fresh source with required Wi-Fi/application modules")
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if sys.platform != "linux" or args.jobs < 1:
        parser.error("Run under Linux/WSL with --jobs >= 1, without sudo")
    ns3_dir = deploy(args.ns3_dir, args.replace_scenes)
    if args.configure:
        subprocess.run([sys.executable, "ns3", "configure", "--build-profile=default",
                        "--disable-examples", "--disable-tests", "--enable-modules=wifi;applications;mobility;internet"],
                       cwd=ns3_dir, check=True)
    if not args.no_build:
        subprocess.run([sys.executable, "ns3", "build", *TARGETS, "-j", str(args.jobs)], cwd=ns3_dir, check=True)
    config = PROJECT / ".local" / "ns3_path"
    config.parent.mkdir(exist_ok=True)
    config.write_text(str(ns3_dir) + "\n", encoding="utf-8")
    print("Saved local-only ns-3 path. Next: python3 scripts/doctor.py", flush=True)


if __name__ == "__main__":
    main()
