"""Portable repository paths and explicit deployment checks (no auto-install)."""
import hashlib
import os
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent
DEFAULT_NETWORK = PROJECT / "scenarios" / "quickstart.net.xml"
TARGETS = ("sumo-online-v2i", "sumo-nearest-rsu")


def ns3_run_command(target, arguments):
    # Forward argv after -- instead of embedding a shell-like command string.
    # ns-3.47 splits embedded command strings on whitespace, ignoring quotes.
    return [sys.executable, "ns3", "run", "--no-build", target, "--", *arguments]


def default_ns3_dir():
    explicit = os.environ.get("NS3_DIR")
    saved = PROJECT / ".local" / "ns3_path"
    if explicit:
        return Path(explicit).expanduser()
    if saved.is_file():
        return Path(saved.read_text(encoding="utf-8").strip()).expanduser()
    return PROJECT.parent / "ns-3.47"


DEFAULT_NS3 = default_ns3_dir()


def check_ns3_dir(directory):
    directory = Path(directory).expanduser().resolve()
    if not (directory / "ns3").is_file() or not (directory / "VERSION").is_file():
        raise ValueError("Select the ns-3 SOURCE root containing ns3 and VERSION, not this repository's ns3/ folder")
    if (directory / "VERSION").read_text().strip() != "3.47":
        raise ValueError("This reproducibility package requires ns-3.47; select it with --ns3-dir")
    return directory


def check_deployed_scene(directory, target):
    directory = check_ns3_dir(directory)
    source = directory / "scratch" / (target + ".cc")
    bundled = PROJECT / "ns3" / "scratch" / source.name
    if not source.is_file() or hashlib.sha256(source.read_bytes()).digest() != hashlib.sha256(bundled.read_bytes()).digest():
        raise ValueError(f"Missing or outdated {source.name}; run bash scripts/setup.sh --ns3-dir '{directory}'")
    return source
