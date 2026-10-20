"""Maintainer tool: export an existing validated run without absolute host paths."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validate_online import resolve_network, validate
from validate_handover import validate_handover

CORE = ("sumo_trajectory.csv", "ns3_sync.csv", "network_feedback.csv", "packets.jsonl", "packet_summary.csv", "results.json")


def export(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    meta = json.loads((source / "sumo_metadata.json").read_text())
    nearest = meta.get("association_mode") == "nearest"
    validator = validate_handover if nearest else validate
    validator(source)
    if destination.exists() and any(destination.iterdir()):
        raise FileExistsError("Refusing to overwrite a populated reference directory")
    destination.mkdir(parents=True, exist_ok=True)
    core = list(CORE) + (["association.csv", "handover_events.jsonl"] if nearest else [])
    for name in core + ["bridge_result.json", "socket_exchange.jsonl", "rsu_layout.csv", "gui_feedback.jsonl", "gui_packet_events.jsonl",
                        "handover_overview.png", "online_overview.png", "gui_at_075s.png"]:
        if (source / name).is_file():
            shutil.copy2(source / name, destination / name)
    shutil.copy2(resolve_network(meta, source), destination / "network_snapshot.net.xml")
    meta.pop("sumo_command", None)
    meta.pop("traci_module", None)
    meta.update(network="network_snapshot.net.xml", network_snapshot="network_snapshot.net.xml")
    (destination / "sumo_metadata.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    report = validator(destination)
    (destination / "validation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    manifest = {"original_run_id": source.name, "provenance": "actual validated experiment, not synthetic",
                "metadata_changes": ["Removed host sumo_command and traci_module", "Replaced network path with local hash-checked snapshot"],
                "rng_seed": 23423, "rng_run": 1,
                "core_sha256": {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in core}}
    assert all(hashlib.sha256((destination / n).read_bytes()).hexdigest() == h for n, h in manifest["core_sha256"].items())
    (destination / "reference_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Exported and independently validated {destination.name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    export(args.source, args.destination)
