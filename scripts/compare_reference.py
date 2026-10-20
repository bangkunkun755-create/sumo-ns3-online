"""Compare default scientific output bytes; metadata is intentionally machine-specific."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_config import PROJECT


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--reference", type=Path)
    args = parser.parse_args()
    results = json.loads((args.directory / "results.json").read_text())
    nearest = results.get("association_mode") == "nearest"
    reference = args.reference or PROJECT / "examples" / ("reference_handover" if nearest else "reference_online")
    manifest = json.loads((reference / "reference_manifest.json").read_text())
    same = {name: (args.directory / name).is_file() and hashlib.sha256((args.directory / name).read_bytes()).hexdigest() == expected
            for name, expected in manifest["core_sha256"].items()}
    print(json.dumps({"passed": all(same.values()), "core_files_identical": same,
                      "note": "Differences may reflect changed versions/parameters; run the independent validator too"}, indent=2))
    sys.exit(0 if all(same.values()) else 1)
