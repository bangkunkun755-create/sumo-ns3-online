"""Maintainer packaging: explicit file allowlist, no caches/results/local paths."""
import argparse
import hashlib
from pathlib import Path
import sys
import zipfile

PROJECT = Path(__file__).resolve().parents[1]


def selected_files():
    files = [p for p in PROJECT.iterdir() if p.is_file() and
             (p.suffix in (".py", ".md") or p.name in ("requirements.txt", ".gitignore", ".gitattributes"))]
    for name in ("ns3", "scenarios", "scripts", "docs", "examples", "third_party"):
        for path in (PROJECT / name).rglob("*"):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix not in (".pyc", ".pyo"):
                files.append(path)
    return sorted(files, key=lambda p: p.relative_to(PROJECT).as_posix())


def build(destination):
    destination = Path(destination).resolve()
    if destination.exists():
        raise FileExistsError("Choose a new archive filename; existing archives are not overwritten")
    destination.parent.mkdir(parents=True, exist_ok=True)
    files = selected_files()
    required = [PROJECT / name for name in ("README.md", "docs/setup.md", "docs/reproducibility.md",
                "scenarios/quickstart.net.xml", "examples/reference_handover/reference_manifest.json",
                "examples/reference_online/reference_manifest.json")]
    if not all(p in files for p in required):
        raise ValueError("Release missing required documentation/data")
    manifest = "".join(hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.relative_to(PROJECT).as_posix() + "\n" for p in files)
    with zipfile.ZipFile(destination, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            archive.write(path, "sumo-ns3-online-repro/" + path.relative_to(PROJECT).as_posix())
        archive.writestr("sumo-ns3-online-repro/MANIFEST.sha256", manifest)
    with zipfile.ZipFile(destination) as archive:
        if archive.testzip() is not None:
            raise ValueError("ZIP CRC failed")
        for path in files:
            name = "sumo-ns3-online-repro/" + path.relative_to(PROJECT).as_posix()
            assert archive.read(name) == path.read_bytes()
    print(f"Packaged {len(files)} files plus manifest; {destination.stat().st_size} bytes")
    print(f"SHA256 {hashlib.sha256(destination.read_bytes()).hexdigest()}")
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    build(parser.parse_args().destination)
