"""Portable defaults, safe deployment, and self-contained result validation."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import project_config
from scripts.setup_ns3 import deploy
from validate_online import resolve_network


class PortabilityTests(unittest.TestCase):
    def test_forwarded_arguments_preserve_spaces_as_one_argument(self):
        value = "--outputDir=/tmp/moved experiment/results"
        command = project_config.ns3_run_command("sumo-nearest-rsu", [value])
        self.assertEqual(command[-2:], ["--", value])

    def test_bundled_network_and_both_scenes_exist(self):
        self.assertTrue(project_config.DEFAULT_NETWORK.is_file())
        for name in project_config.TARGETS:
            self.assertTrue((project_config.PROJECT / "ns3" / "scratch" / (name + ".cc")).is_file())

    def test_ns3_environment_override(self):
        with patch.dict("os.environ", {"NS3_DIR": "/tmp/independent-ns3"}):
            self.assertEqual(project_config.default_ns3_dir(), Path("/tmp/independent-ns3"))

    def test_source_folder_not_complete_ns3_is_rejected(self):
        with self.assertRaises(ValueError):
            project_config.check_ns3_dir(project_config.PROJECT / "ns3")

    def test_wrong_ns3_version_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); (root / "ns3").touch(); (root / "VERSION").write_text("3.46")
            with self.assertRaisesRegex(ValueError, "3.47"):
                project_config.check_ns3_dir(root)

    def test_scene_deployment_is_idempotent_and_refuses_conflict(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); (root / "ns3").touch(); (root / "VERSION").write_text("3.47")
            (root / "scratch").mkdir()
            deploy(root); deploy(root)
            source = project_config.check_deployed_scene(root, project_config.TARGETS[0])
            source.write_text("student modification")
            with self.assertRaises(FileExistsError):
                deploy(root)
            self.assertEqual(source.read_text(), "student modification")
            deploy(root, replace=True)
            self.assertEqual(len(list(source.parent.glob(source.name + ".backup-*"))), 1)
            project_config.check_deployed_scene(root, project_config.TARGETS[0])

    def test_snapshot_does_not_require_authors_original_path(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); snapshot = root / "network_snapshot.net.xml"; snapshot.write_text("network")
            meta = {"network": "/nonexistent/author/network.xml", "network_snapshot": snapshot.name}
            self.assertEqual(resolve_network(meta, root), snapshot)

    def test_missing_or_escaping_snapshot_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            for name in ("missing.xml", "../outside.xml"):
                with self.assertRaises(ValueError):
                    resolve_network({"network_snapshot": name}, temp)

    def test_reference_hashes_match_actual_files(self):
        import hashlib
        for mode in ("reference_handover", "reference_online"):
            root = project_config.PROJECT / "examples" / mode
            manifest = json.loads((root / "reference_manifest.json").read_text())
            for name, digest in manifest["core_sha256"].items():
                self.assertEqual(hashlib.sha256((root / name).read_bytes()).hexdigest(), digest)


if __name__ == "__main__":
    unittest.main()
