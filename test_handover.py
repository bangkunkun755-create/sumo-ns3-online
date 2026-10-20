"""Nearest selection must follow geometry, not a pre-scripted switching animation."""

import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import MagicMock

from packet_demo import PacketAnimation, PacketEventTail
from handover_gui import HandoverGui
from rsu_layout import build_layout, check_handover_settings, nearest_station, point_along, read_layout, write_layout
from sumo_runner import DEFAULT_NETWORK, parse_step
from validate_handover import validate_handover


class LayoutPolicyTests(unittest.TestCase):
    def test_overview_camera_and_hud_anchors_do_not_follow_car(self):
        with tempfile.TemporaryDirectory() as temp:
            fake = MagicMock()
            fake.gui.DEFAULT_VIEW = "View #0"
            fake.simulation.getNetBoundary.return_value = ((-1000, 0), (3000, 1000))
            initial = dict(x_m=-900, y_m=995, vehicle_id="car", ns3_time_s=0,
                           seq=0, serving_rsu=1, nearest_distance_m=50)
            stations = [dict(rsu_id=1, x_m=-850, y_m=1005), dict(rsu_id=2, x_m=0, y_m=1005)]
            gui = HandoverGui(fake, initial, stations, 300, 0, Path(temp), overview=True, packet_demo=False)
            try:
                gui.update(initial, [0, 0, 0, 0])
                gui.update(dict(initial, x_m=100, ns3_time_s=100, seq=100, serving_rsu=2), [10, 5, 10, 3])
                self.assertEqual(fake.gui.setBoundary.call_count, 1)
                fake.gui.setOffset.assert_not_called()
                for name, anchor in gui.fixed_labels.items():
                    positions = [c.args[1:] for c in fake.poi.setPosition.call_args_list if c.args[0] == name]
                    self.assertEqual(positions, [anchor, anchor])
            finally:
                gui.close()

    def test_sites_follow_original_road_geometry(self):
        rows = build_layout(DEFAULT_NETWORK)
        self.assertEqual(len(rows), 24)
        self.assertEqual([r["rsu_id"] for r in rows], list(range(1, 25)))
        self.assertTrue(all(r["y_m"] > 990 for r in rows[:16]))
        self.assertTrue(any(100 < r["y_m"] < 900 for r in rows[16:]))
        self.assertTrue(any(r["y_m"] < 20 for r in rows[16:]))
        self.assertEqual(point_along([(0, 0), (100, 0)], .25, 10), (25, 10))
        self.assertEqual(point_along([(0, 0), (0, 100)], .25, 10), (-10, 25))

    def test_nearest_changes_with_position_and_ties_choose_lower_id(self):
        stations = [dict(rsu_id=1, x_m=0, y_m=10, z_m=3),
                    dict(rsu_id=2, x_m=100, y_m=10, z_m=3)]
        def select(x):
            return nearest_station(dict(x_m=x, y_m=0, z_m=1.5), stations)[0]
        self.assertEqual([select(x) for x in (0, 49, 50, 51, 100, 50)], [1, 1, 1, 2, 2, 1])

    def test_layout_roundtrip_and_duplicate_rejection(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "layout.csv"
            rows = build_layout(DEFAULT_NETWORK)
            write_layout(path, rows)
            self.assertEqual(read_layout(path), rows)
            rows[-1].update({key: rows[0][key] for key in ("x_m", "y_m", "z_m")})
            write_layout(path, rows)
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                read_layout(path)

    def test_multi_rsu_protocol_is_exact_length_and_validates_each_pair(self):
        line = "STEP 2 2 " + " ".join(["10 2"] * 24)
        self.assertEqual(len(parse_step(line, 2, 300, 24)[2]), 48)
        with self.assertRaises(ValueError):
            parse_step(line, 2, 300, 23)
        with self.assertRaises(ValueError):
            parse_step("STEP 2 2 " + " ".join(["10 2"] * 23 + ["1 2"]), 2, 300, 24)

    def test_longer_route_bounds_and_nonfinite_speed_rejected(self):
        check_handover_settings(300.0, 10)
        for duration, speed in ((391.0, 10), (300.5, 10), (300.0, float("nan"))):
            with self.assertRaises(ValueError):
                check_handover_settings(duration, speed)

    def test_packet_reader_and_compact_animation_support_rsu24(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            event = dict(event="tx", seq=0, rsu_id=24, send_time=1.5, arrive_time=None,
                         delay_ms=None, socket_accepted=True, bytes=256)
            (path / "packets.jsonl").write_text(json.dumps(event) + "\n")
            fake = MagicMock()
            fake.gui.DEFAULT_VIEW = "View #0"
            fake.gui.getOffset.return_value = (0, 0)
            animation = PacketAnimation(fake, dict(x_m=0, y_m=0), [(i * 100, 10) for i in range(24)],
                                        path, compact=True)
            try:
                counts = [0] * 48
                counts[46] = 1
                animation.update(dict(x_m=0, y_m=0, ns3_time_s=2), counts, 0)
                self.assertIn("RSU24", animation.last_tx[23])
                self.assertEqual(animation.reader.rx, set())
            finally:
                animation.close()


@unittest.skipUnless(os.environ.get("HANDOVER_TEST_RUN"), "Set HANDOVER_TEST_RUN to an actual run")
class HandoverIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="handover-validation-")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        source = Path(os.environ["HANDOVER_TEST_RUN"])
        for filename in ("sumo_metadata.json", "results.json", "bridge_result.json", "sumo_trajectory.csv",
                         "ns3_sync.csv", "network_feedback.csv", "socket_exchange.jsonl", "packets.jsonl",
                         "packet_summary.csv", "rsu_layout.csv", "association.csv", "handover_events.jsonl",
                         "gui_feedback.jsonl", "gui_packet_events.jsonl", "network_snapshot.net.xml"):
            if (source / filename).exists():
                shutil.copy2(source / filename, self.directory / filename)

    def test_actual_run_has_single_destination_and_correct_switches(self):
        report = validate_handover(self.directory)
        self.assertTrue(report["passed"])
        self.assertEqual(report["rsu_count"], 24)
        self.assertGreater(report["handover_count"], 1)

    def test_non_nearest_target_is_rejected(self):
        path = self.directory / "association.csv"
        lines = path.read_text().splitlines()
        parts = lines[5].split(",")
        parts[3] = "24"
        lines[5] = ",".join(parts)
        path.write_text("\n".join(lines) + "\n")
        with self.assertRaisesRegex(ValueError, "nearest"):
            validate_handover(self.directory)

    def test_missing_handover_event_is_rejected(self):
        path = self.directory / "handover_events.jsonl"
        path.write_text("\n".join(path.read_text().splitlines()[1:]) + "\n")
        with self.assertRaisesRegex(ValueError, "Missing handover"):
            validate_handover(self.directory)

    def test_unused_rsu_pdr_must_be_null_not_zero(self):
        path = self.directory / "results.json"
        results = json.loads(path.read_text())
        station = next(s for s in results["rsus"] if not s["generated_packets"])
        station["pdr"] = 0
        path.write_text(json.dumps(results))
        with self.assertRaisesRegex(ValueError, "undefined PDR"):
            validate_handover(self.directory)

    def test_wrong_packet_destination_is_rejected(self):
        path = self.directory / "packets.jsonl"
        events = [json.loads(line) for line in path.read_text().splitlines()]
        events[0]["rsu_id"] = 24
        path.write_text("".join(json.dumps(event) + "\n" for event in events))
        with self.assertRaises(ValueError):
            validate_handover(self.directory)


if __name__ == "__main__":
    unittest.main()
