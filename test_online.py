"""Protocol rejection and independent validation tests for the live coupling."""

import csv
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import MagicMock

from gui_demo import GuiDemo, link_shape
from sumo_runner import PROJECT, check_settings, parse_step
from validate_online import validate


class GuiFeedbackTests(unittest.TestCase):
    def test_links_follow_new_receipts_not_distance_or_old_counts(self):
        fake = MagicMock()
        fake.gui.DEFAULT_VIEW = "View #0"
        initial = {"vehicle_id": "online_car", "x_m": 0, "y_m": 0,
                   "speed_mps": 10, "seq": 0, "ns3_time_s": 0}
        with tempfile.TemporaryDirectory(prefix="sumo-gui-feedback-") as temp:
            demo = GuiDemo(fake, initial, (10, 20), 100, 0, Path(temp))
            try:
                demo.update(initial, [10, 0, 10, 0])
                fake.polygon.add.assert_not_called()
                demo.update(dict(initial, seq=1, ns3_time_s=1), [20, 3, 20, 0])
                demo.update(dict(initial, seq=2, ns3_time_s=2), [30, 3, 30, 4])
                demo.update(dict(initial, seq=3, ns3_time_s=3), [40, 3, 40, 4])
            finally:
                demo.close()
            events = [json.loads(line) for line in (Path(temp) / "gui_feedback.jsonl").read_text().splitlines()]
            self.assertEqual([event["link_visible"] for event in events],
                             [[False, False], [True, False], [False, True], [False, False]])
            self.assertEqual([event["newly_received"] for event in events],
                             [[0, 0], [3, 0], [0, 4], [0, 0]])
            self.assertEqual(fake.polygon.add.call_count, 2)
            self.assertEqual(fake.polygon.remove.call_count, 2)

    def test_link_shape_is_finite_even_for_coincident_nodes(self):
        for end in ((0, 0), (0, 10), (10, 0)):
            shape = link_shape((0, 0), end)
            self.assertEqual(len(shape), 4)
            self.assertTrue(all(abs(value) < 100 for point in shape for value in point))


class ProtocolTests(unittest.TestCase):
    def test_ordered_step_and_feedback(self):
        self.assertEqual(parse_step("STEP 5 5 40 2 40 0", 5, 100), (5, 5.0, [40, 2, 40, 0]))

    def test_skipped_step_rejected(self):
        with self.assertRaisesRegex(ValueError, "Out-of-order"):
            parse_step("STEP 2 2 0 0 0 0", 1, 100)

    def test_nan_time_rejected(self):
        with self.assertRaises(ValueError):
            parse_step("STEP 1 nan 0 0 0 0", 1, 100)

    def test_impossible_feedback_rejected(self):
        with self.assertRaises(ValueError):
            parse_step("STEP 1 1 2 3 0 0", 1, 100)

    def test_nonintegral_duration_and_excess_speed_rejected(self):
        for duration, speed in ((10.5, 10), (100.0, 20), (100.0, float("nan"))):
            with self.assertRaises(ValueError):
                check_settings(duration, speed)

    @unittest.skipUnless(sys.platform == "linux", "Requires Ubuntu SUMO")
    def test_live_bridge_rejects_clock_jump(self):
        with tempfile.TemporaryDirectory(prefix="sumo-online-protocol-") as temp:
            directory = Path(temp)
            with (directory / "test.log").open("w") as log:
                process = subprocess.Popen([sys.executable, str(PROJECT / "sumo_runner.py"),
                                            "--output-dir", temp, "--duration", "5"],
                                           stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 15
                    while not (directory / "ready.json").exists():
                        self.assertIsNone(process.poll(), "Bridge exited before accepting the test")
                        self.assertLess(time.monotonic(), deadline)
                        time.sleep(0.05)
                    ready = json.loads((directory / "ready.json").read_text())
                    with socket.create_connection((ready["host"], ready["port"]), timeout=5) as client:
                        with client.makefile("rwb") as stream:
                            stream.write(b"STEP 0 0 0 0 0 0\n")
                            stream.flush()
                            self.assertTrue(stream.readline().startswith(b"STATE 0 0 1 1 "))
                            stream.write(b"STEP 2 2 0 0 0 0\n")
                            stream.flush()
                            self.assertEqual(stream.readline(), b"")
                    self.assertNotEqual(process.wait(timeout=10), 0)
                    error = json.loads((directory / "bridge_error.json").read_text())
                    self.assertIn("Out-of-order", error["error"])
                    self.assertFalse((directory / "bridge_result.json").exists())
                finally:
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=10)


@unittest.skipUnless(os.environ.get("ONLINE_TEST_RUN"), "Set ONLINE_TEST_RUN to a successful online run")
class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="sumo-online-validation-")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        source = Path(os.environ["ONLINE_TEST_RUN"])
        for name in ("sumo_metadata.json", "results.json", "bridge_result.json", "sumo_trajectory.csv",
                     "ns3_sync.csv", "network_feedback.csv", "socket_exchange.jsonl", "packets.jsonl",
                     "packet_summary.csv"):
            shutil.copy2(source / name, self.directory / name)
        if (source / "gui_feedback.jsonl").exists():
            shutil.copy2(source / "gui_feedback.jsonl", self.directory / "gui_feedback.jsonl")
        if (source / "network_snapshot.net.xml").exists():
            shutil.copy2(source / "network_snapshot.net.xml", self.directory / "network_snapshot.net.xml")
        if (source / "gui_packet_events.jsonl").exists():
            shutil.copy2(source / "gui_packet_events.jsonl", self.directory / "gui_packet_events.jsonl")

    def change_csv(self, filename, field, value):
        path = self.directory / filename
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            columns = reader.fieldnames
            rows = list(reader)
        rows[5][field] = str(value)
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)

    def test_actual_integrated_run_passes(self):
        self.assertTrue(validate(self.directory)["passed"])

    def test_gui_packet_display_matches_every_real_event(self):
        path = self.directory / "gui_packet_events.jsonl"
        if not path.exists():
            self.skipTest("Requires a successful GUI packet run")
        actual = [json.loads(line) for line in (self.directory / "packets.jsonl").read_text().splitlines()]
        shown = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([row["packet"] for row in shown], actual)
        for row in shown:
            event = row["packet"]
            when = event["send_time"] if event["event"] == "tx" else event["arrive_time"]
            self.assertLessEqual(when, row["barrier_s"] + 1e-8)

    def test_gui_overlay_matches_every_live_feedback_update(self):
        path = self.directory / "gui_feedback.jsonl"
        if not path.exists():
            self.skipTest("Requires a successful GUI run")
        with (self.directory / "network_feedback.csv").open(newline="") as stream:
            feedback = list(csv.DictReader(stream))
        events = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(len(events), len(feedback))
        previous = [0, 0]
        for event, row in zip(events, feedback):
            received = [int(row["rsu1_rx"]), int(row["rsu2_rx"])]
            delta = [received[i] - previous[i] for i in range(2)]
            self.assertEqual(event["seq"], int(row["seq"]))
            self.assertEqual(event["ns3_time_s"], float(row["ns3_time_s"]))
            self.assertEqual(event["received"], received)
            self.assertEqual(event["newly_received"], delta)
            self.assertEqual(event["link_visible"], [value > 0 for value in delta])
            previous = received

    def test_wrong_ns3_position_rejected(self):
        self.change_csv("ns3_sync.csv", "ns3_x_m", 999999)
        with self.assertRaises(ValueError):
            validate(self.directory)

    def test_wrong_feedback_rejected(self):
        self.change_csv("network_feedback.csv", "rsu1_rx", 999)
        with self.assertRaisesRegex(ValueError, "feedback"):
            validate(self.directory)

    def test_missing_live_exchange_rejected(self):
        path = self.directory / "socket_exchange.jsonl"
        path.write_text("\n".join(path.read_text().splitlines()[1:]) + "\n")
        with self.assertRaisesRegex(ValueError, "Missing synchronization"):
            validate(self.directory)

    def test_fabricated_arrival_timestamp_rejected(self):
        path = self.directory / "packets.jsonl"
        events = [json.loads(line) for line in path.read_text().splitlines()]
        event = next(event for event in events if event["event"] == "rx")
        event["arrive_time"] += 0.25
        path.write_text("".join(json.dumps(event) + "\n" for event in events))
        with self.assertRaisesRegex(ValueError, "Arrival time and delay"):
            validate(self.directory)


if __name__ == "__main__":
    unittest.main()
