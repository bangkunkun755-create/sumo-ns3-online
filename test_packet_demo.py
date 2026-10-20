"""Packet display must be traceable to real, ordered ns-3 events."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from packet_demo import PacketAnimation, PacketEventTail, packet_position


def tx(seq=0, when=1.2, rsu=1):
    return dict(event="tx", seq=seq, rsu_id=rsu, send_time=when,
                arrive_time=None, delay_ms=None, socket_accepted=True, bytes=256)


def rx(seq=0, sent=1.2, arrived=1.21, rsu=1, unique=True):
    return dict(event="rx", seq=seq, rsu_id=rsu, send_time=sent,
                arrive_time=arrived, delay_ms=(arrived - sent) * 1000,
                unique=unique, bytes=256)


class PacketDemoTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="packet-demo-test-")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.path = self.directory / "packets.jsonl"
        self.reader = PacketEventTail(self.path)
        self.addCleanup(self.reader.close)

    def append(self, events):
        with self.path.open("a") as stream:
            for event in events:
                stream.write(json.dumps(event) + "\n")

    def test_tail_has_no_future_knowledge_and_accepts_late_arrival(self):
        self.assertEqual(self.reader.read(0, [0, 0, 0, 0]), [])
        self.append([tx()])
        self.assertEqual(self.reader.read(2, [1, 0, 0, 0]), [tx()])
        self.assertEqual(self.reader.rx, set())  # Not received is not a loss decision.
        self.append([rx(arrived=3.5)])
        self.assertEqual(self.reader.read(4, [1, 1, 0, 0]), [rx(arrived=3.5)])
        self.assertEqual(self.reader.read(5, [1, 1, 0, 0]), [])

    def test_counter_mismatch_rejected(self):
        self.append([tx()])
        with self.assertRaisesRegex(ValueError, "disagree"):
            self.reader.read(2, [1, 1, 0, 0])

    def test_future_and_orphan_rx_rejected(self):
        self.append([tx(when=3)])
        with self.assertRaisesRegex(ValueError, "future"):
            self.reader.read(2, [1, 0, 0, 0])
        self.reader.close()
        self.path.write_text(json.dumps(rx()) + "\n")
        reader = PacketEventTail(self.path)
        self.addCleanup(reader.close)
        with self.assertRaisesRegex(ValueError, "matching TX"):
            reader.read(2, [0, 1, 0, 0])

    def test_duplicates_count_only_once_and_wrong_delay_rejected(self):
        self.append([tx(), rx(), rx(arrived=1.22, unique=False)])
        self.assertEqual(len(self.reader.read(2, [1, 1, 0, 0])), 3)
        invalid = rx(arrived=2.5, unique=False)
        invalid["delay_ms"] = 0
        self.append([invalid])
        with self.assertRaisesRegex(ValueError, "delay"):
            self.reader.read(3, [1, 1, 0, 0])

    def test_all_events_audited_without_advancing_simulation(self):
        events = [tx(), rx(), tx(1, 1.5, 2)]
        self.append(events)
        fake = MagicMock()
        fake.gui.DEFAULT_VIEW = "View #0"
        fake.gui.getOffset.return_value = (0, 0)
        initial = dict(x_m=0, y_m=0)
        animation = PacketAnimation(fake, initial, [(250, 10), (750, 10)], self.directory)
        try:
            animation.update(dict(initial, ns3_time_s=2), [1, 1, 1, 0], 0)
        finally:
            animation.close()
        displayed = [json.loads(line)["packet"] for line in
                     (self.directory / "gui_packet_events.jsonl").read_text().splitlines()]
        self.assertEqual(displayed, events)
        fake.simulationStep.assert_not_called()
        self.assertIn("RX #0", animation.last_rx[0])
        self.assertEqual(animation.last_rx[1], "RX: no arrival yet")

    def test_schematic_marker_moves_without_representing_arrival(self):
        self.assertEqual(packet_position((0, 0), (100, 10), 0, 1), (0, 0))
        self.assertLess(packet_position((0, 0), (100, 10), 0.5, 1)[1], 0)
        self.assertLess(packet_position((0, 0), (100, 10), 0.9, 1)[0], 100)

    def test_stable_frames_keep_camera_and_batch_text_updates(self):
        events = [tx(0, 1.7), tx(1, 1.8), rx(0, 1.7, 1.81), rx(1, 1.8, 1.82)]
        self.append(events)
        fake = MagicMock()
        fake.gui.DEFAULT_VIEW = "View #0"
        fake.gui.getZoom.return_value = 42.5
        fake.gui.getOffset.return_value = (125, 250)
        animation = PacketAnimation(fake, dict(x_m=0, y_m=0), [(100, 10), (200, 10)], self.directory)
        try:
            with patch("packet_demo.time.sleep"):
                animation.update(dict(x_m=0, y_m=0, ns3_time_s=2), [2, 2, 0, 0], .25)
            self.assertEqual(fake.gui.setZoom.call_count, 9)
            self.assertTrue(all(c.args == ("View #0", 42.5) for c in fake.gui.setZoom.call_args_list))
            fake.gui.setOffset.assert_not_called()
            fake.simulationStep.assert_not_called()
            for label in ("packet_tx_0", "packet_rx_0"):
                self.assertEqual(sum(c.args[0] == label for c in fake.poi.setType.call_args_list), 1)
            # No simulation advance or bulk deletion at a barrier: recently sent dots survive.
            self.assertEqual(len(animation.markers), 2)
            fake.poi.remove.assert_not_called()
            animation.update(dict(x_m=10, y_m=0, ns3_time_s=2.1), [2, 2, 0, 0], 0)
            self.assertTrue(all(origin == (0, 0) for _, _, origin in animation.markers.values()))
            animation.update(dict(x_m=20, y_m=0, ns3_time_s=3), [2, 2, 0, 0], 0)
            self.assertEqual(animation.markers, {})
        finally:
            animation.close()
        displayed = [json.loads(line)["packet"] for line in
                     (self.directory / "gui_packet_events.jsonl").read_text().splitlines()]
        self.assertEqual(displayed, events)

    def test_idle_station_color_is_not_reset_every_frame(self):
        fake = MagicMock()
        animation = PacketAnimation(fake, dict(x_m=0, y_m=0), [(100, 10)] * 24,
                                    self.directory, compact=True)
        try:
            animation.update_colors()
            self.assertEqual(fake.poi.setColor.call_count, 24)
            animation.update_colors()
            animation.update_colors()
            self.assertEqual(fake.poi.setColor.call_count, 24)
            with patch("packet_demo.time.monotonic", return_value=10):
                animation.show_event(rx(), 2)
                animation.update_colors()
            self.assertEqual(fake.poi.setColor.call_args.args, ("RSU1", (20, 210, 70, 255)))
            with patch("packet_demo.time.monotonic", return_value=11):
                animation.update_colors()
            self.assertEqual(fake.poi.setColor.call_args.args, ("RSU1", animation.base_colors[0]))
        finally:
            animation.close()


if __name__ == "__main__":
    unittest.main()
