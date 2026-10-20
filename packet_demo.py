"""Read genuine ns-3 packet events and display a retrospective GUI schematic.

No simulationStep calls, invented arrival times, or premature loss decisions.
"""

import json
import math
import time


class PacketEventTail:
    def __init__(self, path, rsu_count=2):
        self.path = path
        self.stream = None
        self.tx = {}
        self.rx = set()
        self.rsu_count = rsu_count
        self.counts = [0] * (2 * rsu_count)
        self.previous_time = -1.0

    def read(self, barrier, counters):
        if self.stream is None:
            if not self.path.exists():
                if any(counters):
                    raise ValueError("Packet event log missing despite nonzero counters")
                return []
            self.stream = self.path.open(encoding="utf-8")
        events = []
        for line in self.stream:
            if not line.endswith("\n"):
                raise ValueError("Incomplete packet event at synchronization barrier")
            event = json.loads(line)
            kind, seq, rsu = event["event"], event["seq"], event["rsu_id"]
            if kind not in ("tx", "rx") or not 1 <= rsu <= self.rsu_count or not isinstance(seq, int) or seq < 0:
                raise ValueError("Invalid packet event identity")
            sent = event["send_time"]
            when = sent if kind == "tx" else event["arrive_time"]
            if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (sent, when))
                    or sent < 0 or when < sent or when > barrier + 1e-8
                    or when < self.previous_time - 1e-8):
                raise ValueError("Packet event time is invalid, future, or out of order")
            if kind == "tx":
                if seq in self.tx:
                    raise ValueError("Duplicate TX identity")
                self.tx[seq] = event
                self.counts[2 * (rsu - 1)] += 1
            else:
                original = self.tx.get(seq)
                delay = event["delay_ms"]
                if (original is None or original["rsu_id"] != rsu
                        or abs(original["send_time"] - sent) > 1e-8
                        or not isinstance(delay, (int, float)) or not math.isfinite(delay)
                        or abs(delay - (when - sent) * 1000) > 1e-5):
                    raise ValueError("RX has no matching TX or invalid arrival delay")
                unique = seq not in self.rx
                if event["unique"] != unique:
                    raise ValueError("RX uniqueness mismatch")
                if unique:
                    self.rx.add(seq)
                    self.counts[2 * (rsu - 1) + 1] += 1
            self.previous_time = when
            events.append(event)
        if self.counts != list(counters):
            raise ValueError(f"Live packet events disagree with STEP counters: {self.counts} != {counters}")
        return events

    def close(self):
        if self.stream is not None:
            self.stream.close()


def packet_position(car, station, progress, rsu):
    """An offset schematic arc, not an RF path or a physical packet position."""
    return (car[0] + (station[0] - car[0]) * progress,
            car[1] + (station[1] - car[1]) * progress
            - (45 + (1 + (rsu - 1) % 2) * 20) * math.sin(math.pi * progress))


class PacketAnimation:
    def __init__(self, traci, initial, stations, directory, overview=False, compact=False, stable=True):
        self.traci = traci
        self.stations = stations
        self.overview = overview
        self.compact = compact
        self.stable = stable
        self.label_anchor = None
        self.markers = {}
        self.detail_pending = {}
        self.station_colors = {}
        self.rx_until = [0.0] * len(stations)
        self.base_colors = [(40, 110, 230, 255) if i == 0 else (230, 115, 20, 255)
                            for i in range(len(stations))]
        self.reader = PacketEventTail(directory / "packets.jsonl", len(stations))
        self.audit = (directory / "gui_packet_events.jsonl").open("w", encoding="utf-8")
        self.frames = (directory / "gui_packet_frames.jsonl").open("w", encoding="utf-8")
        self.views = (directory / "gui_view_frames.jsonl").open("w", encoding="utf-8")
        self.previous_barrier = 0.0
        self.last_tx = ["TX: not started"] * len(stations)
        self.last_rx = ["RX: no arrival yet"] * len(stations)
        middle = sum(p[0] for p in stations) / len(stations)
        self.label("packet_legend", middle, initial["y_m"] + 160,
                   "BLUE = TX | GREEN = RX" if overview else
                   "BLUE dots = TX attempts | GREEN tint = recent actual UDP arrival")
        self.label("packet_clock", middle, initial["y_m"] + 125,
                   "Previous-interval event replay | animation NOT RF path / NOT delay scale")
        if compact:
            self.label("packet_tx_detail", middle, initial["y_m"] - 130, "TX: not started")
            self.label("packet_rx_detail", middle, initial["y_m"] - 170, "RX: no arrival yet")
        else:
            for i, station in enumerate(stations):
                self.label(f"packet_tx_{i}", station[0], initial["y_m"] - 110, self.last_tx[i])
                self.label(f"packet_rx_{i}", station[0], initial["y_m"] - 145, self.last_rx[i])

    def label(self, name, x, y, text):
        self.traci.poi.add(name, x, y, (0, 0, 0, 0), poiType=text,
                           layer=31, width=0.01, height=0.01)

    def show_event(self, event, barrier):
        i = event["rsu_id"] - 1
        if event["event"] == "tx":
            accepted = event["socket_accepted"]
            text = (f"TX #{event['seq']:06d} -> RSU{i + 1:02d} send={event['send_time']:010.6f}s"
                    + ("" if accepted else " SOCKET ERROR"))
            self.last_tx[i] = text
            name = "packet_tx_detail" if self.compact else f"packet_tx_{i}"
            self.set_detail(name, f"TX #{event['seq']}" if self.overview and not self.compact else text)
        else:
            text = (f"RX #{event['seq']:06d} at={event['arrive_time']:010.6f}s "
                    f"delay={event['delay_ms']:09.3f}ms")
            self.last_rx[i] = text
            name = "packet_rx_detail" if self.compact else f"packet_rx_{i}"
            if self.compact:
                text = f"RSU{i + 1:02d} " + text
            self.set_detail(name, f"RX #{event['seq']}" if self.overview and not self.compact else text)
            if self.stable:
                # Visual hold only: never alter the real arrive_time or replay ordering.
                self.rx_until[i] = time.monotonic() + .18
            else:
                self.traci.poi.setColor(f"RSU{i + 1}", (20, 210, 70, 255))
        self.audit.write(json.dumps({"barrier_s": barrier, "packet": event}) + "\n")

    def set_detail(self, name, text):
        if self.stable:
            self.detail_pending[name] = text
        else:
            self.traci.poi.setType(name, text)

    def update_colors(self):
        now = time.monotonic()
        for i, base in enumerate(self.base_colors):
            fraction = min(1.0, max(0.0, (self.rx_until[i] - now) / .18))
            if self.stable:
                fraction = fraction * fraction * (3 - 2 * fraction)
                green = (20, 210, 70, 255)
                color = tuple(round(a + (b - a) * fraction) for a, b in zip(base, green))
            else:
                color = base
            if not self.stable or self.station_colors.get(i) != color:
                self.traci.poi.setColor(f"RSU{i + 1}", color)
                self.station_colors[i] = color

    def update(self, state, counters, delay):
        barrier = state["ns3_time_s"]
        events = self.reader.read(barrier, counters)
        begin = self.previous_barrier
        span = max(1e-9, barrier - begin)
        self.previous_barrier = barrier
        clock = (f"REPLAY {begin:g}..{barrier:g}s | schematic" if self.overview else
                 f"REPLAY events {begin:g}..{barrier:g}s | ns-3 PAUSED at {barrier:g}s | schematic only")
        self.traci.poi.setType("packet_clock", clock)
        car = (state["x_m"], state["y_m"])
        if self.compact:
            for name, offset in (("packet_legend", 180), ("packet_clock", 145),
                                 ("packet_tx_detail", -130), ("packet_rx_detail", -170)):
                position = self.label_anchor[name] if self.label_anchor else (car[0], car[1] + offset)
                self.traci.poi.setPosition(name, *position)
        # Each TX gets its own moving marker. Arrival flashes are independent RX events;
        # a blue marker disappearing never means delivery or loss.
        markers = self.markers if self.stable else {}
        cursor = 0
        frames = max(2, math.ceil(delay * (30 if self.stable else 16)) + 1) if delay else 1
        started = time.monotonic()
        for frame in range(frames):
            phase = 1.0 if frames == 1 else frame / (frames - 1)
            replay_time = begin + phase * span
            if not self.stable:
                self.update_colors()
            while cursor < len(events):
                event = events[cursor]
                when = event["send_time"] if event["event"] == "tx" else event["arrive_time"]
                if when > replay_time + 1e-8:
                    break
                self.show_event(event, barrier)
                if event["event"] == "tx":
                    name = f"packet_dot_{event['seq']}"
                    color = (30, 130, 255, 255) if event["socket_accepted"] else (220, 40, 40, 255)
                    self.traci.poi.add(name, *car, color, poiType="" if self.stable else f"#{event['seq']}",
                                       layer=30, width=0.35, height=0.35)
                    markers[name] = (event, when if self.stable else phase, car)
                cursor += 1
            positions = []
            for name, (event, born, origin) in list(markers.items()):
                age = max(0, replay_time - born) if self.stable else phase - born
                if self.stable:
                    u = min(1.0, age / .4)
                    progress = .9 * u * u * (3 - 2 * u)
                    fade_in = min(1.0, .3 + age / .06)
                    fade_out = min(1.0, max(0.0, (.55 - age) / .15))
                    alpha = round(255 * fade_in * fade_out)
                    rgb = (30, 130, 255) if event["socket_accepted"] else (220, 40, 40)
                    self.traci.poi.setColor(name, (*rgb, alpha))
                else:
                    progress = min(.9, max(0, age) / .35 * .9)
                position = packet_position(origin, self.stations[event["rsu_id"] - 1], progress,
                                           event["rsu_id"])
                self.traci.poi.setPosition(name, *position)
                positions.append({"seq": event["seq"], "x_m": position[0], "y_m": position[1]})
                if (self.stable and age >= .55) or (not self.stable and age > .4 and frame != frames - 1):
                    self.traci.poi.remove(name, layer=30)
                    del markers[name]
            if self.stable:
                self.update_colors()
                if frame == frames - 1:
                    for name, text in self.detail_pending.items():
                        self.traci.poi.setType(name, text)
                    self.detail_pending.clear()
            # A same-zoom redraw does not recenter the camera. Do not round-trip offsets.
            # No SUMO or ns-3 time advancement for animation frames.
            view = self.traci.gui.DEFAULT_VIEW
            zoom = float(self.traci.gui.getZoom(view))
            self.traci.gui.setZoom(view, zoom)
            offset = self.traci.gui.getOffset(view)
            self.views.write(json.dumps({"barrier_s": barrier, "frame": frame,
                                         "zoom": zoom, "offset": list(offset),
                                         "wall_time_s": time.monotonic()}) + "\n")
            self.frames.write(json.dumps({"barrier_s": barrier, "frame": frame,
                                          "replay_time_s": replay_time, "markers": positions}) + "\n")
            remaining = started + delay * (frame + 1) / frames - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        if cursor != len(events):
            raise ValueError("Not all live events were displayed")
        self.audit.flush()
        self.frames.flush()
        self.views.flush()
        if not self.stable:
            for name in markers:
                self.traci.poi.remove(name, layer=30)

    def close(self):
        self.reader.close()
        self.audit.close()
        self.frames.close()
        self.views.close()
