"""Multi-RSU view: purple selection is distinct from green actual reception."""

import json
import time

from gui_demo import link_shape
from packet_demo import PacketAnimation


class HandoverGui:
    def __init__(self, traci, initial, stations, duration, delay_ms, directory,
                 overview=False, packet_demo=True):
        self.traci, self.stations = traci, stations
        self.directory, self.overview = directory, overview
        self.delay = delay_ms / 1000
        self.previous_rx = [0] * len(stations)
        self.links = set()
        self.previous_serving = 0
        self.last_change = "Initial selection"
        self.fixed_labels = None
        self.capture_at = {max(0, int(duration / 4) - 1), max(0, int(3 * duration / 4) - 1)}
        view = traci.gui.DEFAULT_VIEW
        traci.gui.setSchema(view, "online_demo")
        if overview:
            (xmin, ymin), (xmax, ymax) = traci.simulation.getNetBoundary()
            traci.gui.setBoundary(view, xmin - 250, ymin - 600, xmax + 250, ymax + 600)
            middle = (xmin + xmax) / 2
            self.fixed_labels = {"serving_banner": (middle, ymax + 440),
                                 "handover_banner": (middle, ymax + 360),
                                 "car_clock": (middle, ymin - 100)}
            self.packet_anchors = {"packet_legend": (middle, ymax + 280),
                                   "packet_clock": (middle, ymax + 200),
                                   "packet_tx_detail": (middle, ymin - 220),
                                   "packet_rx_detail": (middle, ymin - 350)}
        for station in stations:
            i = station["rsu_id"]
            traci.poi.add(f"RSU{i}", station["x_m"], station["y_m"], (100, 140, 190, 255),
                          poiType="", layer=20, width=0.7, height=0.7)
            traci.poi.add(f"station_label_{i}", station["x_m"], station["y_m"] + (100 if overview else 40),
                          (0, 0, 0, 0), poiType=f"R{i}", layer=21, width=.01, height=.01)
        x, y = initial["x_m"], initial["y_m"]
        for name, dy, text in (("serving_banner", 270, "Nearest RSU selection"),
                               ("handover_banner", 230, "Application target only, not L2/cellular handover"),
                               ("car_clock", -80, "Live SUMO vehicle")):
            traci.poi.add(name, x, y + dy, (0, 0, 0, 0), poiType=text,
                          layer=35, width=0.01, height=0.01)
        self.animation = (PacketAnimation(traci, initial,
                          [(s["x_m"], s["y_m"]) for s in stations], directory, compact=True)
                          if packet_demo else None)
        if self.animation and overview:
            self.animation.label_anchor = self.packet_anchors
        self.audit = (directory / "gui_feedback.jsonl").open("w", encoding="utf-8")

    def update(self, state, counters):
        t, selected = state["ns3_time_s"], state["serving_rsu"]
        x, y = state["x_m"], state["y_m"]
        traci = self.traci
        view = traci.gui.DEFAULT_VIEW
        if not self.overview:
            # Follow the vehicle so roads/RSUs farther along the original route become visible.
            traci.gui.setBoundary(view, x - 600, y - 340, x + 600, y + 340)
        if self.previous_serving and selected != self.previous_serving:
            self.last_change = f"SWITCH {t:g}s: RSU{self.previous_serving} -> RSU{selected} (new TX target)"
        received = counters[1::2]
        delta = [now - old for now, old in zip(received, self.previous_rx)]
        colors = []
        for i, station in enumerate(self.stations, 1):
            color = (170, 45, 210, 255) if i == selected else (100, 140, 190, 255)
            colors.append(color)
            if not self.animation or not self.animation.stable:
                traci.poi.setColor(f"RSU{i}", (20, 190, 70, 255) if delta[i - 1] else color)
            suffix = ("*" if self.overview else " [TARGET]") if i == selected else ""
            if i in (selected, self.previous_serving) or not self.previous_serving:
                traci.poi.setType(f"station_label_{i}", f"R{i}" + suffix)
            name = f"received_link_{i}"
            if delta[i - 1]:
                shape = link_shape((x, y), (station["x_m"], station["y_m"]), 1)
                if i in self.links:
                    traci.polygon.setShape(name, shape)
                else:
                    traci.polygon.add(name, shape, (20, 180, 70, 255), fill=True, layer=9)
                    self.links.add(i)
            elif i in self.links:
                traci.polygon.remove(name, layer=9)
                self.links.remove(i)
        target = self.stations[selected - 1]
        shape = link_shape((x, y), (target["x_m"], target["y_m"]), 0.6)
        if self.previous_serving:
            traci.polygon.setShape("selected_link", shape)
        else:
            traci.polygon.add("selected_link", shape, (170, 45, 210, 255), fill=True, layer=8)
        stable = self.animation and self.animation.stable
        traci.vehicle.setColor(state["vehicle_id"], (255, 210, 0, 255) if stable else
                               ((20, 180, 70, 255) if any(delta) else (255, 210, 0, 255)))
        texts = {"serving_banner": f"TARGET RSU{selected:02d} | distance={state['nearest_distance_m']:06.1f}m | sync={t:06.1f}s",
                 "handover_banner": self.last_change,
                 "car_clock": f"ns-3 t={t:g}s | TX={sum(counters[::2])} RX={sum(received)} | purple=selection, green=RX"}
        for name, dy in (("serving_banner", 270), ("handover_banner", 230), ("car_clock", -80)):
            position = self.fixed_labels[name] if self.fixed_labels else (x, y + dy)
            traci.poi.setPosition(name, *position)
            traci.poi.setType(name, texts[name])
        self.audit.write(json.dumps(dict(seq=state["seq"], ns3_time_s=t, serving_rsu=selected,
                                         received=received, newly_received=delta,
                                         link_visible=[n > 0 for n in delta])) + "\n")
        self.audit.flush()
        if self.animation:
            self.animation.base_colors = colors
            self.animation.update(state, counters, self.delay)
        elif self.delay:
            time.sleep(self.delay)
        if state["seq"] in self.capture_at:
            traci.gui.screenshot(view, str(self.directory / f"gui_at_{state['seq'] + 1:03d}s.png"))
        self.previous_rx = list(received)
        self.previous_serving = selected

    def close(self):
        if self.animation:
            self.animation.close()
        self.audit.close()
