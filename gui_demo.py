"""SUMO GUI overlays driven by the real, one-second ns-3 feedback counters."""

import json
import math
import time
import xml.etree.ElementTree as ET

from packet_demo import PacketAnimation


def write_gui_settings(path):
    root = ET.Element("viewsettings")
    scheme = ET.SubElement(root, "scheme", name="online_demo")
    vehicles = ET.SubElement(scheme, "vehicles", vehicleMode="0", vehicleQuality="2",
                             vehicle_minSize="2", vehicle_exaggeration="8",
                             vehicleName_show="0", vehicleName_size="50",
                             vehicleName_color="20,20,20", vehicleName_constantSize="1")
    colors = ET.SubElement(vehicles, "colorScheme", name="given vehicle/type/route color")
    ET.SubElement(colors, "entry", color="255,210,0")
    ET.SubElement(scheme, "pois", poi_minSize="0", poi_exaggeration="12",
                  poiName_show="0", poiName_size="65", poiName_color="20,20,20",
                  poiName_constantSize="1", poiType_show="1", poiType_size="70",
                  poiType_color="20,20,20", poiType_constantSize="1")
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def link_shape(start, end, half_width=1.5):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return [(start[0] - half_width, start[1] - half_width),
                (start[0] + half_width, start[1] - half_width),
                (start[0] + half_width, start[1] + half_width),
                (start[0] - half_width, start[1] + half_width)]
    nx, ny = -dy / length * half_width, dx / length * half_width
    return [(start[0] + nx, start[1] + ny), (end[0] + nx, end[1] + ny),
            (end[0] - nx, end[1] - ny), (start[0] - nx, start[1] - ny)]


class GuiDemo:
    def __init__(self, traci, initial, offsets, duration, delay_ms, directory, overview=False,
                 packet_demo=False):
        self.traci = traci
        self.vehicle = initial["vehicle_id"]
        self.delay = delay_ms / 1000
        self.overview = overview
        self.previous_rx = [0, 0]
        self.visible_links = [False, False]
        self.directory = directory
        self.stations = [(initial["x_m"] + offset, initial["y_m"] + 10)
                         for offset in offsets]
        self.base_colors = [(40, 110, 230, 255), (230, 115, 20, 255)]
        self.capture_at = {max(0, int(duration / 4) - 1), max(0, int(3 * duration / 4) - 1)}
        view = traci.gui.DEFAULT_VIEW
        if not traci.gui.hasView(view):
            raise RuntimeError("SUMO GUI has no usable view")
        traci.gui.setSchema(view, "online_demo")
        # Focus the original QuickStart road carrying this experiment; the full network stays loaded.
        left = min([initial["x_m"], *[p[0] for p in self.stations]]) - 100
        right = max([initial["x_m"] + initial["speed_mps"] * duration,
                     *[p[0] for p in self.stations]]) + 100
        if overview:
            (xmin, ymin), (xmax, ymax) = traci.simulation.getNetBoundary()
            xpad, ypad = max(100, (xmax - xmin) * 0.1), max(100, (ymax - ymin) * 0.1)
            traci.gui.setBoundary(view, xmin - xpad, ymin - ypad, xmax + xpad, ymax + ypad)
        else:
            traci.gui.setBoundary(view, left, initial["y_m"] - 240, right, initial["y_m"] + 230)
        traci.vehicle.setColor(self.vehicle, (255, 210, 0, 255))
        for i, (x, y) in enumerate(self.stations):
            traci.poi.add(f"RSU{i + 1}", x, y, self.base_colors[i],
                          poiType="", layer=20, width=1, height=1)
            # Annotation-only anchors: the visible RSU and simulated radio stay at (x, y).
            # Move text away from the black road without moving the actual station.
            traci.poi.add(f"status_{i + 1}", x, y + 65, (0, 0, 0, 0),
                          poiType=f"RSU{i + 1} | RX=0 TX=0", layer=21,
                          width=0.01, height=0.01)
        traci.poi.add("car_status", initial["x_m"], initial["y_m"] - 65,
                      (0, 0, 0, 0), poiType="CAR | ns-3 t=0s", layer=21,
                      width=0.01, height=0.01)
        self.audit = (directory / "gui_feedback.jsonl").open("w", encoding="utf-8")
        self.packets = PacketAnimation(traci, initial, self.stations, directory, overview) if packet_demo else None

    def update(self, state, counters):
        received = [counters[1], counters[3]]
        delta = [received[i] - self.previous_rx[i] for i in range(2)]
        active = [value > 0 for value in delta]
        stable = self.packets is not None and self.packets.stable
        self.traci.vehicle.setColor(self.vehicle, (255, 210, 0, 255) if stable else
                                    ((20, 180, 70, 255) if any(active) else (255, 210, 0, 255)))
        for i, position in enumerate(self.stations):
            if not stable:
                self.traci.poi.setColor(f"RSU{i + 1}", (20, 180, 70, 255) if active[i]
                                        else self.base_colors[i])
            status = (f"RSU{i + 1} RX={received[i]}" if self.overview else
                      f"RSU{i + 1} | RX={received[i]} (+{delta[i]}) TX={counters[2 * i]}")
            self.traci.poi.setType(f"status_{i + 1}", status)
            link_id = f"rx_link_{i + 1}"
            if active[i]:
                shape = link_shape((state["x_m"], state["y_m"]), position)
                if self.visible_links[i]:
                    self.traci.polygon.setShape(link_id, shape)
                else:
                    self.traci.polygon.add(link_id, shape, (20, 170, 70, 255), fill=True, layer=10)
            elif self.visible_links[i]:
                self.traci.polygon.remove(link_id, layer=10)
            self.visible_links[i] = active[i]
        self.traci.poi.setPosition("car_status", state["x_m"], state["y_m"] - 65)
        clock = (f"t={state['ns3_time_s']:g}s" if self.overview else
                 f"CAR | ns-3 t={state['ns3_time_s']:g}s | {state['speed_mps']:g}m/s")
        self.traci.poi.setType("car_status", clock)
        self.audit.write(json.dumps({"seq": state["seq"], "ns3_time_s": state["ns3_time_s"],
                                     "received": received, "newly_received": delta,
                                     "link_visible": active}) + "\n")
        self.audit.flush()
        self.previous_rx = received
        if state["seq"] in self.capture_at:
            # SUMO writes a requested screenshot on its NEXT simulationStep.
            filename = self.directory / f"gui_at_{state['seq'] + 1:03d}s.png"
            self.traci.gui.screenshot(self.traci.gui.DEFAULT_VIEW, str(filename))
        # Presentation delay is wall-clock only; both simulation clocks stay at this barrier.
        if self.packets is not None:
            self.packets.update(state, counters, self.delay)
        elif self.delay:
            time.sleep(self.delay)

    def close(self):
        if self.packets is not None:
            self.packets.close()
        self.audit.close()
