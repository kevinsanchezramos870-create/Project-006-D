"""
Propulsion, weight and flight-time sizing for Project 006-D.

Run it:
    python -m project006.sizing                     # default 0.5 lb build
    python -m project006.sizing --battery lipo_4s_4000
    python -m project006.sizing --payload-g 300 --json

The flight-time model is simple momentum theory (ideal hover power divided by a
propeller figure of merit and a motor+ESC efficiency). It is a planning tool,
not a substitute for measuring: log real current draw in a hover test and
compare. Every number here is an editable assumption.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field

G = 9.80665          # m/s^2
RHO_SEA_LEVEL = 1.225  # kg/m^3
LB_TO_G = 453.59237


@dataclass
class Battery:
    name: str
    cells: int
    capacity_mah: float
    grams: float
    max_continuous_a: float          # realistic, not the label's inflated "C" figure
    nominal_cell_v: float = 3.7      # 3.7 LiPo, 3.6 Li-ion
    usable_fraction: float = 0.80    # land with ~20% left (protects the pack)

    @property
    def nominal_v(self) -> float:
        return self.cells * self.nominal_cell_v

    @property
    def usable_wh(self) -> float:
        return self.nominal_v * self.capacity_mah / 1000.0 * self.usable_fraction


BATTERIES = {
    # Realistic continuous currents (~30C LiPo, cell datasheet for Li-ion).
    "lipo_4s_3000": Battery("4S 3000 mAh LiPo", 4, 3000, 330, 90),
    "lipo_4s_4000": Battery("4S 4000 mAh LiPo", 4, 4000, 430, 120),
    "liion_4s2p_8400": Battery("4S2P 21700 Li-ion (Molicel P42A)", 4, 8400, 590, 90,
                               nominal_cell_v=3.6, usable_fraction=0.85),
}


@dataclass
class Design:
    payload_g: float = 0.5 * LB_TO_G
    prop_diameter_in: float = 7.0
    motors: int = 4
    # Conservative full-throttle static thrust per motor for a 2807 1500-1700KV
    # motor on 4S with a 7" 3-blade prop. Replace with your motor's thrust table.
    max_thrust_per_motor_g: float = 1200.0
    max_current_per_motor_a: float = 28.0
    esc_rating_a: float = 45.0               # per channel, continuous
    figure_of_merit: float = 0.55            # small multirotor props: ~0.5-0.65
    drive_efficiency: float = 0.75           # motor * ESC
    avionics_w: float = 5.0                  # FC + GPS + Pi Zero 2 W + radios
    air_density: float = RHO_SEA_LEVEL
    battery: Battery = field(default_factory=lambda: BATTERIES["lipo_4s_3000"])
    dry_parts_g: dict = field(default_factory=lambda: dict(DEFAULT_DRY_PARTS_G))

    @property
    def dry_g(self) -> float:
        return float(sum(self.dry_parts_g.values()))


# Everything except the battery and payload. Grams, typical published weights.
DEFAULT_DRY_PARTS_G = {
    "7in carbon frame + hardware (295-315 mm)": 180,
    "4x 2807 1500-1700KV motors": 208,
    "4x 7in props": 36,
    "4-in-1 45-55A ESC (30.5 mm)": 14,
    "Matek H7A3-SLIM flight controller": 9,
    "M10 GPS + compass on short mast": 30,
    "ELRS 2.4 GHz receiver": 3,
    "SiK 915 MHz telemetry radio (air)": 30,
    "Raspberry Pi Zero 2 W + Camera Module 3 + heatsink": 25,
    "5V 3A BEC for Pi + servo": 8,
    "MG90S servo payload release + mount": 29,
    "Remote ID broadcast module": 10,
    "Landing legs (TPU), wiring, XT60, straps": 60,
}


def hover_power_w(total_mass_g: float, d: Design) -> float:
    """Electrical power to hover at total_mass_g (all rotors + avionics)."""
    thrust_per_rotor_n = total_mass_g / 1000.0 * G / d.motors
    radius_m = d.prop_diameter_in * 0.0254 / 2.0
    disk_area = math.pi * radius_m ** 2
    ideal = thrust_per_rotor_n ** 1.5 / math.sqrt(2.0 * d.air_density * disk_area)
    shaft = ideal / d.figure_of_merit
    electrical = shaft / d.drive_efficiency
    return electrical * d.motors + d.avionics_w


def analyze(d: Design) -> dict:
    b = d.battery
    empty_auw = d.dry_g + b.grams
    loaded_auw = empty_auw + d.payload_g
    total_thrust = d.max_thrust_per_motor_g * d.motors

    def flight(auw):
        p = hover_power_w(auw, d)
        hover_fraction = auw / total_thrust
        return {
            "auw_g": round(auw),
            "thrust_to_weight": round(total_thrust / auw, 2),
            "hover_thrust_fraction": round(hover_fraction, 2),
            # thrust ~ rpm^2 ~ throttle^2 for a fixed-pitch prop
            "approx_hover_throttle": round(math.sqrt(hover_fraction), 2),
            "hover_power_w": round(p, 1),
            "hover_current_a": round(p / b.nominal_v, 1),
            "hover_minutes": round(b.usable_wh / p * 60.0, 1),
        }

    peak_current = d.max_current_per_motor_a * d.motors
    warnings = []
    loaded = flight(loaded_auw)
    if loaded["thrust_to_weight"] < 2.0:
        warnings.append("Loaded thrust-to-weight is below 2:1. Use bigger motors/props or less weight.")
    if d.max_current_per_motor_a > 0.8 * d.esc_rating_a:
        warnings.append("ESC has less than 20% current headroom over the motor's max current.")
    if peak_current > b.max_continuous_a:
        warnings.append(
            f"Full-throttle draw ({peak_current:.0f} A) exceeds the battery's realistic "
            f"continuous rating ({b.max_continuous_a:.0f} A). Fine for hover/cruise, but "
            "avoid sustained full-throttle climbs, or lower MOT_THST_MAX / set BATT_* current limits.")

    return {
        "payload_g": round(d.payload_g),
        "payload_lb": round(d.payload_g / LB_TO_G, 2),
        "dry_g": round(d.dry_g),
        "battery": b.name,
        "battery_g": b.grams,
        "total_peak_thrust_g": round(total_thrust),
        "required_thrust_2to1_g": round(2 * loaded_auw),
        "required_thrust_per_motor_g": round(2 * loaded_auw / d.motors),
        "max_payload_at_2to1_g": round(total_thrust / 2.0 - empty_auw),
        "peak_current_a": round(peak_current),
        "loaded": loaded,
        "empty": flight(empty_auw),
        "warnings": warnings,
    }


def format_report(r: dict) -> str:
    L, E = r["loaded"], r["empty"]
    lines = [
        f"Payload: {r['payload_g']} g ({r['payload_lb']} lb) | Battery: {r['battery']} ({r['battery_g']:.0f} g)",
        f"Dry airframe: {r['dry_g']} g | Peak thrust (4 motors): {r['total_peak_thrust_g']} g",
        f"Needed for 2:1 at full load: {r['required_thrust_2to1_g']} g total "
        f"({r['required_thrust_per_motor_g']} g per motor)",
        f"Max payload while keeping 2:1: {r['max_payload_at_2to1_g']} g "
        f"({r['max_payload_at_2to1_g'] / LB_TO_G:.2f} lb)",
        "",
        "| Condition | AUW (g) | T/W | Hover throttle | Hover power | Hover current | Est. hover time |",
        "| :--- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for label, f in (("With 0.5 lb payload" if abs(r["payload_lb"] - 0.5) < 0.01 else "With payload", L),
                     ("No payload", E)):
        lines.append(
            f"| {label} | {f['auw_g']} | {f['thrust_to_weight']}:1 | ~{f['approx_hover_throttle']:.0%} | "
            f"{f['hover_power_w']} W | {f['hover_current_a']} A | {f['hover_minutes']} min |")
    if r["warnings"]:
        lines.append("")
        lines.extend(f"WARNING: {w}" for w in r["warnings"])
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--payload-g", type=float, default=0.5 * LB_TO_G)
    ap.add_argument("--battery", choices=sorted(BATTERIES), default="lipo_4s_3000")
    ap.add_argument("--thrust-per-motor-g", type=float, default=Design.max_thrust_per_motor_g)
    ap.add_argument("--prop-in", type=float, default=Design.prop_diameter_in)
    ap.add_argument("--json", action="store_true", help="print machine-readable JSON")
    ap.add_argument("--parts", action="store_true", help="list the dry-weight budget")
    a = ap.parse_args(argv)

    d = Design(payload_g=a.payload_g, battery=BATTERIES[a.battery],
               max_thrust_per_motor_g=a.thrust_per_motor_g, prop_diameter_in=a.prop_in)
    r = analyze(d)
    if a.json:
        print(json.dumps(r, indent=2))
        return 0
    if a.parts:
        for name, g in d.dry_parts_g.items():
            print(f"{g:5.0f} g  {name}")
        print(f"{d.dry_g:5.0f} g  TOTAL (dry, no battery, no payload)\n")
    print(format_report(r))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
