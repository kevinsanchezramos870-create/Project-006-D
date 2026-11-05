"""
3D-printer-style mission language for the drone.

Units are METRES and seconds (a printer uses millimetres). Coordinates are
relative to the takeoff point: X = North, Y = East, Z = height above takeoff.

    ; comment                     (also "( comment )")
    G90 / G91                     absolute (default) / relative moves
    M17 Z5                        arm and take off to 5 m   (Marlin: "enable motors")
    G0 X15 Y0 Z5 A90 F3           fly to X,Y,Z; A = heading (deg); F = horizontal speed m/s
                                  (F stays in effect until changed, like a printer)
    G1 ...                        same as G0
    G4 S2 | G4 P500               wait 2 s | wait 500 ms
    M101 / M100                   payload grab (lock) / payload release (drop)
    M114                          print current position
    M0 [message]                  pause until the operator presses Enter
    G28                           return to launch (RTL) and land
    M18 (or M84)                  land where you are and disarm

Omitted axes keep their previous value, like a printer. The planner checks
every move against the fence *before* the drone arms.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .geometry import Fence


class GcodeError(ValueError):
    pass


@dataclass
class Step:
    line_no: int
    kind: str                     # takeoff | move | dwell | release | grab | report | pause | rtl | land
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float | None = None
    speed: float | None = None
    seconds: float = 0.0
    text: str = ""
    source: str = field(default="", repr=False)


_WORD = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def _strip_comments(line: str) -> str:
    line = re.sub(r"\(.*?\)", "", line)
    return line.split(";", 1)[0].strip()


def parse(text: str, fence: Fence | None = None, max_speed: float = 8.0) -> list[Step]:
    """Parse and validate a mission. Raises GcodeError with the line number."""
    fence = fence or Fence()
    steps: list[Step] = []
    absolute = True
    airborne = False
    x = y = z = 0.0
    yaw = None

    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comments(raw)
        if not line:
            continue
        head = line.split(None, 1)
        cmd = head[0].upper()
        rest = head[1] if len(head) > 1 else ""
        words = {k.upper(): float(v) for k, v in _WORD.findall(rest)}

        def err(msg):
            raise GcodeError(f"line {line_no}: {msg}  ->  {raw.strip()}")

        if not re.fullmatch(r"[GM]\d+", cmd):
            err(f"unknown command '{cmd}'")

        if cmd == "G90":
            absolute = True
        elif cmd == "G91":
            absolute = False
        elif cmd == "M17":
            if airborne:
                err("M17 (takeoff) while already airborne")
            if "Z" not in words:
                err("M17 needs a takeoff height, e.g. M17 Z5")
            z = words["Z"]
            x = y = 0.0
            v = fence.violation(0, 0, z)
            if v:
                err(v)
            airborne = True
            steps.append(Step(line_no, "takeoff", z=z, source=raw))
        elif cmd in ("G0", "G1"):
            if not airborne:
                err("move before takeoff - add 'M17 Z<height>' first")
            if not words.keys() & {"X", "Y", "Z", "A"}:
                err("move has no X/Y/Z/A word")
            if absolute:
                x, y, z = words.get("X", x), words.get("Y", y), words.get("Z", z)
            else:
                x, y, z = x + words.get("X", 0), y + words.get("Y", 0), z + words.get("Z", 0)
            if "A" in words:
                yaw = words["A"] % 360.0
            speed = words.get("F")
            if speed is not None and not (0.1 <= speed <= max_speed):
                err(f"speed F{speed:g} outside 0.1-{max_speed:g} m/s")
            v = fence.violation(x, y, z)
            if v:
                err(v)
            steps.append(Step(line_no, "move", x, y, z, yaw, speed, source=raw))
        elif cmd == "G4":
            secs = words.get("S", words.get("P", 0) / 1000.0)
            if secs <= 0 or secs > 600:
                err("G4 needs S<seconds> or P<milliseconds> between 0 and 600 s")
            steps.append(Step(line_no, "dwell", x, y, z, seconds=secs, source=raw))
        elif cmd in ("M100", "M101"):
            if cmd == "M100" and not airborne:
                err("M100 (release payload) on the ground")
            steps.append(Step(line_no, "release" if cmd == "M100" else "grab", x, y, z, source=raw))
        elif cmd == "M114":
            steps.append(Step(line_no, "report", x, y, z, source=raw))
        elif cmd in ("M0", "M1"):
            steps.append(Step(line_no, "pause", x, y, z, text=rest.strip(), source=raw))
        elif cmd == "G28":
            if not airborne:
                err("G28 (return home) while on the ground")
            airborne = False
            steps.append(Step(line_no, "rtl", source=raw))
        elif cmd in ("M18", "M84"):
            if not airborne:
                err(f"{cmd} (land) while on the ground")
            airborne = False
            steps.append(Step(line_no, "land", x, y, source=raw))
        else:
            err(f"unsupported command {cmd}")

    if airborne:
        raise GcodeError("mission ends in the air - finish with G28 (return home) or M18 (land)")
    if not steps:
        raise GcodeError("mission is empty")
    return steps


def describe(steps: list[Step]) -> str:
    out = []
    for s in steps:
        if s.kind == "takeoff":
            d = f"arm + take off to Z={s.z:g} m"
        elif s.kind == "move":
            d = f"fly to X={s.x:g} Y={s.y:g} Z={s.z:g}"
            if s.yaw_deg is not None:
                d += f" heading {s.yaw_deg:g} deg"
            if s.speed:
                d += f" at {s.speed:g} m/s"
        elif s.kind == "dwell":
            d = f"hold {s.seconds:g} s"
        elif s.kind == "pause":
            d = f"pause for operator ({s.text or 'press Enter'})"
        else:
            d = {"release": "RELEASE payload", "grab": "lock payload hook", "report": "report position",
                 "rtl": "return to launch + land", "land": "land here + disarm"}[s.kind]
        out.append(f"  line {s.line_no:>3}: {d}")
    return "\n".join(out)
