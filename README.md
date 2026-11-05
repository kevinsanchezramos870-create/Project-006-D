# Project 006-D: 0.5 lb Payload Quadcopter & Autonomous Vision Swarm

**Project 006-D** is an open-source aerospace and robotics project. It covers designing, building and programming a compact, modular quadcopter (and a multi-drone swarm) that carries and releases a **0.5 lb (227 g) payload**. The drone records and streams video, tracks targets from the air, and flies 3D-printer-style coordinate missions written in G-code.

> **v2 (current):** downsized from the 3 lb heavy-lift draft to a 7-inch, 4S, ~1.2 kg airframe sized for 0.5 lb. The v1 documents are in [`docs/archive/`](docs/archive/). Section 11 of the build guide lists what changed and why.

## Specs

| | |
| :--- | :--- |
| **Payload** | 0.5 lb (227 g), servo hook release (`M100` in G-code) |
| **All-up weight** | ~1,199 g loaded / ~972 g empty |
| **Thrust : weight** | ~4.0 : 1 loaded (the design rule is ≥ 2 : 1) |
| **Flight time (est.)** | ~10.5 min with payload / ~14 min without (4S 3000 mAh); ~23 min with a 4S2P Li-ion pack |
| **Airframe** | 7" carbon, 295–315 mm, 2807 1500–1700KV motors, 45–55 A 4-in-1 ESC |
| **Autopilot** | ArduCopter 4.7 stable on a Matek H7A3-SLIM, M10 GPS + compass, ELRS RC, SiK telemetry, Remote ID |
| **Onboard computer** | Raspberry Pi Zero 2 W + Camera Module 3 (1080p30 recording, live browser view, tracking) |
| **Est. cost** | ~$445–$640 for the drone; ~$645–$975 including radio, charger and spare battery |

## Features
- **3D-printer-style flight:** missions are G-code in metres from the takeoff pad (`M17 Z5`, `G0 X15 Y10 Z5 A90 F3`, `M100`, `G28`). A dry run checks every move against the geofence before the drone arms.
- **Video, photos and live view:** hardware-encoded 1080p recording; full-resolution photos; an annotated live stream at `http://drone1.local:8080/`.
- **Target tracking:** color-blob detection converted to a **GPS position** using camera geometry and the drone's attitude. An optional follow mode moves the drone toward the target.
- **Swarm:** one shared GPS origin, a separate altitude layer per drone, a separation check, **HMAC-signed** target sharing with median fusion, one-at-a-time takeoff and landing.
- **Failsafes and anti-hijack:**
  - ArduPilot failsafes (RC, GCS, battery, position estimate, geofence)
  - an emergency return point stored as a rally point in the flight controller
  - MAVLink 2 signing
  - an onboard guard for GPS spoofing, home-position changes and link loss
  - a panic button
- **Sizing calculator:** change any part and see the new weight, thrust, current and flight time.

## Repository layout

```
docs/BUILD_GUIDE.md          Full build manual: sizing, parts list, wiring, firmware, Pi setup, software, tests, legal
docs/PREFLIGHT_CHECKLIST.md  Print-and-tick checklist for every flight
docs/archive/                v1 (3 lb) drafts, kept for reference
config/ardupilot_0.5lb_7in_4S.param   Commented ArduCopter 4.7 parameters (all names checked against 4.7.1)
config/drone.example.toml    Per-drone settings: connection, fence, camera, tracking, swarm, guard
config/mavlink-router.conf   Lets every onboard program share the flight controller
config/systemd/              Services that start the vision system and guard at boot
missions/*.gcode             Example missions (delivery drop, square survey)
src/project006/              Python package (pymavlink, OpenCV, Picamera2)
tests/                       32 unit tests: sizing, geometry, G-code, signing, swarm safety, guard, vision
```

## Quick start (no drone needed)

```bash
git clone https://github.com/kevinsanchezramos870-create/Project-006-D.git && cd Project-006-D
python3 -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
pytest                                                  # 32 tests
python -m project006.sizing --parts                     # weight and flight-time budget
python -m project006.grid_commander missions/delivery_demo.gcode --dry-run
python -m project006.vision --source 0 --no-mavlink --show   # tracking demo on your webcam
```

Then fly it in the [ArduPilot simulator](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) (build guide section 8) before touching real hardware.

## Roadmap

- [x] Phase 1: sizing for 0.5 lb, parts list, system design, software (v2)
- [ ] Phase 2: frame assembly, power wiring and soldering, BEC isolation, continuity checks
- [ ] Phase 3: ArduCopter flash, parameters, calibration, failsafe bench tests
- [ ] Phase 4: Pi Zero 2 W setup, mavlink-router, video and tracking bench test
- [ ] Phase 5: SITL missions, first hover, AUTOTUNE, payload ramp (100 g → 227 g), drop test
- [ ] Phase 6: G-code missions and vision follow in the field
- [ ] Phase 7: two-drone, then three-drone swarm
- [ ] `cad/`: printable hook, camera mount, landing legs

## Safety & legal
This drone is heavy enough to injure people. In the US it must be **registered** with the FAA and broadcast **Remote ID**. Fly within visual line of sight, at or below 400 ft, away from people, with a pilot on the sticks for every autonomous flight. See build guide section 9.
