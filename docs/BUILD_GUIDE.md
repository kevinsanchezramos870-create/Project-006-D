# Project 006-D v2 — Build Guide: 0.5 lb Payload Quadcopter & Vision Swarm

This is the complete build manual for the **v2** airframe: a compact 7-inch, 4S quadcopter sized to carry and release a **0.5 lb (227 g) payload**. It records 1080p video, streams live video to a browser, and tracks targets from the air. You fly it with 3D-printer-style G-code (`G0 X15 Y10 Z5`), it can fly in a multi-drone swarm, and it has layered failsafes.

v1 was a 3 lb heavy-lift design (650 mm frame, Pixhawk, Pi 4). Its documents are kept in [`docs/archive/`](archive/). v2 is smaller, cheaper and lighter, and it flies longer per battery. It also fixes several software problems from the v1 draft; they are listed in [section 11](#11-what-changed-from-the-3-lb-draft-and-why).

> **Safety first.** A 1.2 kg drone with 7-inch carbon-loaded props can cause serious injury. Do every first test with the props removed. Fly in an open field away from people, and keep a pilot on the RC sticks for every autonomous flight. Read [section 9](#9-legal-checklist-usa) before your first outdoor flight.

---

## Contents
1. [Sizing: how 0.5 lb drives every part choice](#1-sizing-how-05-lb-drives-every-part-choice)
2. [Bill of materials](#2-bill-of-materials)
3. [How the parts affect one another](#3-how-the-parts-affect-one-another)
4. [Assembly, soldering & wiring](#4-assembly-soldering--wiring)
5. [Flight controller firmware & parameters](#5-flight-controller-firmware--parameters)
6. [Raspberry Pi Zero 2 W companion setup](#6-raspberry-pi-zero-2-w-companion-setup)
7. [Software: vision, G-code grid flight, swarm, emergency return](#7-software)
8. [Test ladder: simulator to payload drop](#8-test-ladder-simulator-to-payload-drop)
9. [Legal checklist (USA)](#9-legal-checklist-usa)
10. [Upgrades](#10-upgrades)
11. [What changed from the 3 lb draft, and why](#11-what-changed-from-the-3-lb-draft-and-why)
12. [References](#12-references)

---

## 1. Sizing: how 0.5 lb drives every part choice

The design rule is the same as v1: **at full load, total peak thrust must be at least 2× the all-up weight (AUW).** That keeps hover near 50% throttle and leaves reserve for wind, climbs and stopping a swinging payload.

All numbers below come from the sizing calculator in this repo, so you can re-run them whenever you change a part:

```bash
python -m project006.sizing --parts                    # weight budget + flight time
python -m project006.sizing --battery lipo_4s_4000     # try another battery
python -m project006.sizing --payload-g 300            # try another payload
```

**Weight budget (dry, no battery, no payload): 642 g**

| Part | Grams |
| :--- | ---: |
| 7" carbon frame + hardware (295–315 mm) | 180 |
| 4× 2807 1500–1700KV motors | 208 |
| 4× 7" props | 36 |
| 4-in-1 45–55 A ESC (30.5 mm) | 14 |
| Matek H7A3-SLIM flight controller | 9 |
| M10 GPS + compass on short mast | 30 |
| ELRS 2.4 GHz receiver | 3 |
| SiK 915 MHz telemetry radio (air) | 30 |
| Raspberry Pi Zero 2 W + Camera Module 3 + heatsink | 25 |
| 5 V 3 A BEC for Pi + servo | 8 |
| MG90S servo payload release + mount | 29 |
| Remote ID broadcast module | 10 |
| Landing legs (TPU), wiring, XT60, straps | 60 |

**Performance** (assumes a conservative 1,200 g peak thrust per motor, which is typical for a 4S-rated 2807 on a 7" prop; use your motor's thrust table):

| Battery | Condition | AUW | Thrust : weight | Hover throttle | Hover current | Est. hover time |
| :--- | :--- | ---: | ---: | ---: | ---: | ---: |
| **4S 3000 mAh LiPo (baseline)** | with 0.5 lb | 1,199 g | 4.0 : 1 | ~50% | 13.7 A | **10.5 min** |
| | no payload | 972 g | 4.9 : 1 | ~45% | 10.1 A | 14.2 min |
| 4S 4000 mAh LiPo | with 0.5 lb | 1,299 g | 3.7 : 1 | ~52% | 15.4 A | 12.4 min |
| 4S2P 21700 Li-ion (P42A) | with 0.5 lb | 1,459 g | 3.3 : 1 | ~55% | 18.8 A | 22.8 min |

- **Needed for 2:1 at full load:** 2,398 g total, so about **600 g per motor**. The chosen motors have about double that.
- **Why so much margin?** 7" props are less efficient per gram than v1's 12" props. Big thrust margin lets the motors work at an efficient part of their curve, and it gives strong control authority with a payload swinging underneath.
- **What the calculator is (and isn't):** it uses momentum theory with a propeller figure of merit of 0.55 and a 75% efficient motor+ESC. Treat the flight times as ±20% estimates. After your first hover, compare against the real current in your flight log.
- **Battery warning (the calculator flags this too):** four motors at 100% throttle can draw about 112 A, which is more than a 3000 mAh pack should deliver continuously. Hover and cruise use only 14 A, so this is fine, but avoid long full-throttle climbs.
- **Hard payload limit:** the thrust would allow more, but the **MG90S release servo, the hook and flight time set the practical limit at about 1 lb**. Above 0.5 lb, re-run the calculator and re-tune.

## 2. Bill of materials

Prices are approximate 2026 USD. Any part in the same class works; the "Spec to match" column is what matters.

| # | Part | Recommended example | Spec to match | ~Cost |
| :-: | :--- | :--- | :--- | ---: |
| 1 | Frame | Any 7" long-range carbon frame ([RaceDayQuads](https://www.racedayquads.com/), [GetFPV](https://www.getfpv.com/)) | 295–315 mm wheelbase, 30.5 mm stack holes, room for a 4S 3000 mAh pack on top | $50–90 |
| 2 | Motors ×4 | [EMAX ECO III 2807](https://www.myfpvstore.com/motors/emax-eco-iii-2807-7-fpv-drone-motor-1300kv-1700kv-1500kv) **1500 or 1700KV** | 2806.5–2808 stator, **4S KV (1500–1700)**. 1300KV is meant for 6S and is weak on 4S | $80–110 |
| 3 | Props | [Gemfan](https://www.gemfanhobby.com/) 7035/7040 | 7" 2-blade (most efficient hover) or 3-blade (more thrust, more current); buy spares | $10–15 |
| 4 | ESC | Any 4-in-1 30.5 mm, 45–55 A, 3–6S | Bluejay or AM32 firmware, **current-sensor output**, DShot | $45–70 |
| 5 | Flight controller | [Matek H7A3-SLIM](https://www.mateksys.com/?portfolio=h7a3-slim) | ArduPilot target `MatekH7A3` (in ArduCopter stable) | $60–90 |
| 6 | GPS + compass | [Matek M10Q-5883](https://www.mateksys.com/?portfolio=m10q-5883) on a 5–8 cm mast | u-blox M10, separate compass on I²C | $35–45 |
| 7 | RC link | [ExpressLRS](https://www.expresslrs.org/) 2.4 GHz receiver + ELRS transmitter ([RadioMaster](https://www.radiomasterrc.com/)) | CRSF output; transmitter with at least 2 switches | $80–150 |
| 8 | Telemetry radio | [Holybro SiK V3](https://holybro.com/products/sik-telemetry-radio-v3) **915 MHz** (US) | Long-range MAVLink backup to the ground PC | $45–65 |
| 9 | Companion computer | Raspberry Pi Zero 2 W + 32 GB A1 microSD + stick-on heatsink | 64-bit Raspberry Pi OS Lite | $25 |
| 10 | Camera | Raspberry Pi Camera Module 3 (standard; Wide for more coverage) + **Pi Zero camera cable** | The Zero needs the narrow 22-pin cable. The one in the camera box will not fit | $30 |
| 11 | BEC | 5 V 3 A UBEC, 2–6S input | Powers **only** the Pi + servo (power isolation, section 3) | $8–12 |
| 12 | Payload release | MG90S metal-gear micro servo + hook/latch (3D-printed or a commercial servo release) | Mechanical latch: the servo *unlocks* the load, it does not *hold* it | $5–10 |
| 13 | Remote ID | [Holybro Remote ID](https://holybro.com/products/remote-id) or any module on the [FAA's accepted list](https://uasdoc.faa.gov/listDocs) | MAVLink/OpenDroneID, supported by ArduPilot | $40–60 |
| 14 | Batteries ×2 | 4S 3000 mAh LiPo, XT60 | ~330 g; a realistic rating of ≥ 30C | $70–100 |
| 15 | Charger | Any balance charger + LiPo-safe bag | 4S balance charging, storage-charge mode | $40–70 |
| 16 | Small parts | XT60 pigtail, 14 AWG + 22 AWG silicone wire, **1000 µF 35 V low-ESR capacitor**, TPU landing legs (60–80 mm), battery straps, M3 hardware, heat-shrink, blue Loctite | | $20–30 |

**Cost, honestly:**
- **The drone itself** (items 1–6, 9–13, 16 and one battery): about **$445–$640**.
- **Ground gear** (RC transmitter and receiver, SiK radio pair, a second battery, charger): about **$200–$335** more.
- **Everything:** about **$645–$975**.

That is about the same as v1's list ($570–$830). v1 left out the Remote ID module, which the law requires, and priced clone parts. v2 doesn't save money; it gains a much lighter drone that's easier to carry, longer flight per gram of battery, and safer software. To cut cost, skip the SiK radio and use the Pi's Wi-Fi only (short range, about 100 m), and buy one battery to start.

> **Matek H743-SLIM?** It also works (ArduPilot target `MatekH743-bdshot`), but Matek marked it end-of-life in September 2026. If you already own one, use the port table in [section 5.3](#53-parameters). Any [ArduPilot-supported](https://ardupilot.org/copter/docs/common-autopilots.html) 30.5 mm board works if you remap the `SERIALn` ports to match its documentation.

## 3. How the parts affect one another

**Payload ⇄ thrust ⇄ battery ⇄ flight time.** Hover power rises with weight to the **1.5 power**. Adding 227 g of payload to a 972 g drone (+23% weight) costs about 36% more hover power, which is why flight time drops from 14 to 10.5 minutes. A bigger battery adds flight time but also adds weight, so the gains shrink: 4000 mAh gives only +2 minutes. Li-ion cells store far more energy per gram but deliver less current, so they suit gentle missions.

**Motor KV ⇄ battery voltage ⇄ prop.** Unloaded RPM ≈ KV × volts. On 4S (14.8 V), a 1700KV motor spins about 25,000 RPM unloaded, which is right for a 7" prop. A 1300KV "6S" motor on 4S has too little RPM and runs out of thrust. A 2300KV racing motor would overload on a 7" prop and overheat the ESC.

**ESC ⇄ motors.** Each motor peaks around 28 A. A 45–55 A ESC keeps more than 35% headroom, so it stays cool in summer. Solder the **1000 µF low-ESR capacitor** across the ESC battery pads. It absorbs the voltage spikes from fast motor braking that would otherwise reach the flight controller and the Pi.

**Frame stiffness ⇄ gyro noise ⇄ the vibration (notch) filter.** Props vibrate at a frequency tied to motor RPM. ArduPilot's *harmonic notch* filter removes that frequency from the gyro signal. This build uses the throttle-based version, so it works with any ESC. A stiff carbon frame keeps the vibration in one narrow band that the filter can remove.

**Power isolation (critical).** The Pi Zero 2 W and the servo get their **own 5 V BEC**. A servo jamming against the latch can pull more than 1 A in an instant. On the flight controller's regulator, that sag can reboot the flight controller in mid-air.

**Compass ⇄ current.** 15 A of battery current through the wires makes a magnetic field that skews the compass, and a skewed compass makes the drone circle in GPS modes ("toilet-bowling"). Mount the GPS/compass on a mast, run the battery leads twisted together and away from it, and run **CompassMot** in Mission Planner.

**Flight controller ⇄ Pi (who is in charge).** The Pi never drives the motors. It sends high-level targets ("go to this point", "move north at 1 m/s") over MAVLink. If the Pi crashes, `GUID_TIMEOUT` (3 s) stops the drone, and the pilot can always take over by flipping the mode switch to Loiter.

**Payload ⇄ center of gravity.** Hang the hook exactly under the frame's center. Off-center cargo makes the drone lean, which wastes thrust, and dropping it causes a sudden jump. Descend to 2–3 m before a release so the drop is short and the swing is small.

## 4. Assembly, soldering & wiring

### 4.1 Tools
Temperature-controlled iron (380–400 °C, chisel tip for the XT60 and battery pads), 63/37 rosin-core solder, flux, multimeter, heat-shrink, hex drivers, and a **smoke stopper** (or a current-limited bench supply) for the first power-up.

### 4.2 Power
1. Solder the XT60 pigtail (14 AWG) to the 4-in-1 ESC battery pads. Solder the **1000 µF capacitor** to the same pads, observing polarity, with the leads as short as possible.
2. Solder the **5 V BEC** input to the ESC battery pads, or to an XT60 tap.
3. **Multimeter check before any battery:** measure resistance across the XT60 pins. You should read high resistance (it climbs as the capacitor charges), **never a steady 0 Ω**.
4. Make the first power-up through a smoke stopper.

### 4.3 Motors and order
Mount the motors with M3 screws that **do not touch the windings** (screw length = arm thickness + 3–4 mm) and blue Loctite. Solder the motor wires to the ESC pads.

ArduPilot's Quad-X order:

| Motor | Position | Direction |
| :-: | :--- | :--- |
| 1 | Front-right | CCW |
| 2 | Rear-left | CCW |
| 3 | Front-left | CW |
| 4 | Rear-right | CW |

Most 4-in-1 ESCs are labeled in **Betaflight order** instead. You can either remap the outputs, or set `FRAME_TYPE = 12` (BetaFlightX) and wire it Betaflight-style. **Always confirm with Mission Planner → Optional Hardware → Motor Test, with props off.** Reverse a motor in ESC configurator software, or by swapping two motor wires.

### 4.4 Flight controller ports (Matek H7A3-SLIM)

| FC pads | ArduPilot port | Connect to | Notes |
| :--- | :--- | :--- | :--- |
| TX1 / RX1 | `SERIAL1` (DMA) | Pi Zero 2 W: TX1→**GPIO15 (pin 10)**, RX1→**GPIO14 (pin 8)**, GND→pin 6 | 921,600 baud. TX goes to RX |
| TX2 / RX2 | `SERIAL2` (DMA) | ELRS receiver (TX→RX, RX→TX, 5 V, GND) | CRSF (board default) |
| TX3 / RX3 + SDA/SCL | `SERIAL3` + I²C | M10Q-5883 GPS + compass | Arrow forward |
| TX4 / RX4 | `SERIAL4` | SiK radio | 57,600 baud |
| TX5 / RX5 | `SERIAL5` | Remote ID module | 57,600 baud |
| S1–S4, Curr, GND | Motor outputs | 4-in-1 ESC harness | Curr = ESC current sensor |
| S5 | `SERVO5` | MG90S signal wire | Servo **power** comes from the BEC |

**Pi power:** BEC 5 V to Pi **pin 2 or 4**, GND to **pin 6**. Never connect the FC's 5 V to the Pi.

**Mounting:** stack the FC on soft-mount grommets with its arrow toward the nose. Put the Pi under the stack or behind it. Mount the camera at the front, looking straight down (`tilt_from_nadir_deg = 0`) with the **top of the image toward the nose**. If you tilt it forward, set the angle in `drone.toml`.

### 4.5 Payload release
Bolt the servo latch at the exact center under the frame. Fit landing legs long enough that the hanging payload clears the ground, usually 60–80 mm. **Test by hand:** hang 227 g on the closed hook and shake the frame. The latch must hold with the servo unpowered.

## 5. Flight controller firmware & parameters

### 5.1 Ground station software (Linux, Windows, macOS)
- [Mission Planner](https://ardupilot.org/planner/) (Windows; the most complete ArduPilot setup tool) or [QGroundControl](https://qgroundcontrol.com/) (Linux, Windows, macOS).
- On Linux, for USB access: `sudo usermod -aG dialout $USER`, then log out and back in. If QGroundControl can't connect over USB, remove ModemManager.

### 5.2 Flash
1. Download **ArduCopter stable** for target **`MatekH7A3`** from the [ArduPilot firmware server](https://firmware.ardupilot.org/Copter/stable/).
2. First flash: hold the board's BOOT button while plugging in USB (DFU mode), then flash `arducopter_with_bl.hex`. After that, update from Mission Planner.

### 5.3 Parameters
Load [`config/ardupilot_0.5lb_7in_4S.param`](../config/ardupilot_0.5lb_7in_4S.param). It sets the ports, failsafes, geofence, battery, gripper, Remote ID and guided-mode options, and every line is commented. **Load → write → reboot → load → write**, because some parameters only appear after their `*_ENABLE` is set. Set **`MAV_SYSID`** to a different number on each drone.

Every name in that file was checked against the official **ArduCopter 4.7.1** parameter list. **ArduCopter 4.6 uses older names, and some use centimetres.** On 4.6, change these lines:

| 4.7 name (file) | 4.6 name | 4.6 value |
| :--- | :--- | :--- |
| `MAV_SYSID` | `SYSID_THISMAV` | same |
| `MAV_GCS_SYSID` | `SYSID_MYGCS` | same |
| `RTL_ALT_M,15` | `RTL_ALT` | **1500** (cm) |
| `RTL_ALT_FINAL_M,0` | `RTL_ALT_FINAL` | 0 (cm) |
| `LAND_SPD_MS,0.5` | `LAND_SPEED` | **50** (cm/s) |
| `WP_SPD,5` | `WPNAV_SPEED` | **500** (cm/s) |
| `ARMING_SKIPCHK,0` | `ARMING_CHECK` | **1** (= all checks) |

If you use a **Matek H743-SLIM**, flash `MatekH743-bdshot` and use these ports instead:

| Function | H743-SLIM pads | Parameter |
| :--- | :--- | :--- |
| Pi | TX1/RX1 | `SERIAL2_PROTOCOL 2`, `SERIAL2_BAUD 921` |
| ELRS | TX6/RX6 | `SERIAL7_PROTOCOL 23` **and `BRD_ALT_CONFIG 1`** |
| GPS | TX2/RX2 | `SERIAL3_PROTOCOL 5` |
| SiK | TX7/RX7 | `SERIAL1_PROTOCOL 2`, `SERIAL1_BAUD 57` |
| Remote ID | TX8/RX8 | `SERIAL5_PROTOCOL 2`, `DID_MAVPORT 5` |

### 5.4 Calibrate & tune (props OFF until step 5)
1. Accelerometer, compass and radio calibration, and flight-mode setup. Suggested modes on a 3-position switch: **Loiter / AltHold / Stabilize**. Put **RTL** on its own switch.
2. **Mission Planner → Initial Tuning Parameters:** prop 7 in, 4S, LiPo. It sets the filters, `MOT_THST_EXPO` and starting PIDs for this size of drone. The `.param` file leaves these out on purpose.
3. Battery monitor: set `BATT_AMP_PERVLT` from your ESC's specification, then check the voltage and current against a multimeter and a watt meter.
4. Failsafe bench test (props off, armed): switch off the transmitter and the drone must enter RTL. Unplug the telemetry link and the GCS failsafe must trigger.
5. First hover, then **AUTOTUNE** in calm air ([guide](https://ardupilot.org/copter/docs/autotune.html)). Afterwards, set `INS_HNTCH_REF` to the learned `MOT_THST_HOVER` and refine `INS_HNTCH_FREQ` from the log FFT ([notch guide](https://ardupilot.org/copter/docs/common-imu-notch-filtering.html)).
6. **MAVLink signing** (anti-hijack). In Mission Planner → Setup → Advanced → [MAVLink Signing](https://ardupilot.org/copter/docs/common-MAVLink2-signing.html), set a passphrase. Then put the same passphrase in `P006_MAVLINK_PASSPHRASE` on the Pi and the ground PC, and the scripts will sign their packets. Unsigned packets from anyone else are ignored.

## 6. Raspberry Pi Zero 2 W companion setup

1. **Raspberry Pi Imager → Raspberry Pi OS Lite (64-bit).** In its settings, set the hostname (`drone1`), your user, Wi-Fi and an SSH key.
2. **Serial port.** Run `sudo raspi-config` → Interface Options → Serial Port: login shell **No**, serial hardware **Yes**. Then add this to `/boot/firmware/config.txt`:
   ```ini
   enable_uart=1
   dtoverlay=disable-bt     # gives GPIO14/15 the full UART; the mini-UART is unreliable at 921600
   ```
3. **Software:**
   ```bash
   sudo apt update && sudo apt install -y git python3-venv python3-opencv python3-picamera2 ffmpeg \
        meson ninja-build pkg-config gcc g++ chrony
   git clone https://github.com/kevinsanchezramos870-create/Project-006-D.git ~/Project-006-D
   python3 -m venv --system-site-packages ~/p006-venv          # reuses apt's OpenCV + Picamera2
   ~/p006-venv/bin/pip install -e ~/Project-006-D
   cp ~/Project-006-D/config/drone.example.toml ~/Project-006-D/config/drone.toml   # set drone_id!
   ```
4. **mavlink-router** shares the flight controller with every program:
   ```bash
   git clone https://github.com/mavlink-router/mavlink-router.git && cd mavlink-router
   git submodule update --init --recursive
   meson setup build . && ninja -C build && sudo ninja -C build install
   sudo mkdir -p /etc/mavlink-router
   sudo cp ~/Project-006-D/config/mavlink-router.conf /etc/mavlink-router/main.conf   # set your PC's IP + port
   sudo systemctl enable --now mavlink-router
   ```
5. **Secrets** (one shared set for the whole swarm):
   ```bash
   mkdir -p ~/.config/project006 && cp ~/Project-006-D/config/env.example ~/.config/project006/env
   chmod 600 ~/.config/project006/env && nano ~/.config/project006/env
   ```
6. **Clock sync.** Swarm packets older than 2 s are rejected, and the Pi has no clock battery. In the field without internet, make the ground PC a chrony server: on the PC add `allow 192.168.1.0/24` and `local stratum 10`. On each drone, add `server 192.168.1.100 iburst`.
7. **Services:**
   ```bash
   sudo cp ~/Project-006-D/config/systemd/p006-*.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now p006-guard p006-vision
   journalctl -u p006-vision -f       # watch the logs
   ```

## 7. Software

All tools are plain Python 3.11+ built on [pymavlink](https://github.com/ArduPilot/pymavlink), and they run the same on Linux, Windows and macOS. Install them on a PC with `pip install -e ".[dev]"`, then run `pytest` (32 tests, no hardware needed).

| Command | Runs on | What it does |
| :--- | :--- | :--- |
| `python -m project006.sizing` | anywhere | Weight, thrust and flight-time calculator |
| `python -m project006.grid_commander mission.gcode` | ground PC or Pi | Flies a G-code mission |
| `python -m project006.vision` | Pi | 1080p recording, photos, live view, tracking, follow |
| `python -m project006.swarm` | ground PC | Multi-drone formation and shared tracking |
| `python -m project006.emergency rally / panic / guard` | PC / PC / Pi | Emergency return point, panic button, anomaly guard |

### 7.1 Vision: video, photos, live view, tracking
- **1080p30 H.264 recording** uses the Pi's hardware encoder. A separate 640×360 stream feeds OpenCV, so tracking doesn't slow down recording.
- **Live view and controls** from any browser on the same network: `http://drone1.local:8080/`, plus `/photo`, `/record/start`, `/record/stop` and `/status`.
- **Tracking:** it finds the largest colored blob (red by default; set the HSV ranges in `drone.toml`). It then works out where that point is on the ground from the camera's field of view, the mount tilt, and the drone's roll, pitch, heading and height. The result is a **GPS position**, broadcast to the swarm in a **signed** packet.
- **Follow** (`--follow`): in GUIDED mode, above 4 m, it moves toward the target at up to 2 m/s. If the program stops, the drone stops within `GUID_TIMEOUT`.
- **Bench test on a laptop:** `python -m project006.vision --source 0 --no-mavlink --show` (webcam) or `--source clip.mp4`.

### 7.2 3D-printer-style grid flight (G-code)
Coordinates are in **metres from the takeoff pad**: X = North, Y = East, Z = height. The language works like Marlin G-code:

| Code | Meaning |
| :--- | :--- |
| `M17 Z5` | Arm and take off to 5 m (Marlin: "enable motors"). Locks the hook first |
| `G0/G1 X15 Y10 Z5 A90 F3` | Fly to the point. A = heading (degrees), F = horizontal speed (m/s, stays set until changed) |
| `G90` / `G91` | Absolute / relative coordinates |
| `G4 S2` | Wait 2 s |
| `M100` / `M101` | **Release** payload / lock hook |
| `M114` | Report position |
| `M0 message` | Pause until you press Enter |
| `G28` | Return to launch and land |
| `M18` | Land here and disarm |

```bash
python -m project006.grid_commander missions/delivery_demo.gcode --dry-run   # always first
python -m project006.grid_commander missions/delivery_demo.gcode --connect udpin:0.0.0.0:14551
```
Before arming, the dry run checks **every** move against the fence (100 m radius, 2–60 m altitude), the speed limit and the mission structure. For example, it rejects a mission that ends in the air or drops the payload on the ground. During flight, it waits until the drone actually *arrives* at each point instead of sleeping for a fixed time. If the pilot changes mode, the mission stops. Ctrl+C triggers RTL.

### 7.3 Swarm
```bash
export P006_SWARM_KEY=...                   # the same secret on every drone
python -m project006.swarm --config config/drone.toml --dry-run
python -m project006.swarm --config config/drone.toml
```
- **One shared grid.** Each drone's local X/Y starts where *it* took off, so local coordinates aren't shared. The coordinator uses **drone 1's home as the origin** and sends every drone GPS targets.
- **Altitude layers.** Each drone flies its own altitude (8 / 11 / 14 m by default) and the formation is checked for minimum 3D separation before takeoff. Set each drone's `RTL_ALT_M` to its own layer too (15 / 18 / 21 m), so even a firmware RTL keeps them apart.
- **Shared tracking.** Target reports from all drones are signature-checked, then fused with a **median**, so one drone with a bad estimate can't drag the formation away.
- **Safe ending.** Each drone flies back to above **its own** home on **its own** layer, then they land one at a time. Ctrl+C runs the same sequence; a second Ctrl+C sends every drone to RTL.
- Drones take off one at a time. **Launch all of them from the same level ground**, because each drone's altitude is measured from its own takeoff point.

### 7.4 Emergency return and anti-hijack
Protection comes in four layers, from most to least important:

1. **Firmware failsafes** (in the `.param` file): RC loss → RTL, GCS link loss → RTL, low battery → RTL, critical battery → Land, bad position estimate → Land, geofence breach → RTL.
2. **Emergency return point stored in the flight controller.** Set it with `python -m project006.emergency rally --x 20 --y -5 --alt 15`, or with `--lat/--lon`. It's uploaded as an ArduPilot [rally point](https://ardupilot.org/copter/docs/common-rally-points.html), so RTL goes there even if the Pi or laptop is dead.
3. **MAVLink signing** (section 5.4 step 6). This is what actually stops a stranger from sending your drone commands.
4. **Onboard guard** (`p006-guard` service). It watches for warning signs the firmware doesn't act on: GPS lost or too few satellites, a position "teleport" faster than the drone can fly (the typical signature of GPS spoofing), or the home position changing while airborne (someone redirecting RTL). It responds with **LAND** by default, because RTL relies on GPS. Each response is configurable.

**Panic button:** `python -m project006.emergency panic` (RTL) or `--mode LAND`.

## 8. Test ladder: simulator to payload drop

Don't skip steps. Each one makes the next one safe.

1. **Unit tests:** `pytest`.
2. **Simulator ([ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)):**
   ```bash
   sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:127.0.0.1:14551
   # in the MAVProxy console, enable the simulated gripper:
   param set GRIP_ENABLE 1
   param set SERVO5_FUNCTION 28
   python -m project006.grid_commander missions/delivery_demo.gcode --connect udpin:127.0.0.1:14551 --yes
   ```
   For the swarm, start three instances: `-I0 --sysid 1`, `-I1 --sysid 2`, `-I2 --sysid 3`, each with its own `--out` port.
3. **Bench, props off:** motor order and direction, servo open/close (`M101`/`M100` in a tiny mission, or the Mission Planner servo tab), failsafes, Remote ID visible in the OpenDroneID app.
4. **First hover, no payload:** over grass, in Stabilize then AltHold and Loiter. Check vibration (`VIBE` < 30 m/s²), motor temperatures after landing, and the learned `MOT_THST_HOVER`. Then AUTOTUNE.
5. **Payload ramp:** 100 g, then 227 g. Check hover throttle, current, motor and ESC temperatures, and flight time against [section 1](#1-sizing-how-05-lb-drives-every-part-choice).
6. **Release test:** hover at 1–2 m over soft ground, then `M100`.
7. **G-code missions** in an open field, with the pilot holding the mode switch ready for Loiter.
8. **Vision follow**, then a **two-drone** swarm, then three.

Use [`docs/PREFLIGHT_CHECKLIST.md`](PREFLIGHT_CHECKLIST.md) before every flight.

## 9. Legal checklist (USA)

Not legal advice. Check the current FAA rules before flying.

- **Register the drone.** It weighs more than 250 g ([FAA registration](https://www.faa.gov/uas/getting_started/register_drone)).
- **Remote ID** is required unless you fly at an FAA-recognized identification area (FRIA). That's why a broadcast module is in the parts list ([FAA Remote ID](https://www.faa.gov/uas/getting_started/remote_id)).
- **Recreational flying:** pass TRUST, stay at or below 400 ft, keep the drone in sight, and follow community-based safety guidelines ([FAA recreational flyers](https://www.faa.gov/uas/recreational_flyers)). Anything non-recreational needs [Part 107](https://www.faa.gov/uas/commercial_operators).
- **Payload drops:** never drop anything over people or property in a way that creates a hazard. Drop only over an empty area you control.
- **Swarms:** one pilot flying several drones has extra requirements. Under Part 107, it needs a waiver (§107.35). Check the current rules for your type of flying, and have a visual observer.

## 10. Upgrades
- **Longer flights:** 4S2P 21700 Li-ion gives about 23 min with the payload (run the calculator). Fly gently, because Li-ion can't deliver bursts of high current.
- **Smarter vision:** a Raspberry Pi 5 or an AI camera can run YOLO detection. They're heavier and draw more power, so re-run the sizing first.
- **Flying indoors or without GPS:** an optical-flow sensor plus a downward rangefinder, for X/Y/Z flight in a gym.
- **Obstacle avoidance and ROS 2:** a lightweight lidar plus MAVROS.
- **CAD:** printable hook, camera mount and landing legs belong in `cad/` (planned).

## 11. What changed from the 3 lb draft, and why

| Area | 3 lb draft (v1) | v2 | Why |
| :--- | :--- | :--- | :--- |
| Payload | 3 lb, 650 mm, 12" props, Pixhawk 6C, Pi 4 | **0.5 lb**, 7" props, H7A3-SLIM, Pi Zero 2 W | Sized for the actual mission: lighter, cheaper, more portable |
| Links | Every link was a `google.com/search?q=` redirect | Direct links, all checked | The old links were unreliable and hard to read |
| Grid flight | MAVSDK offboard, armed **without taking off**, fixed `sleep()` per move | ArduPilot GUIDED, real takeoff, waits for arrival, fence-checked G-code | MAVSDK offboard targets PX4; the old flow would not fly on ArduPilot |
| Payload command | `set_actuator()` | ArduPilot gripper (`MAV_CMD_DO_GRIPPER`) | Uses the firmware's payload support, with configurable PWM values |
| Programs ⇄ FC | Every script opened UDP 14540, so they clashed | mavlink-router on TCP 5760, many clients | The vision, guard and GCS programs can run together |
| Camera | `cv2.VideoCapture(0)` + `imshow` on a headless Pi | Picamera2: hardware-encoded 1080p, OpenCV on a low-res stream, browser view | The old code doesn't work with the current Pi camera system and needs a screen |
| Target position | Pixel error × made-up factors, heading ignored | Real camera geometry + attitude, reported as GPS | Correct at any heading, height and camera tilt |
| Swarm grid | Each drone's own local grid (different origins) | One GPS origin, altitude layers, separation check | The old formation was not actually shared, so drones could collide |
| Swarm messages | Plain JSON broadcast | HMAC-signed, timestamped, replay-checked, median fusion | Anyone on the Wi-Fi could steer the swarm |
| "Interception" guard | Checked SYSIDs of commands it could never see | MAVLink signing + GPS-spoof, home-change and link checks + rally points | Real protection instead of a check that could never trigger |
| Safety | RTL only | Geofence, Remote ID, battery arming voltage, staged tests, checklist | Required by law (Remote ID) and good practice |
| Parameters | 4.x names in prose | Commented `.param` file checked against 4.7.1, with a 4.6 table | ArduPilot 4.7 renamed several parameters and changed their units |

## 12. References
- ArduPilot Copter: [failsafes](https://ardupilot.org/copter/docs/failsafe-landing-page.html), [geofence](https://ardupilot.org/copter/docs/common-geofencing-landing-page.html), [GUIDED mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html), [gripper](https://ardupilot.org/copter/docs/common-gripper-servo.html), [rally points](https://ardupilot.org/copter/docs/common-rally-points.html), [Remote ID](https://ardupilot.org/copter/docs/common-remoteid.html), [MAVLink signing](https://ardupilot.org/copter/docs/common-MAVLink2-signing.html), [DShot ESCs](https://ardupilot.org/copter/docs/common-dshot-escs.html), [notch filtering](https://ardupilot.org/copter/docs/common-imu-notch-filtering.html), [initial tuning](https://ardupilot.org/copter/docs/setting-up-for-tuning.html), [AUTOTUNE](https://ardupilot.org/copter/docs/autotune.html), [parameter list](https://ardupilot.org/copter/docs/parameters.html), [companion computer over MAVLink](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html), [SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [MAVLink message signing](https://mavlink.io/en/guide/message_signing.html) · [pymavlink](https://github.com/ArduPilot/pymavlink) · [mavlink-router](https://github.com/mavlink-router/mavlink-router)
- [Picamera2 manual (PDF)](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf) · [Matek H7A3-SLIM](https://www.mateksys.com/?portfolio=h7a3-slim) · [ExpressLRS](https://www.expresslrs.org/)
- FAA: [registration](https://www.faa.gov/uas/getting_started/register_drone) · [Remote ID](https://www.faa.gov/uas/getting_started/remote_id) · [recreational flyers](https://www.faa.gov/uas/recreational_flyers) · [Part 107](https://www.faa.gov/uas/commercial_operators)
