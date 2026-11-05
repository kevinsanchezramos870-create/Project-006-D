# Project 006-D Pre-Flight Checklist

Print this page and tick every box, every flight.

## At home (the day before)
- [ ] Batteries balance-charged. Storage-charge any you won't fly.
- [ ] Props: no nicks or cracks, and the right direction on each motor (CW/CCW).
- [ ] Motor screws and prop nuts tight; no screw touching motor windings.
- [ ] Payload hook closes and holds 227 g with the servo unpowered.
- [ ] Mission dry-run passes: `python -m project006.grid_commander <mission> --dry-run`
- [ ] Swarm formation check passes: `python -m project006.swarm --dry-run`
- [ ] Weather: wind under ~15 mph (7 m/s); no rain.
- [ ] Airspace checked (B4UFLY / LAANC), Remote ID registered, TRUST certificate with you.

## At the field (before arming)
- [ ] Open area, no people under the flight path; spectators 30 m back.
- [ ] Transmitter on **first**, then plug in the drone's battery.
- [ ] Battery ≥ 16.4 V (it won't arm below `BATT_ARM_VOLT` 15.6 V).
- [ ] GPS: 3D fix, **≥ 10 satellites**, HDOP < 1.2; home position set.
- [ ] Compass heading on the ground station matches where the nose points.
- [ ] Mode switch works (Loiter / AltHold / Stabilize) and RTL switch works.
- [ ] Ground station connected; MAVLink signing on (if enabled).
- [ ] Rally / emergency return point set if you want something other than home.
- [ ] Live video at `http://drone1.local:8080/`; recording started if wanted.
- [ ] Remote ID shows in the OpenDroneID app.
- [ ] Payload hanging centered and **locked**; landing legs clear it.

## In flight
- [ ] First 10 s: hover at 2 m in Loiter. Check it holds position and sounds smooth.
- [ ] Pilot's hand stays on the mode switch for every autonomous flight.
- [ ] Watch the battery. Land by 14.4 V under load (about 3.6 V per cell) even if it hasn't warned yet.
- [ ] If anything looks wrong: **switch to Loiter**, then land or RTL.

## After landing
- [ ] Disarm, unplug the battery, **then** turn off the transmitter.
- [ ] Feel the motors and ESC: warm is OK; too hot to touch means a problem (props, tune, or overload).
- [ ] Download logs (`.BIN`) and check vibration and current against the sizing estimate.
- [ ] Write down flight time, payload weight and voltage at landing.
