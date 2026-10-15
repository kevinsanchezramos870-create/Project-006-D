# Project-006-D
Modular Open-Source Heavy-Lift (3 lb Payload) Quadcopter &amp; Autonomous Vision Swarm Platform (ArduPilot / PX4 / MAVSDK / OpenCV)
# Project 006-D: Modular Heavy-Lift Quadcopter & Autonomous Vision Swarm

**Project 006-D** is an ongoing open-source aerospace and robotics engineering project to design, build, and program a cost-effective, modular quadcopter and multi-drone swarm platform from scratch.

## Mission Specifications & Core Capabilities

* **Heavy-Lift Payload Capacity:** Carries and releases objects up to **3 lbs (1.36 kg)** via a belly-mounted [MG996R metal-gear servo release mechanism](https://rcdrone.top/products/payload-drop-release-servo-parabolic-switch-releaser) while maintaining a safe 2:1 thrust-to-weight ratio (~5,620 g total peak thrust across four motors).
* * **Live Video, Recording & Photography:** Uses an onboard [Raspberry Pi 4 Model B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) and [Raspberry Pi Camera Module 3 (12MP)](https://www.raspberrypi.com/products/camera-module-3/) to stream low-latency live video, record 1080p `.mp4` flight footage, and capture high-resolution `.jpg` still photos.
  * * **3D-Printer-Style Coordinate Grid Navigation (`X, Y, Z`):** Accepts Cartesian local North-East-Down (NED) meter coordinates `(X, Y, Z, Yaw)` from a computer—similar to 3D printer G-code—allowing precise point-to-point positioning relative to the takeoff origin `(0, 0, 0)`.
    * * **Multi-Drone Swarm & Sky-to-Ground Object Tracking:** Coordinates multiple drones simultaneously over a shared 3D coordinate grid while running [OpenCV](https://docs.opencv.org/) computer vision on each drone to detect, track, and broadcast moving ground targets across the swarm.
      * * **Emergency Return-to-Position & Anti-Interception Failsafe:** Combines hardware [ArduPilot Copter failsafes](https://ardupilot.org/copter/docs/failsafe-landing-page.html) with an onboard Python security watchdog that detects telemetry signal loss or unauthorized MAVLink `SYSID` command interception and automatically flies the drone back to a computer-defined safe coordinate.
        * * **100% Open-Source & Cross-Platform (Linux & Windows):** Runs on [ArduPilot Copter](https://ardupilot.org/copter/) / [PX4 Autopilot](https://px4.io/), [QGroundControl](https://qgroundcontrol.com/), [MAVLink](https://mavlink.io/en/), [MAVSDK-Python](https://mavsdk.mavlink.io/), [Pymavlink](https://github.com/ardupilot/pymavlink), [OpenCV](https://docs.opencv.org/), and [ROS 2](https://docs.ros.org/).
         
          * ## Documentation & Repository Structure
         
          * * **[`docs/BUILD_GUIDE.md`](docs/BUILD_GUIDE.md)** — **Complete Step-by-Step Engineering & Build Manual:** Includes the full Bill of Materials (BOM) and sourcing links, a detailed component-interaction tutorial, soldering and wiring instructions, cross-platform Linux/Windows software setup, and complete Python scripts for vision tracking, 3D coordinate grid navigation, swarm control, and emergency failsafe return.
            * * `src/` *(Planned)* — Standalone onboard and ground-station Python scripts (`vision_tracker_and_recorder.py`, `grid_commander_ned.py`, `swarm_grid_and_vision_coordinator.py`, `emergency_return_watchdog.py`).
              * * `cad/` *(Planned)* — 3D-printable mounts for the Raspberry Pi 4, Camera Module 3 vibration damper, and 3-lb payload release hook.
                * * `config/` *(Planned)* — Baseline ArduPilot `.param` configuration files and PID/filter tuning logs.
                 
                  * ## Project 006-D Build Roadmap & Status
                 
                  * - [x] Phase 1: Propulsion sizing (2:1 thrust-to-weight math for 3-lb payload), BOM sourcing, and system architecture documentation
                    - [ ] - [ ] Phase 2: Airframe assembly, PDB & ESC soldering, continuity verification, and dual 5V/3A UBEC power isolation
                    - [ ] - [ ] Phase 3: Pixhawk flight controller & u-blox M8N GPS/Compass calibration in QGroundControl (Linux & Windows)
                    - [ ] - [ ] Phase 4: Raspberry Pi 4 companion computer UART (`TELEM2`) integration, live video streaming, and OpenCV object tracking test
                    - [ ] - [ ] Phase 5: Single-drone 3D coordinate grid (`X, Y, Z`) flight testing, 3-lb payload drop test, and Emergency Return watchdog validation
                    - [ ] - [ ] Phase 6: Multi-drone swarm formation flight and collaborative sky object tracking
