# Project 006-D: Step-by-Step Quadcopter & Swarm Build Guide

## 1. Physics Sizing for a 3-lb (1.36 kg) Payload
Following the **2:1 Thrust-to-Weight Rule**, lifting a 1,360 g payload + 950 g airframe + 500 g 4S battery (**2,810 g All-Up Weight**) requires **>=5,620 g peak thrust** (>=1,405 g/motor) using a 500–650mm frame, 11"–12" propellers, and 2216 880KV / 3508 700KV motors on 4S (14.8V).

## 2. Bill of Materials (BOM) & Where to Source
| Component | Recommended Spec | Cost | Source |
| :--- | :--- | :--- | :--- |
| **Frame** | [Tarot 650 Sport](https://www.getfpv.com/tarot-650-sport-quadcopter-with-retractable-landing-gear.html) or [S500 PCB Frame](https://www.aliexpress.com/item/32302196734.html) | $25–$110 | [GetFPV](https://www.getfpv.com/) / [AliExpress](https://www.aliexpress.com/item/32302196734.html) |
| **Motors (x4)** | [SunnySky X2216 880KV](https://www.readymaderc.com/products/details/85810-sunnysky-x2216-brushless-motors-long-shaft-version-880kv) + 11x4.7" / 12x3.8" Props | $85–$110 | [ReadyMadeRC](https://www.readymaderc.com/products/details/85810-sunnysky-x2216-brushless-motors-long-shaft-version-880kv) / [RaceDayQuads](https://www.racedayquads.com/) |
| **ESCs (x4)** | [Hobbywing XRotor 40A 3–6S BLDC ESCs](https://www.hobbywingdirect.com/products/xrotor-40a-esc) | $55–$65 | [Hobbywing Direct](https://www.hobbywingdirect.com/products/xrotor-40a-esc) |
| **Autopilot & GPS** | [Holybro Pixhawk 6C](https://holybro.com/products/pixhawk-6c) + [u-blox NEO-M8N GPS/Compass](https://www.readytosky.com/e_productshow/?797-High-Quality-Ublox-NEO-M8N-GPS-With-Compass-For-APMPixhawk-797.html) | $75–$190 | [Holybro](https://holybro.com/products/pixhawk-6c) / [Readytosky](https://www.readytosky.com/e_productshow/?797-High-Quality-Ublox-NEO-M8N-GPS-With-Compass-For-APMPixhawk-797.html) |
| **Vision & Swarm PC** | [Raspberry Pi 4 (4GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) + [Camera Module 3 (12MP)](https://www.raspberrypi.com/products/camera-module-3/) | $80 | [Raspberry Pi](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) |
| **Radios & Dropper** | [Holybro SiK Radio V3](https://holybro.com/products/sik-telemetry-radio-v3) + [FlySky FS-i6X](https://www.aliexpress.com/item/32370714787.html) + [MG996R Dropper](https://rcdrone.top/products/payload-drop-release-servo-parabolic-switch-releaser) | $110–$130 | [Holybro](https://holybro.com/products/sik-telemetry-radio-v3) / [RCDrone](https://rcdrone.top/products/payload-drop-release-servo-parabolic-switch-releaser) |
| **Battery & Power** | [GNB 4S 5000mAh 50C LiPo](https://www.quadjunkie.co.nz/product/gnb-5000mah-14-8v-4s-50c-xt60-lipo-battery-pack/) + IMAX B6 + 2x 5V/3A UBECs | $85–$110 | [Quad Junkie](https://www.quadjunkie.co.nz/product/gnb-5000mah-14-8v-4s-50c-xt60-lipo-battery-pack/) |

## 3. How Materials Affect One Another
* **Frame <-> Props <-> IMU:** 500–650mm wheelbase clears 11"–12" props; carbon arms prevent 30–80Hz flex vibrations from corrupting the Pixhawk gyro under a 3-lb load.
* * **Motors <-> Props <-> ESCs:** `880KV` on 4S (`14.8V`) with low-pitch 11x4.7" props maximizes thrust/watt (`g/W`) while keeping peak current (~28A) safely below the 40A ESC limit.
  * * **Battery & Dual 5V UBECs <-> Avionics & Servo:** Never power the Pi 4 or MG996R payload servo from the Pixhawk logic rail—a stalled 3-lb servo pulls 2.5A and browns out the flight controller. Use UBEC #1 for the Pi 4 and UBEC #2 for the Pixhawk `AUX` servo rail.
    * * **Pixhawk <-> GPS Mast <-> Raspberry Pi 4:** Mount the M8N GPS/Compass on a 15cm mast away from high-current PDB wires to prevent magnetic interference. The Pi 4 runs OpenCV vision and sends setpoints over `TELEM2` UART via MAVLink while Pixhawk runs 400Hz attitude loops.
     
      * ## 4. Step-by-Step Assembly, Soldering & Software Setup (Linux & Windows)
      * 1. **Solder PDB:** At 380°C with 60/40 rosin solder, tin PDB pads and solder the 12AWG XT60 lead, 4x 40A ESCs, and 2x 5V UBECs. **Verify open circuit (`infinite Ohms`) across XT60 `+`/`-` with a multimeter before plugging in a battery.**
        2. 2. **Motors & Spin Order (`Quad-X`):** Bolt motors with blue Loctite. Connect 3.5mm bullets: **M1 (Front-Right)=CCW, M2 (Rear-Left)=CCW, M3 (Front-Left)=CW, M4 (Rear-Right)=CW** (swap any 2 phase wires to reverse).
           3. 3. **Pixhawk, Pi 4 & Servo Wiring:** Foam-mount Pixhawk at CG (arrow forward), GPS on mast (`GPS1`/`I2C`), ESCs on `MAIN OUT 1–4`, MG996R dropper on `AUX OUT 1` (`SERVO9_FUNCTION=28`, powered by UBEC #2). Wire `TELEM2` (`SERIAL2_PROTOCOL=2, SERIAL2_BAUD=921`) [to Pi 4 GPIO](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html): `TX->Pin 10 (RXD)`, `RX->Pin 8 (TXD)`, `GND->Pin 6`.
              4. 4. **Linux & Windows Software:** Install [QGroundControl](https://qgroundcontrol.com/) and flash [ArduPilot Copter](https://ardupilot.org/copter/). On Pi 4, install `python3-opencv mavsdk pymavlink` and route telemetry: `mavproxy.py --master=/dev/serial0,921600 --out=udp:127.0.0.1:14540 --out=udp:192.168.1.100:14550 --daemon`.
                
                 5. ## 5. Vision Tracking, Video Recording & Photo Capture (`vision_tracker.py`)
                 6. ```python
                    import cv2, time, socket, json, numpy as np
                    from pymavlink import mavutil

                    mav = mavutil.mavlink_connection("udpin:127.0.0.1:14540")
                    mav.wait_heartbeat()
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    cap = cv2.VideoCapture(0)
                    writer = cv2.VideoWriter("flight.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (640, 480))

                    while True:
                        ret, frame = cap.read()
                        if not ret: break
                        writer.write(frame)
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        cnts, _ = cv2.findContours(cv2.inRange(hsv, (0,120,70), (10,255,255)), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if cnts:
                            x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
                            ex, ey = ((x + w/2) - 320)/320.0, (240 - (y + h/2))/240.0
                            mav.mav.set_position_target_local_ned_send(0, mav.target_system, mav.target_component,
                                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000011111000111, 0,0,0, ey, ex, 0, 0,0,0, 0,0)
                            sock.sendto(json.dumps({"id": 1, "target_xy": [ey*10, ex*10]}).encode(), ("255.255.255.255", 5005))
                        if (cv2.waitKey(1) & 0xFF) == ord("p"): cv2.imwrite(f"snap_{int(time.time())}.jpg", frame)
                    ```

                    ## 6. 3D Coordinate Grid (`X, Y, Z`) & Swarm Control (`grid_and_swarm.py`)
                    ```python
                    import asyncio
                    from mavsdk import System
                    from mavsdk.offboard import PositionNedYaw

                    async def fly_grid(port=14540, offset=(0.0, 0.0, 6.0)):
                        drone = System()
                        await drone.connect(system_address=f"udp://:{port}")
                        await drone.action.arm()
                        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))
                        await drone.offboard.start()
                        ox, oy, oz = offset
                        for (x, y, z) in [(0,0,oz), (15,0,oz), (15,10,3), (0,0,oz)]:
                            await drone.offboard.set_position_ned(PositionNedYaw(x+ox, y+oy, -abs(z), 0))
                            await asyncio.sleep(6)
                        await drone.action.set_actuator(1, -1.0)  # Release 3-lb payload
                        await drone.action.land()

                    asyncio.run(fly_grid())
                    ```

                    ## 7. Emergency Return & Anti-Interception Failsafe (`emergency_watchdog.py`)
                    Configure ArduPilot `FS_THR_ENABLE=1`, `FS_GCS_ENABLE=1`, `BATT_FS_LOW_ACT=2`, `RTL_ALT=1500` and run:
                    ```python
                    import time
                    from pymavlink import mavutil

                    mav = mavutil.mavlink_connection("udpin:127.0.0.1:14540")
                    mav.wait_heartbeat()
                    last_hb, safe_xyz = time.time(), (0.0, 0.0, 12.0)

                    while True:
                        msg = mav.recv_match(blocking=True, timeout=0.5)
                        if msg and msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == 255:
                            last_hb = time.time()
                        if (msg and msg.get_type() == "COMMAND_LONG" and msg.get_srcSystem() not in (1, 255)) or (time.time() - last_hb > 3.5):
                            mav.set_mode(mav.mode_mapping().get("GUIDED", 4))
                            mav.mav.set_position_target_local_ned_send(0, mav.target_system, mav.target_component,
                                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, safe_xyz[0], safe_xyz[1], -safe_xyz[2], 0,0,0, 0,0,0, 0,0)
                            mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,0,0,0,0,0,0,0)
                            break
                    ```

                    ## 8. Resource Sites & Official Documentation
                    * [ArduPilot Copter Docs](https://ardupilot.org/copter/) | [ArduPilot Failsafes](https://ardupilot.org/copter/docs/failsafe-landing-page.html) | [Raspberry Pi MAVLink Guide](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)
                    * * [PX4 Autopilot](https://px4.io/) | [QGroundControl (Linux/Windows)](https://qgroundcontrol.com/) | [MAVSDK-Python NED Guide](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/offboard_position_ned.py)
                      * * [OpenCV Docs](https://docs.opencv.org/) | [Pymavlink](https://github.com/ardupilot/pymavlink) | [ROS 2 & MAVROS](https://docs.ros.org/en/humble/p/mavros/) | [GetFPV](https://www.getfpv.com/) | [RaceDayQuads](https://www.racedayquads.com/) | [Holybro](https://holybro.com/products/pixhawk-6c)
