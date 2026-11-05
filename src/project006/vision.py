"""
Onboard vision: 1080p recording, still photos, live browser stream, colour
target tracking, GPS-tagged target reports to the swarm, and optional follow.

On the drone (Pi Zero 2 W + Camera Module 3, headless):
    python -m project006.vision --record

On a laptop with a webcam or a video file (no drone needed):
    python -m project006.vision --source 0 --no-mavlink --show
    python -m project006.vision --source flight.mp4 --no-mavlink

Live view / controls from any browser on the same network:
    http://<drone-ip>:8080/            live annotated video
    http://<drone-ip>:8080/photo       take a full-resolution photo
    http://<drone-ip>:8080/record/start   /record/stop   /status

Design notes (fixes vs. the 3-lb draft):
  * cv2.VideoCapture(0) does not work with the Pi camera on current Raspberry
    Pi OS (libcamera). Picamera2 is used instead: the 1080p stream goes straight
    to the hardware H.264 encoder while a small 640x360 stream feeds OpenCV.
  * No cv2.imshow on the drone (there is no screen) - use the browser stream.
  * Target position uses real camera geometry + the drone's attitude and
    heading instead of fixed fudge factors, and is reported as GPS lat/lon so
    every drone in the swarm agrees on where it is.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from . import config as cfg
from .geometry import CameraModel, offset_latlon, pixel_to_ground_offset_ned


# --------------------------------------------------------------------------- #
# Detection (pure; unit tested)
# --------------------------------------------------------------------------- #
def detect_target(bgr: np.ndarray, hsv_ranges, min_area_px: float):
    """Largest blob inside any of the HSV ranges -> dict or None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in hsv_ranges:
        m = cv2.inRange(hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area_px:
        return None
    mom = cv2.moments(c)
    if mom["m00"] == 0:
        return None
    return {"u": mom["m10"] / mom["m00"], "v": mom["m01"] / mom["m00"], "area": area,
            "bbox": cv2.boundingRect(c)}


# --------------------------------------------------------------------------- #
# Camera backends
# --------------------------------------------------------------------------- #
class PiCamera:
    """Picamera2: main 1920x1080 -> H.264 recorder, lores -> tracking frames."""

    def __init__(self, width, height, record_dir):
        from picamera2 import Picamera2
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FfmpegOutput
        self._FfmpegOutput, self._encoder_cls = FfmpegOutput, H264Encoder
        self.cam = Picamera2()
        conf = self.cam.create_video_configuration(
            main={"size": (1920, 1080)}, lores={"size": (width, height), "format": "YUV420"},
            controls={"FrameRate": 30})
        self.cam.configure(conf)
        self.cam.start()
        self.record_dir = record_dir
        self.encoder = None

    def read(self):
        yuv = self.cam.capture_array("lores")
        return cv2.cvtColor(yuv, cv2.COLOR_YUV420p2BGR)

    def start_recording(self, path: Path):
        self.encoder = self._encoder_cls(bitrate=8_000_000)
        self.cam.start_encoder(self.encoder, self._FfmpegOutput(str(path)), name="main")

    def stop_recording(self):
        if self.encoder:
            self.cam.stop_encoder(self.encoder)
            self.encoder = None

    def photo(self, path: Path):
        req = self.cam.capture_request()
        try:
            req.save("main", str(path))
        finally:
            req.release()

    def close(self):
        self.stop_recording()
        self.cam.stop()


class OpenCVCamera:
    """Webcam index, video file or RTSP URL - for laptops and bench testing."""

    def __init__(self, source, width, height):
        self.cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video source {source!r}")
        self.size = (width, height)
        self.writer = None
        self.last = None

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        self.last = cv2.resize(frame, self.size)
        if self.writer:
            self.writer.write(self.last)
        return self.last

    def start_recording(self, path: Path):
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, self.size)

    def stop_recording(self):
        if self.writer:
            self.writer.release()
            self.writer = None

    def photo(self, path: Path):
        if self.last is not None:
            cv2.imwrite(str(path), self.last)

    def close(self):
        self.stop_recording()
        self.cap.release()


# --------------------------------------------------------------------------- #
# Browser live view + controls
# --------------------------------------------------------------------------- #
class LiveServer:
    def __init__(self, app, port):
        self.app = app
        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def _make_handler(self):
        app = self.app

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _text(self, code, body, ctype="text/plain"):
                data = body.encode() if isinstance(body, str) else body
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/":
                    self._text(200, "<html><body style='margin:0;background:#111'>"
                               "<img src='/stream.mjpg' style='width:100%'></body></html>", "text/html")
                elif self.path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            jpg = app.latest_jpeg
                            if jpg:
                                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                            time.sleep(0.1)
                    except (BrokenPipeError, ConnectionResetError):
                        return
                elif self.path == "/photo":
                    self._text(200, f"saved {app.take_photo()}\n")
                elif self.path == "/record/start":
                    self._text(200, f"recording {app.start_recording()}\n")
                elif self.path == "/record/stop":
                    app.stop_recording()
                    self._text(200, "stopped\n")
                elif self.path == "/status":
                    self._text(200, json.dumps(app.status, indent=2), "application/json")
                else:
                    self._text(404, "not found\n")
        return H


# --------------------------------------------------------------------------- #
# Application
# --------------------------------------------------------------------------- #
class VisionApp:
    def __init__(self, conf, camera, vehicle=None, follow=False, swarm_key=None, log=print):
        self.conf, self.camera, self.vehicle, self.follow, self.log = conf, camera, vehicle, follow, log
        c = conf["camera"]
        self.model = CameraModel(c["width"], c["height"], c["hfov_deg"], c["vfov_deg"], c["tilt_from_nadir_deg"])
        self.record_dir = Path(c["record_dir"]).expanduser()
        self.record_dir.mkdir(parents=True, exist_ok=True)
        self.drone_id = conf["vehicle"]["drone_id"]
        self.recording = None
        self.latest_jpeg = None
        self.status = {"drone": self.drone_id, "recording": None, "target": None}
        self._lock = threading.Lock()
        self.swarm_key = swarm_key
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def _stamp(self):
        return time.strftime("%Y%m%d-%H%M%S")

    def take_photo(self):
        p = self.record_dir / f"drone{self.drone_id}_{self._stamp()}.jpg"
        with self._lock:
            self.camera.photo(p)
        self.log(f"[photo] {p}")
        return p

    def start_recording(self):
        with self._lock:
            if not self.recording:
                self.recording = self.record_dir / f"drone{self.drone_id}_{self._stamp()}.mp4"
                self.camera.start_recording(self.recording)
                self.log(f"[video] recording {self.recording}")
            self.status["recording"] = str(self.recording)
            return self.recording

    def stop_recording(self):
        with self._lock:
            if self.recording:
                self.camera.stop_recording()
                self.log(f"[video] saved {self.recording}")
            self.recording = None
            self.status["recording"] = None

    def _telemetry(self):
        if self.vehicle is None:
            return {"alt": 10.0, "att": (0.0, 0.0, 0.0), "gpos": None, "mode": "BENCH"}
        self.vehicle.pump()
        return {"alt": self.vehicle.rel_alt(), "att": self.vehicle.attitude(),
                "gpos": self.vehicle.global_pos(), "mode": self.vehicle.mode}

    def step(self, frame):
        t = self.conf["tracking"]
        tel = self._telemetry()
        det = detect_target(frame, t["hsv_ranges"], t["min_area_px"])
        target_txt = "no target"
        if det:
            roll, pitch, yaw = tel["att"]
            off = pixel_to_ground_offset_ned(self.model, det["u"], det["v"], tel["alt"], roll, pitch, yaw)
            x, y, w, h = det["bbox"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if off:
                dn, de = off
                target_txt = f"target {dn:+.1f}N {de:+.1f}E m"
                self.status["target"] = {"north_m": round(dn, 2), "east_m": round(de, 2), "area": det["area"]}
                if tel["gpos"]:
                    lat, lon, alt = tel["gpos"]
                    tlat, tlon = offset_latlon(lat, lon, dn, de)
                    self.status["target"].update(lat=tlat, lon=tlon)
                    self._broadcast(tlat, tlon, det["area"], lat, lon, alt)
                if self.follow:
                    self._follow(dn, de, tel)
        else:
            self.status["target"] = None
        hud = (f"D{self.drone_id} {tel['mode']} alt {tel['alt']:.1f}m | {target_txt}"
               f"{' | REC' if self.recording else ''}")
        cv2.putText(frame, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            self.latest_jpeg = jpg.tobytes()
        return det

    def _broadcast(self, tlat, tlon, area, lat, lon, alt):
        if not self.swarm_key:
            return
        from . import swarm_protocol as sp
        self.seq += 1
        pkt = sp.encode(sp.target_report(self.drone_id, self.seq, tlat, tlon, area, lat, lon, alt), self.swarm_key)
        self.sock.sendto(pkt, ("255.255.255.255", self.conf["swarm"]["udp_port"]))

    def _follow(self, dn, de, tel):
        """Velocity toward the target. Only in GUIDED and above min altitude.
        If this process dies, ArduPilot's GUID_TIMEOUT brings the drone to a stop."""
        t = self.conf["tracking"]
        if self.vehicle is None or tel["mode"] != "GUIDED" or tel["alt"] < t["follow_min_alt_m"]:
            return
        vmax = t["follow_max_speed_m_s"]
        vn = max(-vmax, min(vmax, dn * t["follow_gain"]))
        ve = max(-vmax, min(vmax, de * t["follow_gain"]))
        self.vehicle.velocity_ned(vn, ve, 0.0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config")
    ap.add_argument("--source", help="webcam index / video file / RTSP URL (default: Pi camera)")
    ap.add_argument("--connect", help="override vehicle.connection")
    ap.add_argument("--no-mavlink", action="store_true", help="bench mode: assume 10 m altitude, level")
    ap.add_argument("--record", action="store_true", help="start recording immediately")
    ap.add_argument("--follow", action="store_true", help="steer toward the target while in GUIDED")
    ap.add_argument("--show", action="store_true", help="also open a desktop window (laptops only)")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args(argv)

    conf = cfg.load(a.config)
    c = conf["camera"]
    camera = (OpenCVCamera(a.source, c["width"], c["height"]) if a.source is not None
              else PiCamera(c["width"], c["height"], c["record_dir"]))

    vehicle = None
    if not a.no_mavlink:
        from pymavlink import mavutil
        from .mav import Vehicle
        # Onboard computer component, so it does not pretend to be the GCS.
        vehicle = Vehicle(a.connect or conf["vehicle"]["connection"], source_system=conf["vehicle"]["drone_id"],
                          source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
                          expected_sysid=conf["vehicle"]["drone_id"]).connect()

    key = None
    try:
        from .swarm_protocol import load_key
        key = load_key(conf["swarm"]["key_env"])
    except RuntimeError as e:
        print(f"(swarm reports disabled) {e}")

    app = VisionApp(conf, camera, vehicle, follow=a.follow, swarm_key=key)
    LiveServer(app, a.port)
    print(f"Live view: http://<this-device-ip>:{a.port}/   (Ctrl+C to stop)")
    if a.record:
        app.start_recording()
    try:
        while True:
            frame = camera.read()
            if frame is None:
                break
            app.step(frame)
            if a.show:
                cv2.imshow("Project 006-D", frame)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("p"):
                    app.take_photo()
                if k == ord("r"):
                    app.stop_recording() if app.recording else app.start_recording()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop_recording()
        camera.close()
        if vehicle:
            vehicle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
