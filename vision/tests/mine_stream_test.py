"""
Live MJPEG stream of the IMX500 camera with mine-detection boxes burned in.

Runs headless on Pi OS Lite (no desktop needed) and serves the annotated feed
over HTTP so you can watch it in a browser on another machine:

    http://<pi-ip>:8000/

Reuses RPICamera for camera setup and detection, so what you see on screen is
exactly what the flight code sees. Run from anywhere; model/label paths in the
config are resolved relative to vision/.
"""

import argparse
import io
import json
import os
import socket
import sys
import threading
import time
from http import server
from urllib.parse import urlparse

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import cv2
except ImportError:  # pragma: no cover - environment guard
    sys.exit(
        "OpenCV is required to draw the overlay.\n"
        "Install it with:  sudo apt install -y python3-opencv"
    )

from picamera2 import MappedArray  # noqa: E402
from picamera2.encoders import MJPEGEncoder  # noqa: E402
from picamera2.outputs import FileOutput  # noqa: E402

from Cameras.RPICamera.RPICamera import RPICamera  # noqa: E402

DEFAULT_CONFIG_PATH = os.path.join(_TESTS_DIR, "mine_detection_config.json")

# BGRA, because the preview stream is XBGR8888 (4 channels).
BOX_COLOR = (0, 255, 0, 0)
TEXT_COLOR = (255, 255, 255, 0)
TEXT_SHADOW = (0, 0, 0, 0)


class DetectionState:
    """Latest detections plus timing, shared between the detect thread and the
    camera callback."""

    def __init__(self):
        self._lock = threading.Lock()
        self._boxes = []
        self._detect_fps = 0.0
        self._frames = 0

    def update(self, boxes, detect_fps):
        with self._lock:
            self._boxes = boxes
            self._detect_fps = detect_fps

    def snapshot(self):
        with self._lock:
            return list(self._boxes), self._detect_fps


class StreamingOutput(io.BufferedIOBase):
    """Holds the most recent JPEG so every HTTP client gets the current frame
    instead of a per-client backlog."""

    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

    def wait_for_frame(self, timeout=5.0):
        with self.condition:
            if not self.condition.wait(timeout):
                return None
            return self.frame


PAGE = """<!DOCTYPE html>
<html>
<head>
<title>IARC-10 mine detection</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body {{ background:#111; color:#eee; font-family:system-ui,sans-serif;
         margin:0; padding:16px; }}
  h1 {{ font-size:16px; font-weight:600; margin:0 0 12px; }}
  .wrap {{ display:flex; gap:16px; flex-wrap:wrap; align-items:flex-start; }}
  img {{ background:#000; max-width:100%; border:1px solid #333; }}
  #dets {{ font-family:ui-monospace,monospace; font-size:13px;
           min-width:260px; line-height:1.6; }}
  .none {{ color:#888; }}
</style>
</head>
<body>
<h1>IARC-10 mine detection &mdash; live feed</h1>
<div class="wrap">
  <img src="stream.mjpg" width="{width}" height="{height}">
  <div id="dets"><span class="none">waiting&hellip;</span></div>
</div>
<script>
async function poll() {{
  try {{
    const r = await fetch('detections.json', {{cache: 'no-store'}});
    const d = await r.json();
    let html = '<div>detect rate: ' + d.detect_fps.toFixed(1) + ' Hz</div>';
    html += '<div>detections: ' + d.detections.length + '</div><br>';
    if (d.detections.length === 0) {{
      html += '<span class="none">nothing detected</span>';
    }} else {{
      d.detections.forEach((x, i) => {{
        html += '[' + i + '] score=' + x.score.toFixed(3) +
                ' cx=' + x.cx.toFixed(0) + ' cy=' + x.cy.toFixed(0) +
                ' w=' + x.w.toFixed(0) + ' h=' + x.h.toFixed(0) + ' px<br>';
      }});
    }}
    document.getElementById('dets').innerHTML = html;
  }} catch (e) {{ /* keep polling */ }}
}}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""


def box_to_pixels(box, frame_w, frame_h):
    """RPICamera already returns (cx, cy, w, h) in main-stream pixels, which is
    the same frame this overlay draws into, so just convert to corners."""
    cx, cy, w, h = (float(v) for v in box)
    x1 = int(round(cx - w / 2))
    y1 = int(round(cy - h / 2))
    x2 = int(round(cx + w / 2))
    y2 = int(round(cy + h / 2))
    x1 = max(0, min(frame_w - 1, x1))
    y1 = max(0, min(frame_h - 1, y1))
    x2 = max(0, min(frame_w - 1, x2))
    y2 = max(0, min(frame_h - 1, y2))
    return x1, y1, x2, y2


def _label(array, text, org, scale=0.45):
    cv2.putText(array, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, TEXT_SHADOW, 3)
    cv2.putText(array, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, TEXT_COLOR, 1)


def make_overlay_callback(state, label_name):
    """picamera2 pre_callback: draws boxes into the frame before it reaches the
    JPEG encoder, so the annotation is baked into the stream."""

    def draw(request):
        boxes, detect_fps = state.snapshot()
        with MappedArray(request, "main") as m:
            array = m.array
            frame_h, frame_w = array.shape[:2]
            for score, box in boxes:
                x1, y1, x2, y2 = box_to_pixels(box, frame_w, frame_h)
                cv2.rectangle(array, (x1, y1), (x2, y2), BOX_COLOR, 2)
                _label(array, f"{label_name} {score:.2f}", (x1, max(12, y1 - 6)))
            _label(
                array,
                f"{len(boxes)} det  {detect_fps:.1f} Hz",
                (8, frame_h - 10),
                scale=0.5,
            )

    return draw


def detection_loop(cam, state, stop_event, interval):
    """Pulls detections through the same RPICamera path the flight code uses."""
    last = time.monotonic()
    fps = 0.0
    while not stop_event.is_set():
        try:
            detections = cam.capture_and_detect_mines()
        except Exception as exc:  # keep the stream alive if a frame fails
            print(f"detection error: {exc}", file=sys.stderr)
            time.sleep(0.5)
            continue

        now = time.monotonic()
        dt = now - last
        last = now
        if dt > 0:
            fps = 0.8 * fps + 0.2 * (1.0 / dt) if fps else 1.0 / dt

        state.update([(float(d.score), tuple(d.box)) for d in detections], fps)
        if interval > 0:
            time.sleep(interval)


def make_handler(output, state, frame_size):
    width, height = frame_size

    class StreamingHandler(server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, fmt, *args):  # quieter console
            pass

        def _send(self, code, content_type, body, extra_headers=()):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            for key, value in extra_headers:
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self.send_response(301)
                self.send_header("Location", "/index.html")
                self.end_headers()
            elif path == "/index.html":
                body = PAGE.format(width=width, height=height).encode("utf-8")
                self._send(200, "text/html; charset=utf-8", body)
            elif path == "/detections.json":
                boxes, detect_fps = state.snapshot()
                payload = {
                    "detect_fps": detect_fps,
                    "detections": [
                        dict(
                            zip(("cx", "cy", "w", "h"), (float(v) for v in box)),
                            score=score,
                        )
                        for score, box in boxes
                    ],
                }
                body = json.dumps(payload).encode("utf-8")
                self._send(
                    200,
                    "application/json",
                    body,
                    extra_headers=(("Cache-Control", "no-store"),),
                )
            elif path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                try:
                    while True:
                        frame = output.wait_for_frame()
                        if frame is None:
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # viewer closed the tab
            else:
                self.send_error(404)
                self.end_headers()

    return StreamingHandler


def local_ips():
    ips = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ips.append(s.getsockname()[0])
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except socket.gaierror:
        pass
    return ips


def load_config(path):
    with open(path) as f:
        config = json.load(f)

    # Resolve relative model/label paths against vision/ so cwd doesn't matter.
    for key in ("modelPath", "labelsPath"):
        value = config.get(key)
        if value and not os.path.isabs(value):
            candidate = os.path.join(_VISION_DIR, value)
            if os.path.exists(candidate):
                config[key] = candidate
    return config


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="vision config JSON")
    p.add_argument("--port", type=int, default=8000, help="HTTP port (default 8000)")
    p.add_argument("--bind", default="0.0.0.0", help="bind address (default 0.0.0.0)")
    p.add_argument("--conf", type=float, help="override confThreshold")
    p.add_argument("--max-detections", type=int, help="override maxDetections")
    p.add_argument(
        "--bbox-order",
        choices=("xy", "yx"),
        help="override bboxOrder; use this to check which one the model wants "
        "(wrong value mirrors boxes across the image diagonal)",
    )
    p.add_argument("--bitrate", type=int, default=4_000_000, help="MJPEG bitrate in bits/s")
    p.add_argument(
        "--detect-interval",
        type=float,
        default=0.0,
        help="extra sleep between detections, seconds (0 = as fast as the camera)",
    )
    return p.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    if args.conf is not None:
        config["confThreshold"] = args.conf
    if args.max_detections is not None:
        config["maxDetections"] = args.max_detections
    if args.bbox_order is not None:
        config["bboxOrder"] = args.bbox_order

    for key in ("modelPath", "labelsPath"):
        if not os.path.exists(config[key]):
            sys.exit(f"{key} not found: {config[key]}")

    print(f"model:  {config['modelPath']}")
    print(f"labels: {config['labelsPath']}")
    print(f"conf threshold: {config['confThreshold']}")
    print(f"bbox order: {config.get('bboxOrder', 'xy')}")
    print("Loading model onto the IMX500 (first run can take ~1 minute)...")

    cam = RPICamera(config)
    cam.initialize_camera()

    input_size = (cam.input_w, cam.input_h)
    label_name = cam.labels[0] if cam.labels else "object"
    frame_size = cam.picam2.camera_configuration()["main"]["size"]
    print(f"stream: {frame_size[0]}x{frame_size[1]}  inference input: {input_size}")

    state = DetectionState()
    cam.picam2.pre_callback = make_overlay_callback(state, label_name)

    output = StreamingOutput()
    encoder = MJPEGEncoder(bitrate=args.bitrate)
    cam.picam2.start_encoder(encoder, FileOutput(output), name="main")

    stop_event = threading.Event()
    detect_thread = threading.Thread(
        target=detection_loop,
        args=(cam, state, stop_event, args.detect_interval),
        daemon=True,
    )
    detect_thread.start()

    handler = make_handler(output, state, frame_size)
    httpd = server.ThreadingHTTPServer((args.bind, args.port), handler)
    httpd.daemon_threads = True

    print("\nStream is live. Open one of these on your computer:")
    for ip in local_ips() or ["<pi-ip>"]:
        print(f"    http://{ip}:{args.port}/")
    print("\nPress Ctrl+C to stop.")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop_event.set()
        httpd.shutdown()
        httpd.server_close()
        cam.picam2.pre_callback = None
        try:
            cam.picam2.stop_encoder()
        except Exception:
            pass
        cam.picam2.stop()
        detect_thread.join(timeout=2.0)
        print("Stopped.")


if __name__ == "__main__":
    main()
