"""
Interactive mine scan: press Enter, get a scan, watch the result in a browser.

Streams the live camera over HTTP like mine_stream_test.py, but instead of
drawing continuous per-frame detections it holds the result of the last scan on
screen until you trigger another one. Trigger with Enter in this terminal or the
button on the web page.

    uv run vision/tests/scan_trigger_test.py

then open the printed URL, e.g. http://192.168.1.42:8000/

Boxes are color coded by how the vote judged each candidate:

    green   confirmed  -- passed both the frame count and the average confidence
    amber   seen in enough frames, but the averaged confidence was too low
    red     confident enough on average, but not seen in enough frames

The amber and red boxes are the point of this test: they are the near-misses,
and where they cluster tells you which threshold to move. A rock that keeps
coming up amber at 0.68 average means minAverageConfidence is close to the line.

Runs the same RPICamera capture path and the same vote as the flight code, so
the verdicts here are the verdicts scan() would reach. It does not connect to
the drone, so there is no GPS projection -- boxes stay in image pixels.
"""

import argparse
import io
import json
import os
import socket
import sys
import threading
import time
from datetime import datetime
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

from BIGVISIONCLASS import analyze_frames  # noqa: E402
from Cameras.RPICamera.RPICamera import RPICamera  # noqa: E402

DEFAULT_CONFIG_PATH = os.path.join(_TESTS_DIR, "mine_detection_config.json")

# BGRA, because the preview stream is XBGR8888 (4 channels).
COLOR_CONFIRMED = (0, 255, 0, 0)  # green
COLOR_LOW_CONF = (0, 190, 255, 0)  # amber: enough frames, weak average
COLOR_FEW_HITS = (0, 0, 255, 0)  # red: strong enough, too few frames
TEXT_COLOR = (255, 255, 255, 0)
TEXT_SHADOW = (0, 0, 0, 0)


def result_color(result):
    if result.confirmed:
        return COLOR_CONFIRMED
    if result.enough_hits:
        return COLOR_LOW_CONF
    return COLOR_FEW_HITS


class ScanState:
    """The most recent scan, shared between the trigger threads, the camera
    callback that draws it, and the HTTP handler that reports it."""

    def __init__(self, frames_per_scan):
        self._lock = threading.Lock()
        self._results = []
        self._scan_number = 0
        self._elapsed_ms = 0.0
        self._timestamp = None
        self._scanning = False
        self.frames_per_scan = frames_per_scan

    def begin(self):
        """Claim the right to scan. Returns False if one is already running, so
        a leaned-on Enter key or a double-clicked button cannot start two
        overlapping scans on one camera."""
        with self._lock:
            if self._scanning:
                return False
            self._scanning = True
            return True

    def finish(self, results, elapsed_ms):
        with self._lock:
            self._results = results
            self._scan_number += 1
            self._elapsed_ms = elapsed_ms
            self._timestamp = datetime.now()
            self._scanning = False
            return self._scan_number

    def abort(self):
        with self._lock:
            self._scanning = False

    def snapshot(self):
        with self._lock:
            return (
                list(self._results),
                self._scan_number,
                self._elapsed_ms,
                self._timestamp,
                self._scanning,
            )


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
<title>IARC-10 mine scan</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body {{ background:#111; color:#eee; font-family:system-ui,sans-serif;
         margin:0; padding:16px; }}
  h1 {{ font-size:16px; font-weight:600; margin:0 0 12px; }}
  .wrap {{ display:flex; gap:16px; flex-wrap:wrap; align-items:flex-start; }}
  img {{ background:#000; max-width:100%; border:1px solid #333; }}
  #panel {{ font-family:ui-monospace,monospace; font-size:13px;
            min-width:340px; line-height:1.6; }}
  button {{ background:#2d7; border:0; color:#062; font-weight:700;
            font-size:15px; padding:10px 18px; border-radius:6px;
            cursor:pointer; margin-bottom:12px; }}
  button:disabled {{ background:#444; color:#888; cursor:default; }}
  .none {{ color:#888; }}
  .ok {{ color:#2d7; }}
  .lowconf {{ color:#fb0; }}
  .fewhits {{ color:#f55; }}
  .meta {{ color:#888; margin-bottom:8px; }}
  .row {{ margin-bottom:6px; }}
  .legend {{ margin-top:16px; color:#888; font-size:12px; line-height:1.8; }}
  .sw {{ display:inline-block; width:10px; height:10px; margin-right:6px; }}
</style>
</head>
<body>
<h1>IARC-10 mine scan &mdash; press Enter in the terminal, or click Scan</h1>
<div class="wrap">
  <img src="stream.mjpg" width="{width}" height="{height}">
  <div>
    <button id="scan" onclick="triggerScan()">Scan ({frames} frames)</button>
    <div id="panel"><span class="none">no scan yet</span></div>
    <div class="legend">
      <div><span class="sw" style="background:#2d7"></span>confirmed</div>
      <div><span class="sw" style="background:#fb0"></span>enough frames, average confidence too low</div>
      <div><span class="sw" style="background:#f55"></span>confident, but seen in too few frames</div>
    </div>
  </div>
</div>
<script>
async function triggerScan() {{
  document.getElementById('scan').disabled = true;
  try {{ await fetch('scan', {{method: 'POST'}}); }} catch (e) {{ }}
  poll();
}}
async function poll() {{
  try {{
    const r = await fetch('scan.json', {{cache: 'no-store'}});
    const d = await r.json();
    document.getElementById('scan').disabled = d.scanning;

    let html = '';
    if (d.scan_number === 0) {{
      html = d.scanning ? '<span class="none">scanning&hellip;</span>'
                        : '<span class="none">no scan yet &mdash; press Enter or click Scan</span>';
    }} else {{
      html += '<div class="meta">scan #' + d.scan_number + ' &middot; ' +
              d.frames + ' frames in ' + d.elapsed_ms.toFixed(0) + ' ms' +
              ' &middot; ' + d.timestamp + '</div>';
      html += '<div class="row"><span class="ok">' + d.confirmed_count +
              ' confirmed</span> of ' + d.candidates.length + ' candidates</div><br>';
      if (d.candidates.length === 0) {{
        html += '<span class="none">nothing detected in any frame</span>';
      }} else {{
        d.candidates.forEach((c, i) => {{
          const cls = c.confirmed ? 'ok' : (c.enough_hits ? 'lowconf' : 'fewhits');
          html += '<div class="row"><span class="' + cls + '">[' + i + '] ' +
                  c.reason + '</span><br>' +
                  '&nbsp;&nbsp;' + c.hits + '/' + d.frames + ' frames, avg ' +
                  c.average_score.toFixed(3) + ', best ' + c.best_score.toFixed(3) + '<br>' +
                  '&nbsp;&nbsp;scores: ' + c.scores.map(s => s.toFixed(2)).join(', ') + '<br>' +
                  '&nbsp;&nbsp;cx=' + c.cx.toFixed(0) + ' cy=' + c.cy.toFixed(0) +
                  ' w=' + c.w.toFixed(0) + ' h=' + c.h.toFixed(0) + ' px</div>';
        }});
      }}
    }}
    document.getElementById('panel').innerHTML = html;
  }} catch (e) {{ /* keep polling */ }}
}}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""


def box_to_pixels(box, frame_w, frame_h):
    """Boxes are already (cx, cy, w, h) in main-stream pixels, the same frame
    this overlay draws into, so just convert to corners and clamp."""
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
    """picamera2 pre_callback: draws the last scan's verdict into every frame
    before it reaches the JPEG encoder, so the boxes persist on the live video
    until the next scan replaces them."""

    def draw(request):
        results, scan_number, elapsed_ms, _, scanning = state.snapshot()
        with MappedArray(request, "main") as m:
            array = m.array
            frame_h, frame_w = array.shape[:2]

            for result in results:
                color = result_color(result)
                x1, y1, x2, y2 = box_to_pixels(result.box, frame_w, frame_h)
                cv2.rectangle(array, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    array,
                    f"{label_name} {result.average_score:.2f}"
                    f" {result.hits}/{state.frames_per_scan}",
                    (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    TEXT_SHADOW,
                    3,
                )
                cv2.putText(
                    array,
                    f"{label_name} {result.average_score:.2f}"
                    f" {result.hits}/{state.frames_per_scan}",
                    (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                )

            if scanning:
                status = "scanning..."
            elif scan_number == 0:
                status = "press Enter to scan"
            else:
                confirmed = sum(1 for r in results if r.confirmed)
                status = (
                    f"scan #{scan_number}  {confirmed} confirmed"
                    f"  {len(results)} candidates  {elapsed_ms:.0f} ms"
                )
            _label(array, status, (8, frame_h - 10), scale=0.5)

    return draw


def run_scan(cam, config, state, save_dir):
    """Capture the burst and vote on it, exactly as Vision.scan() does.

    Returns None if a scan was already in flight.
    """
    if not state.begin():
        return None

    try:
        start = time.perf_counter()
        frames = [
            cam.capture_and_detect_mines() for _ in range(state.frames_per_scan)
        ]
        results = analyze_frames(
            frames,
            iou_threshold=config["voteIoU"],
            min_hits=config["minFrameHits"],
            min_average_score=config["minAverageConfidence"],
        )
        elapsed_ms = (time.perf_counter() - start) * 1000.0
    except Exception:
        state.abort()
        raise

    scan_number = state.finish(results, elapsed_ms)

    if save_dir:
        save_scan(save_dir, scan_number, results, config, state.frames_per_scan, elapsed_ms)

    confirmed = sum(1 for r in results if r.confirmed)
    print(
        f"scan #{scan_number}: {confirmed} confirmed / {len(results)} candidates "
        f"in {elapsed_ms:.0f} ms"
    )
    for index, result in enumerate(results):
        print(
            f"    [{index}] {result.reason:<28} "
            f"{result.hits}/{state.frames_per_scan} frames  "
            f"avg {result.average_score:.3f}  "
            f"scores {[round(s, 2) for s in result.scores]}"
        )
    print("\nPress Enter to scan again (Ctrl+C to quit).")
    return scan_number


def save_scan(save_dir, scan_number, results, config, frames_per_scan, elapsed_ms):
    """Write the vote breakdown so false positives can be reviewed later."""
    os.makedirs(save_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(save_dir, f"scan_{scan_number:04d}_{stamp}.json")
    payload = {
        "scan_number": scan_number,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "elapsed_ms": elapsed_ms,
        "frames_per_scan": frames_per_scan,
        "thresholds": {
            "voteThreshold": config.get("voteThreshold", config["confThreshold"]),
            "minFrameHits": config["minFrameHits"],
            "minAverageConfidence": config["minAverageConfidence"],
            "voteIoU": config["voteIoU"],
        },
        "confirmed_count": sum(1 for r in results if r.confirmed),
        "candidates": [r.to_dict() for r in results],
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"    saved {path}")


def enter_key_loop(trigger_scan, stop_event):
    """Blocks on stdin so Enter in the terminal triggers a scan."""
    while not stop_event.is_set():
        line = sys.stdin.readline()
        if line == "":  # stdin closed (e.g. running detached); stop listening
            print("stdin closed; use the web button to scan.")
            return
        if stop_event.is_set():
            return
        try:
            if trigger_scan() is None:
                print("a scan is already running, ignoring.")
        except Exception as exc:
            print(f"scan failed: {exc}", file=sys.stderr)


def make_handler(output, state, frame_size, trigger_scan):
    width, height = frame_size

    class ScanHandler(server.BaseHTTPRequestHandler):
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

        def _send_json(self, payload, code=200):
            body = json.dumps(payload).encode("utf-8")
            self._send(
                code,
                "application/json",
                body,
                extra_headers=(("Cache-Control", "no-store"),),
            )

        def do_POST(self):
            if urlparse(self.path).path != "/scan":
                self.send_error(404)
                self.end_headers()
                return
            # Run the scan on the request thread; the server is threaded, so the
            # MJPEG stream keeps flowing while this blocks for ~170 ms.
            try:
                scan_number = trigger_scan()
            except Exception as exc:
                self._send_json({"error": str(exc)}, code=500)
                return
            if scan_number is None:
                self._send_json({"busy": True}, code=409)
            else:
                self._send_json({"scan_number": scan_number})

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self.send_response(301)
                self.send_header("Location", "/index.html")
                self.end_headers()
            elif path == "/index.html":
                body = PAGE.format(
                    width=width, height=height, frames=state.frames_per_scan
                ).encode("utf-8")
                self._send(200, "text/html; charset=utf-8", body)
            elif path == "/scan.json":
                results, scan_number, elapsed_ms, timestamp, scanning = (
                    state.snapshot()
                )
                self._send_json(
                    {
                        "scan_number": scan_number,
                        "scanning": scanning,
                        "frames": state.frames_per_scan,
                        "elapsed_ms": elapsed_ms,
                        "timestamp": (
                            timestamp.strftime("%H:%M:%S") if timestamp else ""
                        ),
                        "confirmed_count": sum(1 for r in results if r.confirmed),
                        "candidates": [r.to_dict() for r in results],
                    }
                )
            elif path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
                )
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

    return ScanHandler


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
    p.add_argument("--frames", type=int, help="override scanFrames")
    p.add_argument("--vote-conf", type=float, help="override voteThreshold")
    p.add_argument("--min-hits", type=int, help="override minFrameHits")
    p.add_argument("--min-avg", type=float, help="override minAverageConfidence")
    p.add_argument("--iou", type=float, help="override voteIoU")
    p.add_argument(
        "--bbox-order",
        choices=("xy", "yx"),
        help="override bboxOrder; wrong value mirrors boxes across the diagonal",
    )
    p.add_argument(
        "--bitrate", type=int, default=4_000_000, help="MJPEG bitrate in bits/s"
    )
    p.add_argument(
        "--no-save", action="store_true", help="do not write per-scan JSON records"
    )
    return p.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)

    if args.frames is not None:
        config["scanFrames"] = args.frames
    if args.vote_conf is not None:
        config["voteThreshold"] = args.vote_conf
    if args.min_hits is not None:
        config["minFrameHits"] = args.min_hits
    if args.min_avg is not None:
        config["minAverageConfidence"] = args.min_avg
    if args.iou is not None:
        config["voteIoU"] = args.iou
    if args.bbox_order is not None:
        config["bboxOrder"] = args.bbox_order

    for key in ("modelPath", "labelsPath"):
        if not os.path.exists(config[key]):
            sys.exit(f"{key} not found: {config[key]}")

    save_dir = None
    if not args.no_save:
        save_dir = config["pathToDetections"]
        if not os.path.isabs(save_dir):
            save_dir = os.path.join(_VISION_DIR, save_dir)

    print(f"model:  {config['modelPath']}")
    print(f"labels: {config['labelsPath']}")
    print(
        f"vote: {config['scanFrames']} frames, "
        f">={config['minFrameHits']} hits, "
        f">={config['minAverageConfidence']} avg conf, "
        f"IoU {config['voteIoU']}"
    )
    print(f"saving scan records to: {save_dir or '(disabled)'}")
    print("Loading model onto the IMX500 (first run can take ~1 minute)...")

    cam = RPICamera(config)
    cam.initialize_camera()

    label_name = cam.labels[0] if cam.labels else "object"
    frame_size = cam.picam2.camera_configuration()["main"]["size"]
    print(f"per-frame detection threshold: {cam.detect_threshold}")
    print(f"stream: {frame_size[0]}x{frame_size[1]}")

    state = ScanState(config["scanFrames"])
    cam.picam2.pre_callback = make_overlay_callback(state, label_name)

    output = StreamingOutput()
    encoder = MJPEGEncoder(bitrate=args.bitrate)
    cam.picam2.start_encoder(encoder, FileOutput(output), name="main")

    stop_event = threading.Event()

    def trigger_scan():
        return run_scan(cam, config, state, save_dir)

    handler = make_handler(output, state, frame_size, trigger_scan)
    httpd = server.ThreadingHTTPServer((args.bind, args.port), handler)
    httpd.daemon_threads = True

    key_thread = threading.Thread(
        target=enter_key_loop, args=(trigger_scan, stop_event), daemon=True
    )
    key_thread.start()

    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()

    print("\nStream is live. Open one of these on your computer:")
    for ip in local_ips() or ["<pi-ip>"]:
        print(f"    http://{ip}:{args.port}/")
    print("\nPress Enter here to run a scan (Ctrl+C to quit).")

    try:
        while True:
            time.sleep(0.5)
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
        print("Stopped.")


if __name__ == "__main__":
    main()
