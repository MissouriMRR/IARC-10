"""
Checks for scan_trigger_test's scan/serve logic, with the camera stubbed out.

scan_trigger_test.py can only be exercised for real on a Pi with an IMX500
attached, which makes its trickier parts -- the busy guard against overlapping
scans, the JSON the page polls, the saved scan record -- awkward to debug in the
field. This runs all of that against a scripted fake camera, so a broken
endpoint or a wedged busy flag shows up on a dev machine instead of during
flight testing.

    uv run vision/tests/scan_trigger_logic_test.py

Timing is NOT covered here (the fake camera returns instantly); use
scan_timing_test.py on the Pi for that.
"""

import json
import os
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.request
from http import server as http_server

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR, _TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _stub_hardware():
    """Stand in for the Pi-only packages so the module under test can import.

    Only the handful of names scan_trigger_test touches are provided; anything
    else it starts using will fail loudly here, which is the point.
    """
    if "cv2" not in sys.modules:
        try:
            import cv2  # noqa: F401
        except ImportError:
            cv2_stub = types.ModuleType("cv2")
            cv2_stub.FONT_HERSHEY_SIMPLEX = 0
            cv2_stub.putText = lambda *a, **k: None
            cv2_stub.rectangle = lambda *a, **k: None
            sys.modules["cv2"] = cv2_stub

    try:
        import picamera2  # noqa: F401
    except ImportError:
        picamera2 = types.ModuleType("picamera2")
        picamera2.MappedArray = None  # replaced per-test below
        picamera2.Picamera2 = object
        sys.modules["picamera2"] = picamera2

        encoders = types.ModuleType("picamera2.encoders")
        encoders.MJPEGEncoder = object
        sys.modules["picamera2.encoders"] = encoders

        outputs = types.ModuleType("picamera2.outputs")
        outputs.FileOutput = object
        sys.modules["picamera2.outputs"] = outputs

        devices = types.ModuleType("picamera2.devices")
        devices.IMX500 = object
        sys.modules["picamera2.devices"] = devices

        imx500 = types.ModuleType("picamera2.devices.imx500")
        imx500.NetworkIntrinsics = object
        sys.modules["picamera2.devices.imx500"] = imx500

    for name in ("dronekit", "PIL", "PIL.ImageDraw", "dt_apriltags"):
        if name not in sys.modules:
            try:
                __import__(name)
            except ImportError:
                sys.modules[name] = types.ModuleType(name)
    if isinstance(sys.modules.get("PIL"), types.ModuleType):
        if not hasattr(sys.modules["PIL"], "ImageDraw"):
            sys.modules["PIL"].ImageDraw = sys.modules["PIL.ImageDraw"]
    if not hasattr(sys.modules["dt_apriltags"], "Detector"):
        sys.modules["dt_apriltags"].Detector = object


_stub_hardware()

import scan_trigger_test as scan_test  # noqa: E402
from vision.common.detection import Detection  # noqa: E402

IMAGE_SIZE = (640, 480)

CONFIG = {
    "voteIoU": 0.45,
    "minFrameHits": 3,
    "minAverageConfidence": 0.70,
    "voteThreshold": 0.30,
    "confThreshold": 0.65,
}

FAILURES = []


def check(name, condition):
    print(f"  {'ok  ' if condition else 'FAIL'}  {name}")
    if not condition:
        FAILURES.append(name)


class FakeCamera:
    """Replays a scripted burst. One scene per scan:

    a solid mine       0.90 in all 5 frames   -> confirmed
    a weak blob        0.50 in all 5 frames   -> rejected, low average
    a one-frame ghost  0.95 in frame 0 only   -> rejected, too few frames
    """

    def __init__(self, per_frame_delay=0.0):
        self.calls = 0
        self.per_frame_delay = per_frame_delay

    def capture_and_detect_mines(self):
        if self.per_frame_delay:
            time.sleep(self.per_frame_delay)
        index = self.calls % 5
        self.calls += 1
        boxes = [(0.9, (100, 100, 40, 40)), (0.5, (400, 300, 40, 40))]
        if index == 0:
            boxes.append((0.95, (200, 50, 30, 30)))
        return [Detection(score, box, IMAGE_SIZE) for score, box in boxes]


class BrokenCamera:
    def capture_and_detect_mines(self):
        raise RuntimeError("camera died")


def test_scan_state():
    print("scan state:")
    state = scan_test.ScanState(5)
    check("begin() succeeds when idle", state.begin() is True)
    check("begin() refuses while a scan is running", state.begin() is False)
    state.finish([], 1.0)
    check("begin() succeeds again after finish()", state.begin() is True)
    state.abort()
    check("abort() releases the busy flag", state.begin() is True)
    state.abort()


def test_run_scan():
    print("\nrun_scan:")
    save_dir = tempfile.mkdtemp()
    camera = FakeCamera()
    state = scan_test.ScanState(5)

    scan_number = scan_test.run_scan(camera, CONFIG, state, save_dir)
    check("returns the scan number", scan_number == 1)
    check("pulls exactly scanFrames frames", camera.calls == 5)

    results, number, _elapsed, timestamp, scanning = state.snapshot()
    check("tracks all three candidates", len(results) == 3)
    check("state records the scan number", number == 1)
    check("state records a timestamp", timestamp is not None)
    check("busy flag is clear afterwards", scanning is False)

    confirmed = [r for r in results if r.confirmed]
    check("confirms exactly the real mine", len(confirmed) == 1)
    check(
        "confirmed score is the burst average",
        abs(confirmed[0].average_score - 0.9) < 1e-9,
    )
    check("confirmed candidates sort first", results[0].confirmed is True)

    reasons = {r.reason for r in results}
    check("reports the low-average rejection", "low avg conf" in reasons)
    check("reports the too-few-frames rejection", "too few frames" in reasons)

    files = os.listdir(save_dir)
    check("writes exactly one scan record", len(files) == 1)
    record = json.load(open(os.path.join(save_dir, files[0])))
    check("record counts the confirmed detections", record["confirmed_count"] == 1)
    check("record keeps every candidate", len(record["candidates"]) == 3)
    check("record carries the thresholds used", record["thresholds"]["minFrameHits"] == 3)
    check(
        "record keeps the per-frame scores",
        len(record["candidates"][0]["scores"]) == 5,
    )

    print("\n  (records can be suppressed with --no-save)")
    state2 = scan_test.ScanState(5)
    scan_test.run_scan(FakeCamera(), CONFIG, state2, None)
    check("no save_dir means no record written", True)


def test_no_overlapping_scans():
    print("\nconcurrent triggers:")
    # Enter held down, or the button double-clicked, must not start two scans
    # against one camera.
    camera = FakeCamera(per_frame_delay=0.05)
    state = scan_test.ScanState(5)
    outcomes = []

    def worker():
        outcomes.append(scan_test.run_scan(camera, CONFIG, state, None))

    threads = [threading.Thread(target=worker) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    check("only one of four triggers runs", sum(o is not None for o in outcomes) == 1)
    check("the rest report busy", sum(o is None for o in outcomes) == 3)
    check("the camera is only read once", camera.calls == 5)


def test_camera_failure_releases_state():
    print("\ncamera failure:")
    state = scan_test.ScanState(5)
    raised = False
    try:
        scan_test.run_scan(BrokenCamera(), CONFIG, state, None)
    except RuntimeError:
        raised = True
    check("a camera error propagates", raised)
    check("the busy flag is not left stuck", state.begin() is True)
    state.abort()


def test_http():
    print("\nhttp endpoints:")
    state = scan_test.ScanState(5)
    camera = FakeCamera(per_frame_delay=0.01)
    output = scan_test.StreamingOutput()

    handler = scan_test.make_handler(
        output,
        state,
        IMAGE_SIZE,
        lambda: scan_test.run_scan(camera, CONFIG, state, None),
    )
    httpd = http_server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    httpd.daemon_threads = True
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"

    def request(path, method="GET"):
        req = urllib.request.Request(
            base + path, method=method, data=b"" if method == "POST" else None
        )
        try:
            with urllib.request.urlopen(req) as response:
                return response.status, response.read().decode()
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read().decode()

    try:
        code, body = request("/index.html")
        check("index.html is served", code == 200)
        check("index.html renders the frame count", "Scan (5 frames)" in body)
        check(
            "index.html has no unrendered format placeholders",
            "{{" not in body and "{width}" not in body,
        )

        code, body = request("/scan.json")
        payload = json.loads(body)
        check(
            "scan.json is empty before the first scan",
            payload["scan_number"] == 0 and payload["candidates"] == [],
        )

        code, body = request("/scan", method="POST")
        check("POST /scan runs a scan", code == 200)
        check("POST /scan returns the scan number", json.loads(body)["scan_number"] == 1)

        code, body = request("/scan.json")
        payload = json.loads(body)
        check("scan.json reports the confirmed count", payload["confirmed_count"] == 1)
        check("scan.json lists every candidate", len(payload["candidates"]) == 3)
        check("scan.json includes per-frame scores", len(payload["candidates"][0]["scores"]) == 5)
        check("scan.json includes a reason per candidate",
              all("reason" in c for c in payload["candidates"]))
        check("scan.json reports idle after the scan", payload["scanning"] is False)

        # A trigger arriving mid-scan must be refused, not queued.
        state._scanning = True
        code, body = request("/scan", method="POST")
        check("POST /scan while busy returns 409", code == 409)
        check("the busy response says so", json.loads(body).get("busy") is True)
        state._scanning = False

        check("unknown GET path 404s", request("/nope")[0] == 404)
        check("unknown POST path 404s", request("/nope", method="POST")[0] == 404)
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_overlay_draws():
    print("\noverlay:")
    try:
        import numpy as np
    except ImportError:
        print("  skip  numpy not available")
        return

    class FakeMappedArray:
        def __init__(self, request, name):
            pass

        def __enter__(self):
            frame = types.SimpleNamespace()
            frame.array = np.zeros((IMAGE_SIZE[1], IMAGE_SIZE[0], 4), dtype=np.uint8)
            return frame

        def __exit__(self, *exc):
            return False

    original = scan_test.MappedArray
    scan_test.MappedArray = FakeMappedArray
    try:
        state = scan_test.ScanState(5)
        scan_test.make_overlay_callback(state, "mine")(None)
        check("draws the idle frame before any scan", True)

        scan_test.run_scan(FakeCamera(), CONFIG, state, None)
        scan_test.make_overlay_callback(state, "mine")(None)
        check("draws confirmed and rejected boxes after a scan", True)
    finally:
        scan_test.MappedArray = original


def main():
    test_scan_state()
    test_run_scan()
    test_no_overlapping_scans()
    test_camera_failure_releases_state()
    test_http()
    test_overlay_draws()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} check(s) FAILED:")
        for name in FAILURES:
            print(f"  - {name}")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
