"""
Measure how long a multi-frame mine scan actually takes on the Pi.

Inference runs on the IMX500 sensor, not on the Pi, so a scan does not cost
N model runs -- capture_and_detect_mines() blocks waiting for the next frame
and then reads an output tensor the sensor already produced. The expected cost
of an N-frame scan is therefore N camera frame intervals (~33 ms each at the
default 30 fps inference rate), plus a millisecond or two of numpy parsing that
hides inside that interval.

This script checks that claim against real hardware. Run it on the Pi:

    uv run vision/tests/scan_timing_test.py
    uv run vision/tests/scan_timing_test.py --scans 50 --frames 5

It reports per-frame and per-scan timing, and how many detections survive the
vote versus how many raw detections went in.
"""

import argparse
import json
import os
import statistics
import sys
import time

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Cameras.RPICamera.RPICamera import RPICamera  # noqa: E402
from vision.common.mine_voting import vote_on_frames  # noqa: E402

DEFAULT_CONFIG_PATH = os.path.join(_TESTS_DIR, "mine_detection_config.json")


def load_config(path):
    with open(path) as f:
        config = json.load(f)
    for key in ("modelPath", "labelsPath"):
        value = config.get(key)
        if value and not os.path.isabs(value):
            candidate = os.path.join(_VISION_DIR, value)
            if os.path.exists(candidate):
                config[key] = candidate
    return config


def summarize(name, samples, unit="ms"):
    if not samples:
        print(f"{name}: no samples")
        return
    ordered = sorted(samples)
    print(
        f"{name}: mean {statistics.mean(samples):7.2f} {unit}  "
        f"median {statistics.median(samples):7.2f}  "
        f"min {ordered[0]:7.2f}  max {ordered[-1]:7.2f}"
        + (
            f"  stdev {statistics.stdev(samples):6.2f}"
            if len(samples) > 1
            else ""
        )
    )


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="vision config JSON")
    p.add_argument("--scans", type=int, default=30, help="number of scans to time")
    p.add_argument(
        "--frames",
        type=int,
        help="frames per scan (default: scanFrames from the config)",
    )
    p.add_argument(
        "--warmup",
        type=int,
        default=5,
        help="untimed frames first, so the pipeline is at steady state",
    )
    return p.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    frames_per_scan = args.frames or config["scanFrames"]

    for key in ("modelPath", "labelsPath"):
        if not os.path.exists(config[key]):
            sys.exit(f"{key} not found: {config[key]}")

    print("Loading model onto the IMX500 (first run can take ~1 minute)...")
    load_start = time.perf_counter()
    cam = RPICamera(config)
    cam.initialize_camera()
    load_elapsed = time.perf_counter() - load_start

    frame_rate = cam.picam2.camera_configuration()["controls"].get("FrameRate")
    print(f"\none-time camera + model init: {load_elapsed:.2f} s")
    print(f"configured frame rate: {frame_rate}")
    if frame_rate:
        print(f"  -> expected {1000.0 / float(frame_rate):.1f} ms per frame")
        print(
            f"  -> expected {frames_per_scan * 1000.0 / float(frame_rate):.1f} ms "
            f"per {frames_per_scan}-frame scan"
        )
    print(f"per-frame detection threshold: {cam.detect_threshold}")
    print(f"\nwarming up ({args.warmup} frames)...")
    for _ in range(args.warmup):
        cam.capture_and_detect_mines()

    print(f"timing {args.scans} scans of {frames_per_scan} frames...\n")

    frame_times = []
    scan_times = []
    vote_times = []
    raw_counts = []
    voted_counts = []

    for _ in range(args.scans):
        scan_start = time.perf_counter()

        frames = []
        for _ in range(frames_per_scan):
            frame_start = time.perf_counter()
            detections = cam.capture_and_detect_mines()
            frame_times.append((time.perf_counter() - frame_start) * 1000.0)
            frames.append(detections)

        vote_start = time.perf_counter()
        confirmed = vote_on_frames(
            frames,
            iou_threshold=config["voteIoU"],
            min_hits=config["minFrameHits"],
            min_average_score=config["minAverageConfidence"],
        )
        vote_times.append((time.perf_counter() - vote_start) * 1000.0)
        scan_times.append((time.perf_counter() - scan_start) * 1000.0)

        raw_counts.append(sum(len(f) for f in frames))
        voted_counts.append(len(confirmed))

    print(f"--- timing over {args.scans} scans ---")
    summarize("per frame     ", frame_times)
    summarize("vote (CPU)    ", vote_times)
    summarize(f"per {frames_per_scan}-frame scan", scan_times)

    print(f"\n--- detections ---")
    print(
        f"raw (all frames, >= {cam.detect_threshold} conf): "
        f"{sum(raw_counts)} total, {statistics.mean(raw_counts):.2f} per scan"
    )
    print(
        f"confirmed (>= {config['minFrameHits']}/{frames_per_scan} frames, "
        f">= {config['minAverageConfidence']} avg conf): "
        f"{sum(voted_counts)} total, {statistics.mean(voted_counts):.2f} per scan"
    )

    cam.picam2.stop()


if __name__ == "__main__":
    main()
