"""
Field data-collection loop: connects to the flight controller, then captures
images with RPICamera as fast as the camera/model allow, running mine
detection on each one. For every capture this saves:

  - the raw, untouched image (raw/)
  - an annotated copy with each detection's box plus its ground lat/lon burned
    in as a text label next to the box, using BaseCamera's pixel-to-coordinate
    wiring (annotated/)
  - a JSON sidecar with the drone's pose at capture time, the two image
    filenames it corresponds to, and the computed coordinate of each
    detection (pose/)

Useful for sanity-checking the corner/pixel geolocation math (see
vision/Cameras/baseCamera.py) against real flight controller telemetry, and
for building up a labeled dataset with ground-truth-ish coordinates attached.

Run on the Pi, connected to the flight controller over /dev/serial0:

    uv run vision/tests/live_mine_geolocation_test.py
    uv run vision/tests/live_mine_geolocation_test.py --captures 50
    uv run vision/tests/live_mine_geolocation_test.py --address /dev/ttyUSB0 --baud 921600

Ctrl+C stops the loop; captures already saved are kept.

Note: capture_and_detect_mines() and capture_image() are two separate camera
captures (RPICamera runs detection off on-chip metadata, decoupled from
fetching a full frame -- see RPICamera.capture_and_detect_mines), so the boxes
drawn here are one frame interval (~33ms at 30fps) newer than the saved image,
not pixel-exact. Fine for a slow-moving or stationary bench/field test; not a
frame-synced capture.
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import dronekit  # noqa: E402
from PIL import ImageDraw  # noqa: E402

from Cameras.RPICamera.RPICamera import RPICamera  # noqa: E402
from vision.common.drone_coordinates import DronePose  # noqa: E402

DEFAULT_CONFIG_PATH = os.path.join(_TESTS_DIR, "mine_detection_config.json")
DEFAULT_OUT_DIR = os.path.join(_TESTS_DIR, "geolocation_captures")
DEFAULT_ADDRESS = "/dev/serial0"
DEFAULT_BAUD = 115200

BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 0)


def load_config(path: str) -> dict:
    with open(path) as f:
        config = json.load(f)
    for key in ("modelPath", "labelsPath"):
        value = config.get(key)
        if value and not os.path.isabs(value):
            candidate = os.path.join(_VISION_DIR, value)
            if os.path.exists(candidate):
                config[key] = candidate
    return config


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="vision config JSON")
    p.add_argument("--address", default=DEFAULT_ADDRESS, help="flight controller connection string")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="flight controller baud rate")
    p.add_argument("--out", default=DEFAULT_OUT_DIR, help="output directory")
    p.add_argument(
        "--captures",
        type=int,
        default=None,
        help="stop after N captures (default: run until Ctrl+C)",
    )
    return p.parse_args()


def drone_pose_from_vehicle(vehicle: dronekit.Vehicle) -> DronePose:
    """DroneKit reports attitude in radians; DronePose/rotation_matrix expect degrees."""
    location = vehicle.location.global_relative_frame
    attitude = vehicle.attitude
    return DronePose(
        lat=location.lat,
        lon=location.lon,
        altitude=location.alt,
        yaw=math.degrees(attitude.yaw),
        pitch=math.degrees(attitude.pitch),
        roll=math.degrees(attitude.roll),
    )


def annotate_detections(image, detections, camera: RPICamera, drone_pose: DronePose) -> list[dict]:
    """Draws each detection's box and geocoded lat/lon onto `image` in place.
    Returns the per-detection records (box + coordinate) for the pose JSON."""
    draw = ImageDraw.Draw(image)
    width, height = image.size
    records = []
    for det in detections:
        cx, cy, w, h = det.box
        box = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        draw.rectangle(box, outline=BOX_COLOR, width=3)

        coord = camera.get_pixel_coordinate(cx, cy, width, height, drone_pose)
        label = f"{coord[0]:.6f}, {coord[1]:.6f}" if coord is not None else "no ground intersection"
        draw.text((box[2] + 4, box[1]), label, fill=TEXT_COLOR)

        records.append(
            {
                "score": det.score,
                "box_cxcywh": [cx, cy, w, h],
                "lat": coord[0] if coord is not None else None,
                "lon": coord[1] if coord is not None else None,
            }
        )
    return records


def main():
    args = parse_args()
    config = load_config(args.config)

    raw_dir = os.path.join(args.out, "raw")
    annotated_dir = os.path.join(args.out, "annotated")
    pose_dir = os.path.join(args.out, "pose")
    for d in (raw_dir, annotated_dir, pose_dir):
        os.makedirs(d, exist_ok=True)

    print(f"connecting to {args.address} @ {args.baud} baud...")
    vehicle = dronekit.connect(args.address, wait_ready=True, baud=args.baud)
    print("connected")

    print("loading camera + model (first run can take ~1 minute)...")
    camera = RPICamera(config)
    camera.initialize_camera()
    print("camera ready\n")

    count = 0
    loop_start = time.perf_counter()
    try:
        while args.captures is None or count < args.captures:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

            drone_pose = drone_pose_from_vehicle(vehicle)
            image = camera.capture_image(only_metadata=False)
            detections = camera.capture_and_detect_mines()

            raw_path = os.path.join(raw_dir, f"image_{timestamp}.jpg")
            image.image.save(raw_path)

            annotated = image.image.copy()
            detection_records = annotate_detections(annotated, detections, camera, drone_pose)
            annotated_path = os.path.join(annotated_dir, f"image_{timestamp}.jpg")
            annotated.save(annotated_path)

            pose_path = os.path.join(pose_dir, f"pose_{timestamp}.json")
            with open(pose_path, "w") as f:
                json.dump(
                    {
                        "image_raw": os.path.relpath(raw_path, args.out),
                        "image_annotated": os.path.relpath(annotated_path, args.out),
                        "lat": drone_pose.lat,
                        "lon": drone_pose.lon,
                        "altitude_m": drone_pose.altitude,
                        "yaw_deg": drone_pose.yaw,
                        "pitch_deg": drone_pose.pitch,
                        "roll_deg": drone_pose.roll,
                        "detections": detection_records,
                    },
                    f,
                    indent=2,
                )

            count += 1
            elapsed = time.perf_counter() - loop_start
            print(
                f"[{count}] {len(detections)} detection(s)  "
                f"({count / elapsed:.2f} captures/s avg)  -> {raw_path}"
            )
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        camera.picam2.stop()
        vehicle.close()
        print(f"done -- {count} capture(s) saved under {args.out}")


if __name__ == "__main__":
    main()
