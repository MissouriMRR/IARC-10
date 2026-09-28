"""
Continuously runs mine detection on the physical IMX500 camera and prints
detections until interrupted with Ctrl+C.
Run from the vision/ directory so relative model paths resolve correctly.
"""

import json
import os
import sys
import time

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from vision.Cameras.RPICamera.RPICamera import RPICamera  # noqa: E402

CONFIG_PATH = os.path.join(_TESTS_DIR, "mine_detection_config.json")


def main():
    with open(CONFIG_PATH) as f:
        config = json.load(f)

    cam = RPICamera(config)
    cam.initialize_camera()

    print("Starting continuous mine detection. Press Ctrl+C to stop.")
    try:
        while True:
            detections = cam.capture_and_detect_mines()
            print(f"Detected {len(detections)} mine(s):")
            for i, det in enumerate(detections):
                cx, cy, w, h = det.box
                print(
                    f"  [{i}] score={det.score:.3f}  cx={cx:.1f}  cy={cy:.1f}  w={w:.1f}  h={h:.1f}  (px)"
                )
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
