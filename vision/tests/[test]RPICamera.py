"""
Integration test for RPICamera mine detection.
Requires the physical IMX500 camera and the model files from mine_detection_config.json.
Run from the vision/ directory so relative model paths resolve correctly.
"""
import json
import os
import sys
import unittest

_TESTS_DIR  = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR   = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Cameras.RPICamera.RPICamera import RPICamera  # noqa: E402

CONFIG_PATH = os.path.join(_TESTS_DIR, "mine_detection_config.json")


class TestRPICameraMineDetection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(CONFIG_PATH) as f:
            config = json.load(f)
        cls.cam = RPICamera(config)
        cls.cam.initialize_camera()

    def test_mine_detection_returns_valid_bounding_boxes(self):
        detections = self.cam.capture_and_detect_mines()

        self.assertIsInstance(detections, list)

        for det in detections:
            cx, cy, w, h = det.box
            self.assertGreaterEqual(w, 0, "box width must be non-negative")
            self.assertGreaterEqual(h, 0, "box height must be non-negative")
            self.assertGreaterEqual(det.score, 0.0)
            self.assertLessEqual(det.score, 1.0)

        print(f"\nDetected {len(detections)} mine(s):")
        for i, det in enumerate(detections):
            cx, cy, w, h = det.box
            print(f"  [{i}] score={det.score:.3f}  cx={cx:.4f}  cy={cy:.4f}  w={w:.4f}  h={h:.4f}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
