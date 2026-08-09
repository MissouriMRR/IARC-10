"""
Checks for the multi-frame mine vote in BIGVISIONCLASS.vote_on_frames().

The vote is pure logic over Detection objects, so this runs anywhere -- no Pi,
no camera, no model. Run it after touching the voting or IoU code:

    uv run vision/tests/mine_voting_test.py
"""

import os
import sys
import types

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.dirname(_TESTS_DIR)
_IARC_DIR = os.path.dirname(_VISION_DIR)
for _p in (_IARC_DIR, _VISION_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# BIGVISIONCLASS pulls in flight-only packages at import time. The vote itself
# touches none of them, so stub whatever is missing to keep this runnable on a
# dev machine that has no dronekit.
for _name in ("dronekit", "PIL", "PIL.ImageDraw"):
    if _name not in sys.modules:
        try:
            __import__(_name)
        except ImportError:
            sys.modules[_name] = types.ModuleType(_name)
if isinstance(sys.modules.get("PIL"), types.ModuleType):
    sys.modules["PIL"].ImageDraw = sys.modules["PIL.ImageDraw"]

from BIGVISIONCLASS import _iou, vote_on_frames  # noqa: E402
from vision.common.detection import Detection  # noqa: E402

IMAGE_SIZE = (640, 480)

# Same gates as the shipped config.
VOTE = dict(iou_threshold=0.45, min_hits=3, min_average_score=0.70)


def det(score, cx, cy, w=40, h=40):
    return Detection(score, (cx, cy, w, h), IMAGE_SIZE)


def check(name, condition):
    if not condition:
        raise AssertionError(name)
    print(f"  ok  {name}")


def main():
    print("iou:")
    check("identical boxes -> 1.0", _iou((100, 100, 40, 40), (100, 100, 40, 40)) == 1.0)
    check("disjoint boxes -> 0.0", _iou((100, 100, 40, 40), (500, 500, 40, 40)) == 0.0)
    check(
        "half-shifted boxes -> 1/3",
        abs(_iou((100, 100, 40, 40), (120, 100, 40, 40)) - 1 / 3) < 1e-9,
    )

    print("\nvote:")

    # A real mine: present every frame, confidently.
    confirmed = vote_on_frames([[det(0.9, 100, 100)] for _ in range(5)], **VOTE)
    check(
        "strong detection in 5/5 frames is confirmed",
        len(confirmed) == 1 and abs(confirmed[0].score - 0.9) < 1e-9,
    )

    # A confident flicker still fails: two frames is under the hit floor.
    confirmed = vote_on_frames(
        [[det(0.95, 100, 100)], [det(0.95, 100, 100)], [], [], []], **VOTE
    )
    check("confident detection in only 2/5 frames is rejected", confirmed == [])

    # Present every frame but never convincing -- this is the case the averaged
    # confidence gate exists to catch.
    confirmed = vote_on_frames([[det(0.55, 100, 100)] for _ in range(5)], **VOTE)
    check("persistent 0.55 detection is rejected on average confidence", confirmed == [])

    # Exactly on the average threshold, which is inclusive.
    frames = [[det(s, 100, 100)] for s in (0.6, 0.7, 0.8, 0.7)] + [[]]
    confirmed = vote_on_frames(frames, **VOTE)
    check(
        "average of exactly 0.70 over 4/5 frames is confirmed",
        len(confirmed) == 1 and abs(confirmed[0].score - 0.7) < 1e-9,
    )
    check(
        "confirmed detection reports the best frame's box",
        confirmed[0].box == (100, 100, 40, 40),
    )

    # Two mines in frame, plus the small drift a hovering drone produces.
    frames = [[det(0.85, 100 + i * 3, 100), det(0.75, 400, 300)] for i in range(5)]
    confirmed = vote_on_frames(frames, **VOTE)
    check("two separate mines stay separate under small drift", len(confirmed) == 2)

    # Drift faster than the IoU gate tolerates: every frame starts a new track,
    # so nothing accumulates enough hits. This is the failure mode to watch for
    # if the drone is moving during a scan.
    confirmed = vote_on_frames([[det(0.9, 100 + i * 60, 100)] for i in range(5)], **VOTE)
    check("drift beyond the IoU gate prevents association", confirmed == [])

    # Two overlapping detections in one frame must feed two tracks, not both
    # collapse onto the nearest one.
    frames = [[det(0.9, 100, 100), det(0.9, 108, 100)] for _ in range(3)]
    confirmed = vote_on_frames(frames, **VOTE)
    check("one detection cannot claim two tracks in a frame", len(confirmed) == 2)

    check("no frames -> no detections", vote_on_frames([], **VOTE) == [])
    check("all-empty frames -> no detections", vote_on_frames([[]] * 5, **VOTE) == [])

    print("\nall checks passed")


if __name__ == "__main__":
    main()
