"""The flight harness's fake camera (integration_tests.TruthCamera) must report one mine as one
spot, whichever drone's camera sees it and from wherever.

It once boxed every detection at the image centre, so each drone put the same mine at a
different place (the middle of its own photo): 7 true mines became 13, the drones' mine sets
never agreed, and the mission waited out its timeouts.
"""

import json

from integration_tests.one_drone_sim_test import TruthCamera
from vision.common.drone_coordinates import DronePose

ORIGIN = (36.0, -95.9)
MINE = (36.00003, -95.89998)


def _camera(pose, mines=(MINE,)):
    with open("vision/config.json") as f:
        return TruthCamera(json.load(f), lambda: pose, list(mines))


def _pose(dlat, dlon):
    return DronePose(lat=ORIGIN[0] + dlat, lon=ORIGIN[1] + dlon, altitude=2.0,
                     yaw=0.0, pitch=0.0, roll=0.0)


def _located(camera):
    (detection,) = camera.capture_and_detect_mines()
    px, py, _w, _h = detection.box
    width, height = detection.imageSize
    return camera.get_pixel_coordinate(px, py, width, height, camera._pose_provider())


def test_two_drones_report_the_same_spot_for_one_mine():
    poses = ((0.00003, 0.00002), (0.000032, 0.000017), (0.000027, 0.000023))   # around the mine
    spots = {_located(_camera(_pose(*p))) for p in poses}
    assert spots == {MINE}


def test_a_mine_is_detected_once_per_camera_and_only_inside_the_footprint():
    camera = _camera(_pose(0.00003, 0.00002))
    assert len(camera.capture_and_detect_mines()) == 1
    assert camera.capture_and_detect_mines() == [], "already reported by this camera"
    assert _camera(_pose(0.001, 0.001)).capture_and_detect_mines() == [], "out of view"


def test_the_detection_box_is_where_the_mine_lies_in_the_frame():
    near_centre = _camera(_pose(0.00003, 0.00002)).capture_and_detect_mines()[0]
    off_centre = _camera(_pose(0.000034, 0.000026)).capture_and_detect_mines()[0]
    assert near_centre.box[:2] != off_centre.box[:2]
