"""
Multi-frame voting used to reject false-positive mine detections.

A scan captures several frames, associates detections across them by bounding
box IoU, and keeps a candidate only if it shows up in enough frames AND its mean
confidence across those frames clears a bar. Vision.scan() drives this; the
interactive and timing tests use it directly.

This module deliberately depends on nothing beyond Detection -- no dronekit, no
picamera2, no PIL -- so the vote can be exercised and debugged on any machine.
"""

from vision.common.detection import Detection


def iou(box_a, box_b) -> float:
    """Intersection over union of two (cx, cy, w, h) boxes in image pixels."""
    ax, ay, aw, ah = box_a
    bx, by, bw, bh = box_b

    a_x1, a_y1, a_x2, a_y2 = ax - aw / 2, ay - ah / 2, ax + aw / 2, ay + ah / 2
    b_x1, b_y1, b_x2, b_y2 = bx - bw / 2, by - bh / 2, bx + bw / 2, by + bh / 2

    inter_w = min(a_x2, b_x2) - max(a_x1, b_x1)
    inter_h = min(a_y2, b_y2) - max(a_y1, b_y1)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0

    intersection = inter_w * inter_h
    union = aw * ah + bw * bh - intersection
    if union <= 0:
        return 0.0
    return intersection / union


class _Track:
    """One candidate mine, accumulated across the frames of a single scan."""

    def __init__(self, detection: Detection):
        self.detections: list[Detection] = [detection]

    @property
    def hits(self) -> int:
        return len(self.detections)

    @property
    def average_score(self) -> float:
        return sum(d.score for d in self.detections) / len(self.detections)

    @property
    def best(self) -> Detection:
        """Highest-scoring view of this candidate; its box is the one we report,
        since it is the frame the model was most sure about."""
        return max(self.detections, key=lambda d: d.score)

    @property
    def latest_box(self):
        return self.detections[-1].box


class VoteResult:
    """One candidate's fate in a scan, including why it was rejected.

    vote_on_frames() throws the rejects away; tooling that needs to explain a
    scan (the interactive scan test, saved scan records) wants them, so
    analyze_frames() returns these instead.
    """

    def __init__(self, track: _Track, min_hits: int, min_average_score: float):
        self.box = track.best.box  # best frame's box, in main-stream pixels
        self.imageSize = track.best.imageSize
        self.hits = track.hits
        self.scores = [d.score for d in track.detections]
        self.average_score = track.average_score
        self.best_score = track.best.score

        self.enough_hits = self.hits >= min_hits
        self.enough_confidence = self.average_score >= min_average_score
        self.confirmed = self.enough_hits and self.enough_confidence

    @property
    def reason(self) -> str:
        """Short human-readable verdict, for overlay labels and saved records."""
        if self.confirmed:
            return "confirmed"
        failures = []
        if not self.enough_hits:
            failures.append("too few frames")
        if not self.enough_confidence:
            failures.append("low avg conf")
        return " + ".join(failures)

    def as_detection(self) -> Detection:
        """The averaged score is the number the vote actually justifies, so that
        is what travels onward rather than any single frame's score."""
        return Detection(self.average_score, self.box, self.imageSize)

    def to_dict(self) -> dict:
        cx, cy, w, h = (float(v) for v in self.box)
        return {
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h,
            "hits": self.hits,
            "scores": [float(s) for s in self.scores],
            "average_score": float(self.average_score),
            "best_score": float(self.best_score),
            "confirmed": self.confirmed,
            "enough_hits": self.enough_hits,
            "enough_confidence": self.enough_confidence,
            "reason": self.reason,
        }


def analyze_frames(
    frames: list[list[Detection]],
    iou_threshold: float,
    min_hits: int,
    min_average_score: float,
) -> list[VoteResult]:
    """Associate detections across frames and judge each candidate.

    Returns every candidate found, confirmed or not, sorted best first. Use
    vote_on_frames() if you only want the ones that passed.
    """
    tracks: list[_Track] = []

    for frame in frames:
        # Strongest detections claim their track first, so a confident mine is
        # not stolen by an overlapping weak box earlier in the list.
        claimed: set[int] = set()
        for detection in sorted(frame, key=lambda d: d.score, reverse=True):
            best_index = -1
            best_iou = iou_threshold
            for index, track in enumerate(tracks):
                if index in claimed:
                    continue
                overlap = iou(detection.box, track.latest_box)
                if overlap >= best_iou:
                    best_iou = overlap
                    best_index = index

            if best_index == -1:
                tracks.append(_Track(detection))
                claimed.add(len(tracks) - 1)
            else:
                tracks[best_index].detections.append(detection)
                claimed.add(best_index)

    results = [VoteResult(track, min_hits, min_average_score) for track in tracks]
    results.sort(key=lambda r: (r.confirmed, r.average_score), reverse=True)
    return results


def vote_on_frames(
    frames: list[list[Detection]],
    iou_threshold: float,
    min_hits: int,
    min_average_score: float,
) -> list[Detection]:
    """Collapse several frames of detections into the ones worth believing.

    Detections are associated across frames by box IoU, then a candidate is kept
    only if it appears in at least `min_hits` frames AND its mean score across
    those frames is at least `min_average_score`. This is the false-positive
    filter: a one-frame flicker fails the hit count, and a persistent but weak
    detection fails the average.

    Note that the frames of a scan are captured back to back, so a false
    positive that is stable across the burst (a rock, a shadow) will satisfy the
    hit count as readily as a real mine. The averaged score is what has to
    reject those.
    """
    return [
        result.as_detection()
        for result in analyze_frames(
            frames, iou_threshold, min_hits, min_average_score
        )
        if result.confirmed
    ]
