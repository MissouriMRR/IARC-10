# Raspberry Pi Setup — IARC-10 Mine Detection

## 1. Install system packages

```bash
sudo apt update
sudo apt install python3-picamera2 python3-libcamera python3-apriltag cmake python3-dev
```

## 2. Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

## 2.5. Setup the Camera Firmware

Setup the camera firmware.
'''bash
sudo apt update && sudo apt full-upgrade
sudo apt install imx500-all
sudo reboot
'''

## 3. Clone the repo

```bash
git clone https://github.com/MissouriMRR/IARC-10.git
cd IARC-10
```

## 4. Check which Python has picamera2

```bash
python3 --version
python3 -c "import picamera2; print('ok')"
```

If that fails, try:

```bash
python3.13 -c "import picamera2; print('ok')"
python3.12 -c "import picamera2; print('ok')"
```

Use whichever version succeeds — call it `python3.X` below.

## 5. Match the project Python version to the system

Open `pyproject.toml` and set `requires-python` to the version from step 4:

```toml
requires-python = "==3.X.Y"
```

Update `.python-version` to match:

```bash
echo "3.X.Y" > .python-version
```

## 6. Create the venv

```bash
uv lock
uv venv --system-site-packages --python python3.X
source .venv/bin/activate
```

Verify it worked:

```bash
cat .venv/pyvenv.cfg | grep system-site
# should say: include-system-site-packages = true
```

## 7. Install dependencies

```bash
uv pip install -e .
uv pip install apriltags
```

> **Note:** do not run `uv sync` — it recreates the venv without `--system-site-packages`.

## 8. Setup Camera Model

Go to vision\models and run the following:

```bash
imx500-package -i packerOut.zip -o .
```

## 9. Run vision code (from the vision directory)!

```bash
uv run tests/\[test\]RPICamera.py
```

## 10. Watch the live feed with detection boxes (headless)

`tests/mine_stream_test.py` runs the same `RPICamera` detection path and serves
the annotated camera feed over HTTP, so you can watch it from a browser on
another computer. Pi OS Lite needs OpenCV for the overlay drawing:

```bash
sudo apt install -y python3-opencv
```

Then, from the repo root with the venv active:

```bash
uv run vision/tests/mine_stream_test.py
```

It prints the URLs to open, e.g. `http://192.168.1.42:8000/`. Useful flags:

- `--port 8080` — change the HTTP port
- `--conf 0.4` — lower the confidence threshold to see more boxes
- `--config <path>` — use a different vision config JSON
- `--bitrate 2000000` — lower stream bitrate on a weak WiFi link
- `--bbox-order xy|yx` — override the `bboxOrder` config key (see below)

### Box coordinates

`capture_and_detect_mines()` returns `Detection.box` as `(cx, cy, w, h)` in
**pixels of the main camera stream**, with `Detection.imageSize` set to that
stream's size. The mapping from the network's square input tensor to the
preview frame is done by `IMX500.convert_inference_coords()`, which accounts for
the ISP scaler crop — normalized inference coords must never be scaled straight
onto the frame, or boxes come out stretched.

`bboxOrder` in the config says how the network orders the four numbers in each
raw box:

- `"xy"` — `[x1, y1, x2, y2]`. Ultralytics YOLO exported with `format="imx"`,
  which is what `models/best_imx_model` is. This is the default.
- `"yx"` — `[y1, x1, y2, x2]`. TensorFlow SSD models; picamera2's own default.

If this is wrong, detection still works but every box is **mirrored across the
image diagonal**: an object in the top-right gets a box in the bottom-left, and
box width and height are swapped. Objects near the diagonal look fine, which
makes it easy to miss. Flip `--bbox-order` if you see that.

## Multi-frame mine voting

`Vision.scan()` does not trust a single frame. It captures a burst of frames and
keeps a detection only if it appears in enough of them *and* its mean confidence
across those frames clears a bar. Association between frames is by bounding-box
IoU in image space.

The vote itself lives in `common/mine_voting.py`, deliberately kept free of
`dronekit`, `picamera2`, and `PIL` so it can be imported and tested anywhere.
`BIGVISIONCLASS.py` re-exports it, so existing imports from there still work.

Config keys (in `config.json` / `tests/mine_detection_config.json`):

| key | default | meaning |
| --- | --- | --- |
| `scanFrames` | 5 | frames captured per scan |
| `minFrameHits` | 3 | frames a candidate must appear in to be confirmed |
| `minAverageConfidence` | 0.70 | mean score across those frames, inclusive |
| `voteIoU` | 0.45 | box overlap needed to call two frames' detections the same object |
| `voteThreshold` | 0.30 | per-frame cutoff for what enters the vote |

`voteThreshold` is deliberately looser than `confThreshold`. A detection
discarded at the camera can never pull a voted average *down*, so filtering the
burst at 0.65 would floor every average at 0.65 and make
`minAverageConfidence` a no-op. If `voteThreshold` is absent the camera falls
back to `confThreshold` and behaves exactly as it did before.

The confirmed `Detection.score` is the **burst average**, not any single frame's
score, and its box is taken from the highest-scoring frame.

### Cost

Inference runs on the IMX500 sensor, not the Pi, and is pipelined into the frame
stream — `capture_and_detect_mines()` waits for the next frame and reads an
output tensor the sensor already computed. An N-frame scan therefore costs N
camera frame intervals (~33 ms each at the default 30 fps inference rate), not N
model runs. A 5-frame scan is ~170 ms. Uploading the `.rpk` to the sensor is a
one-time startup cost, not a per-scan one.

Measure it on real hardware with:

```bash
uv run vision/tests/scan_timing_test.py
```

### Caveat: the frames are captured back to back

At ~33 ms apart the five frames are highly correlated. A false positive that is
*stable* — a rock, a shadow, a patch of dirt — will appear in all five and pass
the hit count exactly as a real mine does; only `minAverageConfidence` rejects
it. Voting suppresses flicker, not persistent misclassification. If persistent
false positives turn out to be the problem, spacing the frames out (so drone
motion changes the viewpoint between them) is the lever, and it costs flight
time: 200 ms spacing makes a scan ~1 s, 500 ms makes it ~2.5 s.

Fast drift is the opposite failure: if the drone moves enough that a mine's box
shifts by more than `voteIoU` allows between frames, each frame starts its own
track and nothing is ever confirmed. `vision/tests/mine_voting_test.py` covers
both edges.

## Interactive scan test (press Enter, see the result)

`tests/scan_trigger_test.py` runs one scan on demand and shows the verdict on a
live web feed. Trigger with **Enter in the terminal** or the **Scan button** on
the page; the result stays on screen until the next scan.

```bash
uv run vision/tests/scan_trigger_test.py
```

Open the printed URL, e.g. `http://192.168.1.42:8000/`.

Boxes are colored by how the vote judged each candidate:

| color | meaning |
| --- | --- |
| green | confirmed — passed both the frame count and the average confidence |
| amber | seen in enough frames, but the averaged confidence was too low |
| red | confident enough on average, but seen in too few frames |

The amber and red boxes are the point of the test. They are the near-misses, and
where they cluster tells you which threshold to move: a rock that keeps coming
up amber at 0.68 average means `minAverageConfidence` is sitting right on the
line. The side panel lists each candidate's per-frame scores, hit count, and
average, so you can see the whole vote rather than just its outcome.

Each scan writes a JSON record to `pathToDetections` with the full vote
breakdown and the thresholds in force, so false positives can be reviewed later.
Pass `--no-save` to skip that.

Threshold overrides, for tuning without editing the config:

- `--frames N` — frames per scan
- `--min-hits N` — frames required to confirm
- `--min-avg 0.75` — required average confidence
- `--vote-conf 0.2` — per-frame cutoff for entering the vote
- `--iou 0.35` — overlap needed to associate detections across frames
- `--port 8080`, `--bitrate 2000000`, `--bbox-order xy|yx` — as in `mine_stream_test.py`

This test does not connect to the drone, so there is no GPS projection — boxes
stay in image pixels.

## Testing off the Pi

Both of these run anywhere, no camera or model needed:

```bash
uv run vision/tests/mine_voting_test.py
```

covers the vote itself — hit counts, averaging, IoU association, and the drift
edges described above.

```bash
uv run vision/tests/scan_trigger_logic_test.py
```

covers `scan_trigger_test.py` against a scripted fake camera: the guard that
stops a held-down Enter key from starting overlapping scans, the JSON the page
polls, the saved scan record, and the overlay draw path. Timing is not covered
there — use `scan_timing_test.py` on the Pi for that.

## Trouble Shooting

If you're still getting issues running the code, try the following:

- Delete the .venv and walk back through steps 4-7
- Reboot the PI
