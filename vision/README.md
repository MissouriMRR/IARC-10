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

## Trouble Shooting

If you're still getting issues running the code, try the following:

- Delete the .venv and walk back through steps 4-7
- Reboot the PI
