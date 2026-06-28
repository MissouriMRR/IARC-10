# Raspberry Pi Setup — IARC-10 Mine Detection

## 1. Install system packages

```bash
sudo apt update
sudo apt install python3-picamera2 python3-libcamera python3-apriltag
```

## 2. Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

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
uv venv --system-site-packages --python python3.X
```

Verify it worked:

```bash
cat .venv/pyvenv.cfg | grep system-site
# should say: include-system-site-packages = true
```

## 7. Install dependencies

```bash
uv pip install -e .
```

> **Note:** do not run `uv sync` — it recreates the venv without `--system-site-packages`.

## 9. Run vision code!
