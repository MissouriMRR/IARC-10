"""
Comprehensive IARC Field Simulation

Edit the CONFIG section below to change mines, path, and flight parameters,
then run:   uv run test_field_grid.py
"""

from field_grid import BlockField

"""
Config
"""
WARNING_RADIUS = 1   # blocks of unsafe buffer around each mine (cityblock)
BUFFER_WIDTH   = 2   # green zone half-width in squares (G in IARC formula)
FLIGHT_TIME    = 5.5 # minutes (score = 0 if > 7)
DRONE_WEIGHT   = 14  # oz total  (N = max(0, weight - 16))

# Mine positions as (col 0-39, row 0-149)
MINES = [
    (5,  15),
    (28, 30),
    (10, 47),
    (35, 58),
    (7,  72),
    (30, 83),
    (5,  96),
    (33, 108),
    (18, 122),
    (38, 137),
]

# Path: starting column (0-39), then (direction, step_count) commands
#   U = north, D = south, L = west, R = east
START_COL = 19
COMMANDS = [
    ("U", 30),   # north to row 30
    ("L", 5),    # west  to col 14
    ("U", 20),   # north to row 50
    ("R", 8),    # east  to col 22
    ("U", 25),   
    ("L", 10),   
    ("U", 20),  
    ("R", 15),   
    ("U", 25),   
]

"""
Simulation
"""


bf = BlockField(warning_radius=WARNING_RADIUS)

# ── mine placement ───────────────────────────────────────────────────────
print("=" * 50)
print("IARC Mission 10 — Field Simulation")
print("=" * 50)
print(f"\nField:   {BlockField.NUM_COLS} cols × {BlockField.NUM_ROWS} rows  "
      f"({BlockField.NUM_COLS * BlockField.BLOCK_SIZE} ft × "
      f"{BlockField.NUM_ROWS * BlockField.BLOCK_SIZE} ft)")
print(f"Warning radius: {WARNING_RADIUS} block(s)\n")

print("── Mines ──────────────────────────────────────")
for col, row in MINES:
    wx, wy = BlockField.block_to_real(col, row)
    bf.add_mine(col, row)
    state = bf.get_state(col, row)
    print(f"  block ({col:>2}, {row:>3})  →  world ({wx:.0f} ft, {wy:.0f} ft)  [{state.name}]")

# ── path ─────────────────────────────────────────────────────────────────
print(f"\n── Path (start col {START_COL}) ────────────────────")
bf.path_from_commands(START_COL, COMMANDS)

# Trace the path segment by segment
col, row = START_COL, 0
print(f"  Start     at block ({col:>2}, {row:>3})")
for direction, count in COMMANDS:
    DIRS = {"U": (0, 1), "D": (0, -1), "L": (-1, 0), "R": (1, 0)}
    dc, dr = DIRS[direction]
    col += dc * count
    row += dr * count
    print(f"  {direction},{count:<4}   →  block ({col:>2}, {row:>3})")

path_len_ft = (len(bf.path_order) - 1) * BlockField.BLOCK_SIZE
print(f"\n  Total path blocks : {len(bf.path_order)}")
print(f"  Total path length : {path_len_ft} ft  ({path_len_ft / 300 * 100:.1f}% of field)")

# ── IARC path string ─────────────────────────────────────────────────────
print(f"\n── IARC Path String (buffer_width={BUFFER_WIDTH}) ────")
iarc_str = bf.to_iarc_path(buffer_width=BUFFER_WIDTH)
for line in iarc_str.split("\n"):
    print(f"  {line}")

# ── score ────────────────────────────────────────────────────────────────
score_info = bf.calculate_score(
    buffer_width=BUFFER_WIDTH,
    flight_time_min=FLIGHT_TIME,
    drone_weight_oz=DRONE_WEIGHT,
)

W, B, L, A, N = (score_info[k] for k in ("W", "B", "L", "A", "N"))

print(f"\n── IARC Score ─────────────────────────────────")
print(f"  Formula:  150000 × W / ((1+B) × L × (1 + 7A + 100N))")
print(f"  W  path width (ft)    = {W}")
print(f"  B  mines in buffer    = {B}")
print(f"  L  path length (ft)   = {L}")
print(f"  A  flight time (min)  = {A}")
print(f"  N  excess weight (oz) = {N}")
print(f"  ──────────────────────────────────────────────")
print(f"  Score = 150000 × {W} / ((1+{B}) × {L} × (1 + 7×{A} + 100×{N}))")
print(f"  Score = {score_info['score']:.4f}")

if A > 7:
    print("  ⚠ Flight time exceeds 7 minutes — score is 0.")

# ── visualization ────────────────────────────────────────────────────────
print("\nOpening field visualization (close window to exit)...")
bf.show_grid(
    cell_size=8,
    title="IARC Mission 10 — Field Simulation",
    score_info=score_info,
    buffer_width=BUFFER_WIDTH,
)
