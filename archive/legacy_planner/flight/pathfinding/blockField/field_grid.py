import numpy as np
from enum import IntEnum


class BlockState(IntEnum):
    EMPTY = 0
    PATH = 1  # Square where drone will travel through
    UNSAFE = 2  # Within the cityblock radius of a mine
    MINE = 3  # Contains the mine itself


class BlockField:
    NUM_COLS = 40
    NUM_ROWS = 150
    BLOCK_SIZE = 2  # ft

    def __init__(self, warning_radius: int = 1):
        """
        warning_radius: cityblock distance in blocks to mark as UNSAFE around each mine.
        Can be changed based on how big of a buffer we want.
        """
        self.warning_radius = warning_radius
        self.grid = np.zeros((self.NUM_ROWS, self.NUM_COLS), dtype=np.int8)
        self.mines: list[tuple[int, int]] = []  # (col, row) position of each mine
        self.path_order: list[tuple[int, int]] = []  # ordered path blocks for IARC output

    """
    Coordinate functions
    """

    # Converts real world coordinates (feet, relative to field origin) to block indices
    @staticmethod
    def real_to_block(real_x: float, real_y: float) -> tuple[int, int]:
        col = int(real_x / BlockField.BLOCK_SIZE)
        col = max(0, min(col, BlockField.NUM_COLS - 1))

        row = int(real_y / BlockField.BLOCK_SIZE)
        row = max(0, min(row, BlockField.NUM_ROWS - 1))
        return col, row

    # Returns the real world coordinate of the center of a block
    @staticmethod
    def block_to_real(col: int, row: int) -> tuple[float, float]:
        x = (col + 0.5) * BlockField.BLOCK_SIZE
        y = (row + 0.5) * BlockField.BLOCK_SIZE
        return x, y

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.NUM_COLS and 0 <= row < self.NUM_ROWS

    """
    Mine functions
    """

    # Adds mine at the specified block coordinate and updates surrounding block states
    def add_mine(self, col: int, row: int) -> None:
        if not self.in_bounds(col, row):
            raise ValueError(f"Block ({col}, {row}) is outside of the field.")
        self.mines.append((col, row))
        self.set_unsafe_zone(col, row)
        self.grid[row, col] = BlockState.MINE  # mine block always wins over UNSAFE

    # Adds a mine based on real world coordinates (feet)
    def add_mine_real(self, real_x: float, real_y: float) -> None:
        col, row = self.real_to_block(real_x, real_y)
        self.add_mine(col, row)

    # Marks all blocks within warning_radius (cityblock distance) around (col, row) as UNSAFE
    def set_unsafe_zone(self, col: int, row: int) -> None:
        radius = self.warning_radius
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                if abs(dc) + abs(dr) > radius:
                    continue
                c2, r2 = col + dc, row + dr
                if self.in_bounds(c2, r2) and self.grid[r2, c2] < BlockState.UNSAFE:
                    self.grid[r2, c2] = BlockState.UNSAFE

    # Removes mine from position and rebuilds the grid
    def remove_mine(self, col: int, row: int) -> None:
        self.mines = [m for m in self.mines if m != (col, row)]
        self._rebuild()

    # Rebuilds the field from stored mines, preserving paths that don't conflict
    def _rebuild(self) -> None:
        saved_path = list(self.path_order)
        self.grid[:] = BlockState.EMPTY
        self.path_order.clear()
        for mc, mr in self.mines:
            self.set_unsafe_zone(mc, mr)
            self.grid[mr, mc] = BlockState.MINE
        for col, row in saved_path:
            if self.grid[row, col] == BlockState.EMPTY:
                self.grid[row, col] = BlockState.PATH
                self.path_order.append((col, row))

    """
    Path functions
    """

    # Marks PATH blocks from a list of real-world waypoints (e.g. from path_subdivision)
    def path_from_waypoints(self, waypoints: list) -> None:
        for wp in waypoints:
            col, row = self.real_to_block(wp[0], wp[1])
            self.mark_path_block(col, row)

    # Marks PATH blocks from IARC U/D/L/R commands
    def path_from_commands(self, start_col: int, commands: list[tuple[str, int]]) -> None:
        DIRS = {"U": (0, 1), "D": (0, -1), "L": (-1, 0), "R": (1, 0)}
        col, row = start_col, 0
        self.mark_path_block(col, row)
        for direction, count in commands:
            dc, dr = DIRS[direction]
            for _ in range(count):
                col += dc
                row += dr
                if not self.in_bounds(col, row):
                    raise ValueError(f"Path exits field at block ({col}, {row}).")
                self.mark_path_block(col, row)

    # Changes block state to PATH and records it in path_order
    def mark_path_block(self, col: int, row: int) -> None:
        if self.grid[row, col] == BlockState.EMPTY:
            self.grid[row, col] = BlockState.PATH
        # Only append if the list is empty or the last entry is different (dedup consecutive)
        if not self.path_order or self.path_order[-1] != (col, row):
            self.path_order.append((col, row))

    # Clears all path blocks and resets path order
    def reset_path(self) -> None:
        self.grid[self.grid == BlockState.PATH] = BlockState.EMPTY
        self.path_order.clear()

    """
    Queries
    """

    # Returns the state of a block based on block index
    def get_state(self, col: int, row: int) -> BlockState:
        if not self.in_bounds(col, row):
            raise ValueError(f"Block ({col}, {row}) is outside the field.")
        return BlockState(int(self.grid[row, col]))

    # Returns the state of a block based on real world coordinates
    def get_state_real(self, real_x: float, real_y: float) -> BlockState:
        col, row = self.real_to_block(real_x, real_y)
        return self.get_state(col, row)

    # Returns True if block is safe to fly through (EMPTY or PATH)
    def is_safe(self, col: int, row: int) -> bool:
        return self.get_state(col, row) < BlockState.UNSAFE

    # Returns True if block at real world coordinates is safe
    def is_safe_real(self, real_x: float, real_y: float) -> bool:
        col, row = self.real_to_block(real_x, real_y)
        return self.is_safe(col, row)

    # Returns list of (col, row) for every MINE block
    def get_mine_blocks(self) -> list[tuple[int, int]]:
        rows, cols = np.where(self.grid == BlockState.MINE)
        return list(zip(cols.tolist(), rows.tolist()))

    # Returns list of (col, row) for every UNSAFE block (excluding mines themselves)
    def get_unsafe_blocks(self) -> list[tuple[int, int]]:
        rows, cols = np.where(self.grid == BlockState.UNSAFE)
        return list(zip(cols.tolist(), rows.tolist()))

    # Returns list of (col, row) for every block the drone must not enter
    def get_no_go_blocks(self) -> list[tuple[int, int]]:
        rows, cols = np.where(self.grid >= BlockState.UNSAFE)
        return list(zip(cols.tolist(), rows.tolist()))

    # Returns list of (col, row) for every PATH block in travel order
    def get_path_blocks(self) -> list[tuple[int, int]]:
        return list(self.path_order)

    """
    IARC scoring conversion
    """

    def to_iarc_path(self, buffer_width: int) -> str:
        """
        Turns path blocks into the IARC reporting format:
            S,<start_col>,<buffer_width>
            U,9
            L,1
            U,5
            ...

        buffer_width: green zone width in squares on each side of the path.
        Raises ValueError if no path is marked or if a step is not a single
        cardinal move (meaning the path is not a valid cityblock path).
        """
        if not self.path_order:
            raise ValueError(
                "No path marked. Call path_from_waypoints or path_from_commands first."
            )

        STEP_TO_DIR: dict[tuple[int, int], str] = {
            (0, 1): "U",
            (0, -1): "D",
            (-1, 0): "L",
            (1, 0): "R",
        }
        start_col: int = self.path_order[0][0]
        lines: list[str] = [f"S,{start_col},{buffer_width}"]

        commands: list[tuple[str, int]] = []
        for i in range(1, len(self.path_order)):
            col1, row1 = self.path_order[i - 1]
            col2, row2 = self.path_order[i]
            step = (col2 - col1, row2 - row1)
            if step not in STEP_TO_DIR:
                raise ValueError(
                    f"Path step from ({col1},{row1}) to ({col2},{row2}) is not a single cityblock move."
                )
            direction = STEP_TO_DIR[step]
            if commands and commands[-1][0] == direction:
                commands[-1] = (direction, commands[-1][1] + 1)
            else:
                commands.append((direction, 1))

        for direction, count in commands:
            lines.append(f"{direction},{count}")

        return "\n".join(lines)

    """
    Scoring
    """

    # Counts the number of mines that fall into the buffer zone
    def count_buffer_mines(self, buffer_width: int) -> int:
        path_set = set(self.path_order)
        count = 0
        for mc, mr in self.mines:
            if (mc, mr) in path_set:
                continue  # mine is directly on the path (invalid IARC run)
            for pc, pr in path_set:
                if pr == mr and abs(mc - pc) <= buffer_width:
                    count += 1
                    break
        return count

    def calculate_score(
        self,
        buffer_width: int,
        flight_time_min: float,
        drone_weight_oz: float,
    ) -> dict:
        """
        IARC scoring formula:
            Score = 150000 * W / ((1+B) * L * (1 + 7*A + 100*N))

        W = path width in feet  = 2 * (1 + 2*G)
        B = mines inside the green zone
        L = path length in feet
        A = flight time in minutes  (score = 0 if A > 7)
        N = oz exceeding 1 lb (16 oz)
        """
        G = buffer_width
        W = 2 * (1 + 2 * G)
        B = self.count_buffer_mines(buffer_width)
        L = max(1, (len(self.path_order) - 1) * self.BLOCK_SIZE)
        A = flight_time_min
        N = max(0.0, drone_weight_oz - 16.0)

        if A > 7:
            score = 0.0
        else:
            score = 150000 * W / ((1 + B) * L * (1 + 7 * A + 100 * N))

        return {
            "score": round(score, 4),
            "W": W,
            "B": B,
            "L": L,
            "A": A,
            "N": N,
            "buffer_width": G,
        }

    """
    Visualization
    """

    # Thanks claude
    def show_grid(
        self,
        cell_size: int = 8,
        title: str = "IARC Field",
        score_info: dict = None,
        buffer_width: int = 0,
    ) -> None:
        """
        Open a tkinter window showing the field grid.
        North (row 149) is at the top, South (row 0) at the bottom.

        score_info:   optional dict returned by calculate_score(), displayed in the panel.
        buffer_width: if > 0, draws the green zone (G squares on each side of the path).
        """
        import tkinter as tk

        COLORS = {
            BlockState.EMPTY: "#EEEEEE",
            BlockState.PATH: "#1565C0",  # IARC blue path
            BlockState.UNSAFE: "#FF8F00",  # orange warning zone
            BlockState.MINE: "#C62828",  # IARC red square
        }
        LABELS = {
            BlockState.EMPTY: "Empty",
            BlockState.PATH: "Path (blue)",
            BlockState.UNSAFE: "Mine buffer",
            BlockState.MINE: "Mine (red)",
        }
        BUFFER_COLOR = "#43A047"  # IARC green zone

        root = tk.Tk()
        root.title(title)
        root.configure(bg="#F0F0F0")

        canvas_w = self.NUM_COLS * cell_size
        canvas_h = self.NUM_ROWS * cell_size
        view_w = canvas_w  # exact fit — no horizontal expansion needed
        view_h = min(canvas_h, 820)

        # ── left: scrollable canvas (fixed width = field width) ──────────
        left = tk.Frame(root, bg="#F0F0F0")
        left.pack(side=tk.LEFT, fill=tk.Y, expand=False, padx=(8, 0), pady=8)

        canvas = tk.Canvas(
            left,
            width=view_w,
            height=view_h,
            bg=COLORS[BlockState.EMPTY],
            scrollregion=(0, 0, canvas_w, canvas_h),
            highlightthickness=1,
            highlightbackground="#AAAAAA",
        )
        vsb = tk.Scrollbar(left, orient=tk.VERTICAL, command=canvas.yview)
        hsb = tk.Scrollbar(left, orient=tk.HORIZONTAL, command=canvas.xview)
        canvas.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        hsb.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.pack(side=tk.LEFT)

        # Compute green zone: G squares perpendicular to travel direction at each path block
        buffer_set: set[tuple[int, int]] = set()
        if buffer_width > 0 and self.path_order:
            for i, (pc, pr) in enumerate(self.path_order):
                # Direction of travel at this block
                if i < len(self.path_order) - 1:
                    nc, nr = self.path_order[i + 1]
                else:
                    nc, nr = self.path_order[i - 1]
                dc_travel = nc - pc
                dr_travel = nr - pr

                if dc_travel == 0:
                    # Moving U/D → buffer extends left and right
                    for dc in range(-buffer_width, buffer_width + 1):
                        if dc == 0:
                            continue
                        bc = pc + dc
                        if self.in_bounds(bc, pr):
                            buffer_set.add((bc, pr))
                else:
                    # Moving L/R → buffer extends up and down
                    for dr in range(-buffer_width, buffer_width + 1):
                        if dr == 0:
                            continue
                        br = pr + dr
                        if self.in_bounds(pc, br):
                            buffer_set.add((pc, br))

        # Draw buffer zone first (underneath everything else)
        for bc, br in buffer_set:
            x0 = bc * cell_size
            y0 = (self.NUM_ROWS - 1 - br) * cell_size
            canvas.create_rectangle(
                x0, y0, x0 + cell_size, y0 + cell_size, fill=BUFFER_COLOR, outline="", width=0
            )

        # Draw non-empty cells on top of buffer (background already fills EMPTY colour)
        for row in range(self.NUM_ROWS):
            for col in range(self.NUM_COLS):
                state = BlockState(int(self.grid[row, col]))
                if state == BlockState.EMPTY:
                    continue
                x0 = col * cell_size
                y0 = (self.NUM_ROWS - 1 - row) * cell_size
                canvas.create_rectangle(
                    x0,
                    y0,
                    x0 + cell_size,
                    y0 + cell_size,
                    fill=COLORS[state],
                    outline="",
                    width=0,
                )

        # Individual cell grid lines drawn on top of colored cells
        for c in range(0, self.NUM_COLS + 1):
            canvas.create_line(c * cell_size, 0, c * cell_size, canvas_h, fill="#BBBBBB", width=1)
        for r in range(0, self.NUM_ROWS + 1):
            canvas.create_line(0, r * cell_size, canvas_w, r * cell_size, fill="#BBBBBB", width=1)

        # Thicker lines every 10 blocks (20 ft) for orientation
        for c in range(0, self.NUM_COLS + 1, 10):
            canvas.create_line(c * cell_size, 0, c * cell_size, canvas_h, fill="#888888", width=1)
        for r in range(0, self.NUM_ROWS + 1, 10):
            canvas.create_line(0, r * cell_size, canvas_w, r * cell_size, fill="#888888", width=1)

        # N / S labels (plain ASCII — unicode arrows break on some Windows fonts)
        canvas.create_text(
            canvas_w // 2, 6, text="N", fill="#444444", font=("Arial", 8, "bold"), anchor="n"
        )
        canvas.create_text(
            canvas_w // 2,
            canvas_h - 6,
            text="S",
            fill="#444444",
            font=("Arial", 8, "bold"),
            anchor="s",
        )

        # Scroll to South end (path start) after the event loop is running
        root.after(10, lambda: canvas.yview_moveto(1.0))

        # ── right: legend + info panel ───────────────────────────────────
        right = tk.Frame(root, bg="#FAFAFA", padx=14, pady=14, relief=tk.RIDGE, bd=1)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=8, pady=8)

        def section(text):
            tk.Frame(right, height=1, bg="#CCCCCC").pack(fill=tk.X, pady=(10, 6))
            tk.Label(right, text=text, font=("Arial", 11, "bold"), bg="#FAFAFA", anchor="w").pack(
                fill=tk.X
            )

        def row_label(label, value):
            f = tk.Frame(right, bg="#FAFAFA")
            f.pack(anchor=tk.W, pady=1)
            tk.Label(f, text=label, width=22, anchor=tk.W, bg="#FAFAFA", font=("Arial", 10)).pack(
                side=tk.LEFT
            )
            tk.Label(f, text=value, bg="#FAFAFA", font=("Arial", 10, "bold")).pack(side=tk.LEFT)

        tk.Label(right, text="IARC Mission 10", font=("Arial", 14, "bold"), bg="#FAFAFA").pack(
            anchor=tk.W
        )
        tk.Label(
            right,
            text=f"{self.NUM_COLS} cols × {self.NUM_ROWS} rows  |  {self.BLOCK_SIZE} ft/block",
            font=("Arial", 9),
            fg="#777777",
            bg="#FAFAFA",
        ).pack(anchor=tk.W, pady=(2, 0))

        section("Legend")
        for state in [BlockState.EMPTY, BlockState.PATH, BlockState.UNSAFE, BlockState.MINE]:
            f = tk.Frame(right, bg="#FAFAFA")
            f.pack(anchor=tk.W, pady=3)
            tk.Canvas(
                f,
                width=18,
                height=18,
                bg=COLORS[state],
                highlightthickness=1,
                highlightbackground="#AAAAAA",
            ).pack(side=tk.LEFT)
            tk.Label(f, text=f"  {LABELS[state]}", bg="#FAFAFA", font=("Arial", 10)).pack(
                side=tk.LEFT
            )
        if buffer_width > 0:
            f = tk.Frame(right, bg="#FAFAFA")
            f.pack(anchor=tk.W, pady=3)
            tk.Canvas(
                f,
                width=18,
                height=18,
                bg=BUFFER_COLOR,
                highlightthickness=1,
                highlightbackground="#AAAAAA",
            ).pack(side=tk.LEFT)
            tk.Label(
                f, text=f"  Green zone (G={buffer_width})", bg="#FAFAFA", font=("Arial", 10)
            ).pack(side=tk.LEFT)

        section("Field Info")
        path_len_ft = max(0, (len(self.path_order) - 1)) * self.BLOCK_SIZE
        row_label("Mines placed:", str(len(self.mines)))
        row_label("Path blocks:", str(len(self.path_order)))
        row_label("Path length:", f"{path_len_ft} ft")
        row_label("Warning radius:", f"{self.warning_radius} block(s)")

        if score_info:
            section("IARC Score")
            row_label("Score:", f"{score_info['score']:.2f}")
            row_label("W  (path width):", f"{score_info['W']} ft")
            row_label("B  (buffer mines):", str(score_info["B"]))
            row_label("L  (path length):", f"{score_info['L']} ft")
            row_label("A  (flight time):", f"{score_info['A']} min")
            row_label("N  (excess weight):", f"{score_info['N']} oz")
            row_label("G  (buffer width):", f"{score_info['buffer_width']} sq")

        root.mainloop()

    # Fallback ASCII view (kept for headless environments)
    def print_grid(self) -> None:
        symbols = {
            BlockState.EMPTY: ".",
            BlockState.PATH: "P",
            BlockState.UNSAFE: "X",
            BlockState.MINE: "M",
        }
        for row in range(self.NUM_ROWS - 1, -1, -1):
            print("".join(symbols[BlockState(int(v))] for v in self.grid[row]))

    def __repr__(self) -> str:
        return (
            f"BlockField(warning_radius={self.warning_radius}, "
            f"mines={len(self.mines)}, path_blocks={len(self.path_order)})"
        )
