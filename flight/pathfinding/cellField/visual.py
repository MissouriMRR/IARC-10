"""
Visualization for CellField -- kept separate from cellField.py so using the
core bit-manipulation class never requires importing matplotlib/numpy.
"""

import numpy as np
from matplotlib import pyplot as plt

from flight.pathfinding.cellField.cellField import CellField


def render_field(
    field: CellField,
    save_path: str | None = None,
    cell_pixels: int = 16,
    grid_line_width: float = 0.6,
    title: str = "",
) -> None:
    """
    Renders `field` as an image: off cells (0) are black, on cells (1) are
    white, with a thin grid line clearly dividing every square. Saves to
    `save_path` if given, otherwise shows it interactively.
    """
    width, height = field.width, field.height

    arr = np.zeros((height, width), dtype=np.uint8)
    for x, y in field.on_cells():
        arr[y, x] = 1

    fig_w = max(2.0, width * cell_pixels / 100)
    fig_h = max(2.0, height * cell_pixels / 100)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=100)

    # origin="lower" so row 0 (y=0) is at the bottom, matching this
    # codebase's other field visualizations (Field.plotField,
    # BlockField.show_grid) where y increases upward.
    ax.imshow(arr, cmap="gray", vmin=0, vmax=1, interpolation="nearest", origin="lower")

    ax.set_xticks([i - 0.5 for i in range(width + 1)])
    ax.set_yticks([i - 0.5 for i in range(height + 1)])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(length=0)
    ax.grid(which="major", color="gray", linewidth=grid_line_width)
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(-0.5, height - 0.5)

    if title:
        ax.set_title(title)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        plt.close(fig)
    else:
        plt.show()
