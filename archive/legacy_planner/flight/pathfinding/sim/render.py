"""Full-detail renderer for the two-pair (four-drone) simulation drivers.

This is the "everything on" view: seen coverage, mine safety blocks, obstacle hulls,
undiscovered ground truth, the rasterised cell corridor each pair's route occupies, the
C/B/A segments, all four drones' photo trails and live positions, and a legend.

`droneWorkflowTest.render_two_pairs_workflow_gif` is the lighter version of the same idea
(trails + segments only). This one exists because the detailed view kept being rebuilt ad
hoc and then lost; it lives here so it is reproducible.

Usage:
    result = simulate_two_pairs_maze_round_robin(pf1, pf2, mines, record_frames=True, ...)
    render_full_two_pair_gif(mines, result, r"...\\out.gif")
"""

import io

import matplotlib

matplotlib.use("Agg")
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from matplotlib import pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Patch, Polygon as MplPolygon  # noqa: E402

from flight.pathfinder import HEIGHTOFFIELD, WIDTHOFFIELD, WIDTHOFSQUARE
from flight.pathfinding.cellField.cellField import CellField

# (role label, pair id) -> trail colour. Pair 1 gold/steel-blue, pair 2 brick/green, so the
# two pairs stay distinguishable where their routes meet at the midline.
TRAIL_COLOR = {
    ("A", 1): "#c9972b", ("B", 1): "#2e6f95",
    ("A", 2): "#c0392b", ("B", 2): "#27ae60",
}
# "currently here" ring per drone, deliberately unlike any trail colour.
RING_COLOR = {
    ("A", 1): "magenta", ("B", 1): "darkorchid",
    ("A", 2): "orange", ("B", 2): "cyan",
}
ROLE_NAME = {
    ("A", 1): "gambler 1", ("B", 1): "assistant 1",
    ("A", 2): "gambler 2", ("B", 2): "assistant 2",
}
# Corridor tint per pair -- the cells the reported route actually occupies.
CORRIDOR_COLOR = {1: "#4c78a8", 2: "#b279a2"}


def _cells_of(points, width_cells, height_cells):
    """Rasterised cell list for a route, via the same CellField path used for reporting."""
    if len(points) < 2:
        return []
    grid = CellField(width_cells, height_cells, max_corner=(WIDTHOFFIELD, HEIGHTOFFIELD))
    try:
        return grid.path_cells(points)
    except Exception:
        return []


def _joined(frame, pid):
    """C + B + A for one pair as a flat point list (duplicate seams dropped)."""
    pts = []
    for key in (f"c_path_{pid}", f"b_path_{pid}", f"a_path_{pid}"):
        seg = frame[key]
        if not seg:
            continue
        if pts and abs(pts[-1][0] - seg[0][0]) < 1e-6 and abs(pts[-1][1] - seg[0][1]) < 1e-6:
            seg = seg[1:]
        pts.extend(seg)
    return pts


def render_full_two_pair_gif(
    true_mines,
    result,
    save_path,
    max_frames=300,
    frame_duration_ms=90,
    hold_last_frames=8,
    show_corridor=True,
):
    """
    Takes:
    - true_mines: the ground-truth (x, y) list the driver was given.
    - result: a driver result dict recorded with record_frames=True.
    - save_path: output .gif path.
    - show_corridor: draw the rasterised cell corridor under each route.

    Does: renders one frame per recorded snapshot (evenly subsampled to max_frames),
    holding the final frame so the end state is readable, and writes an animated GIF.

    Raises ValueError if the result carries no frames.
    """
    frames_data = result.get("frames") or []
    if not frames_data:
        raise ValueError(
            "render_full_two_pair_gif needs frames -- run the driver with record_frames=True"
        )
    if len(frames_data) > max_frames:
        idxs = sorted(
            {round(i * (len(frames_data) - 1) / (max_frames - 1)) for i in range(max_frames)}
        )
        frames_data = [frames_data[i] for i in idxs]

    width_cells = WIDTHOFFIELD // WIDTHOFSQUARE
    height_cells = HEIGHTOFFIELD // WIDTHOFSQUARE
    extent = (0, WIDTHOFFIELD, 0, HEIGHTOFFIELD)
    midline_y = HEIGHTOFFIELD / 2.0

    legend_handles = [
        Patch(facecolor="#8fd19e", alpha=0.55, label="photographed (seen)"),
        Patch(facecolor="darkorange", alpha=0.6, label="mine safety blocks"),
        Patch(facecolor=CORRIDOR_COLOR[1], alpha=0.35, label="pair 1 route corridor"),
        Patch(facecolor=CORRIDOR_COLOR[2], alpha=0.35, label="pair 2 route corridor"),
        Line2D([], [], color="black", lw=1.8, label="confirmed (C)"),
        Line2D([], [], color="black", lw=1.4, ls="--", label="bridge (B)"),
        Line2D([], [], color="black", lw=1.4, ls=":", label="approach (A)"),
        Line2D([], [], color="gray", lw=1.0, ls="--", label="undiscovered mine"),
        Line2D([], [], color="firebrick", lw=1.1, label="obstacle hull"),
    ]
    legend_handles += [
        Line2D([], [], color=TRAIL_COLOR[k], marker="o", ls="-", ms=4, label=ROLE_NAME[k])
        for k in (("A", 1), ("B", 1), ("A", 2), ("B", 2))
    ]

    pil_frames = []
    for frame in frames_data:
        seen_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["seen"].on_cells():
            seen_arr[y, x] = 1
        mine_arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
        for x, y in frame["mine_blocks"].on_cells():
            mine_arr[y, x] = 1

        fig, ax = plt.subplots(figsize=(7.2, 15), dpi=80)

        ax.imshow(seen_arr, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
                  interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1)
        ax.imshow(mine_arr, cmap=ListedColormap(["none", "darkorange"]), vmin=0, vmax=1,
                  interpolation="nearest", origin="lower", alpha=0.6, extent=extent, zorder=2)

        if show_corridor:
            for pid in (1, 2):
                cells = _cells_of(_joined(frame, pid), width_cells, height_cells)
                if not cells:
                    continue
                arr = np.zeros((height_cells, width_cells), dtype=np.uint8)
                for x, y in cells:
                    if 0 <= x < width_cells and 0 <= y < height_cells:
                        arr[y, x] = 1
                ax.imshow(arr, cmap=ListedColormap(["none", CORRIDOR_COLOR[pid]]), vmin=0, vmax=1,
                          interpolation="nearest", origin="lower", alpha=0.35, extent=extent,
                          zorder=2.2)

        for verts in frame["polygons"]:
            ax.add_patch(MplPolygon(verts, closed=True, facecolor="none", edgecolor="firebrick",
                                    linewidth=1.1, zorder=2.5))

        discovered = frame["discovered"]
        for mx, my in true_mines:
            if (mx, my) in discovered:
                continue
            ax.add_patch(Circle((mx, my), WIDTHOFSQUARE * 1.5, facecolor="none", edgecolor="gray",
                                linestyle="--", linewidth=1.0, zorder=3))
        for mx, my in discovered:
            ax.plot(mx, my, marker="x", color="firebrick", ms=6, mew=1.6, zorder=3.5)

        ax.axhline(midline_y, color="dimgray", lw=1.0, ls="-.", alpha=0.8, zorder=3.2)

        visited = frame["visited"]
        for i in range(len(visited) - 1):
            x0, y0, _s0, lab0, pid0 = visited[i]
            x1, y1, _s1, lab1, pid1 = visited[i + 1]
            if lab0 != lab1 or pid0 != pid1:
                continue
            ax.plot([x0, x1], [y0, y1], color=TRAIL_COLOR[(lab0, pid0)], lw=1.0, zorder=4,
                    alpha=0.85)
        if visited:
            ax.scatter([v[0] for v in visited], [v[1] for v in visited],
                       c=[TRAIL_COLOR[(v[3], v[4])] for v in visited], s=10, zorder=5,
                       edgecolors="white", linewidths=0.2)

        for pid in (1, 2):
            for key, style, lw in ((f"c_path_{pid}", "-", 1.8), (f"b_path_{pid}", "--", 1.4),
                                   (f"a_path_{pid}", ":", 1.4)):
                seg = frame[key]
                if len(seg) >= 2:
                    ax.plot([p[0] for p in seg], [p[1] for p in seg], color="black", ls=style,
                            lw=lw, zorder=6, alpha=0.9)

        # Last photo each drone took this frame = where it currently is.
        for key in (("A", 1), ("B", 1), ("A", 2), ("B", 2)):
            here = [v for v in visited if (v[3], v[4]) == key]
            if here:
                ax.plot(here[-1][0], here[-1][1], marker="o", ms=13, mfc="none",
                        mec=RING_COLOR[key], mew=2.0, zorder=7)

        seen_n = int(seen_arr.sum())
        total = width_cells * height_cells
        ax.set_title(
            f"round {frame['step']}   mines found {len(discovered)}/{len(true_mines)}   "
            f"photos {len(visited)}   seen {seen_n}/{total} cells ({100*seen_n/total:.1f}%)",
            fontsize=10,
        )
        ax.set_xlim(0, WIDTHOFFIELD)
        ax.set_ylim(0, HEIGHTOFFIELD)
        ax.set_aspect("equal")
        ax.set_xlabel("x (ft)")
        ax.set_ylabel("y (ft)")
        ax.legend(handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, -0.045),
                  ncol=3, fontsize=7, frameon=False)
        fig.tight_layout()

        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight")
        plt.close(fig)
        buf.seek(0)
        pil_frames.append(Image.open(buf).convert("P", palette=Image.ADAPTIVE))

    pil_frames.extend([pil_frames[-1]] * hold_last_frames)
    pil_frames[0].save(save_path, save_all=True, append_images=pil_frames[1:],
                       duration=frame_duration_ms, loop=0, optimize=True)
    return save_path
