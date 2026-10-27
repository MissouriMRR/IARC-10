"""GIF renderer for the path-node simulation.

Usage:
    m = Mission(mines, drones=4, record_frames=True).run()
    render_mission_gif(m, "out.gif")
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

from flight.pathfinding.pathnode.geometry import CELL, SAFETY_RADIUS_FT
from flight.pathfinding.pathnode.planner import GROUPS

# How a claimed waypoint is drawn, by group: (marker, filled).
GROUP_MARKER = {"path": ("D", True), "big_shortcut": ("^", True), "adjacent": ("s", False),
                "shortcut": ("^", False), "outer": ("o", False)}

DRONE_COLOR = {1: "#c9972b", 2: "#2e6f95", 3: "#c0392b", 4: "#27ae60"}


def _drone_name(drone, side_of):
    side = "bottom" if side_of[drone] == 1 else "top"
    return f"drone {drone} ({side} side)"


def render_mission_gif(mission, save_path, frame_duration_ms=160, hold_last_frames=10,
                       max_frames=300):
    """GIF of a recorded Mission (sim.py, record_frames=True).

    - Each side's chain in its speaker's colour: checked solid, pending spans dashed, the
      forward gap dotted, jumps starred.
    - Every claimed waypoint in the colour of the drone holding it, marked by group
      (GROUP_MARKER). Shortcuts being tried are dash-dotted."""
    frames = mission.frames
    if not frames:
        raise ValueError("render_mission_gif needs frames -- run with record_frames=True")
    if len(frames) > max_frames:
        idx = sorted({round(i * (len(frames) - 1) / (max_frames - 1)) for i in range(max_frames)})
        frames = [frames[i] for i in idx]

    fw, fh = mission.field_w, mission.field_h
    w, h = int(fw // CELL), int(fh // CELL)
    extent = (0, fw, 0, fh)
    side_of = frames[0]["side_of"]
    legend = [
        Patch(facecolor="#8fd19e", alpha=0.55, label="photographed"),
        Patch(facecolor="darkorange", alpha=0.45, edgecolor="firebrick", label="known mine zone"),
        Line2D([], [], color="gray", ls="--", lw=1, label="undiscovered mine (3ft radius)"),
        Line2D([], [], color="black", lw=2.2, label="checked link (C)"),
        Line2D([], [], color="black", lw=1.6, ls="--", label="pending span (B)"),
        Line2D([], [], color="black", lw=1.4, ls=":", label="forward gap"),
        Line2D([], [], color="black", marker="*", ls="", ms=11, label="jump (new point A)"),
    ] + [Line2D([], [], color="black", marker=GROUP_MARKER[g][0], ls="", ms=6,
                mfc=None if GROUP_MARKER[g][1] else "none", label=f"claimed: {g} group")
         for g in GROUPS] + [
        Line2D([], [], color="purple", lw=1.4, ls="-.", label="shortcut being tried"),
    ] + [Line2D([], [], color=DRONE_COLOR[d], marker="o", ls="", ms=6, label=_drone_name(d, side_of))
         for d in sorted(side_of)]

    pil = []
    for fr in frames:
        seen = np.zeros((h, w), dtype=np.uint8)
        for c, r in fr["seen"].on_cells():
            if r < h and c < w:
                seen[r, c] = 1
        fig, ax = plt.subplots(figsize=(7.2, 15), dpi=80)
        ax.imshow(seen, cmap=ListedColormap(["none", "#8fd19e"]), vmin=0, vmax=1,
                  interpolation="nearest", origin="lower", alpha=0.55, extent=extent, zorder=1)
        for poly in fr["zones"]:
            ax.add_patch(MplPolygon(poly, closed=True, facecolor="darkorange", alpha=0.45,
                                    edgecolor="firebrick", lw=1.0, zorder=2))
        for mx, my in mission.true_mines:
            if (mx, my) not in fr["discovered"]:
                ax.add_patch(Circle((mx, my), SAFETY_RADIUS_FT, facecolor="none",
                                    edgecolor="gray", ls="--", lw=0.8, zorder=2))

        n_pending, open_gaps = 0, 0
        n_claimed = [0] * len(GROUPS)
        for g, side in fr["sides"].items():
            color = DRONE_COLOR[g]
            for kind, poly in side["links"]:
                xs, ys = [p[0] for p in poly], [p[1] for p in poly]
                if kind == "checked":
                    ax.plot(xs, ys, color=color, lw=2.2, zorder=5)
                elif kind == "pending":
                    n_pending += 1
                    ax.plot(xs, ys, color="black", lw=1.6, ls="--", zorder=5)
                else:
                    open_gaps += 1
                    ax.plot(xs, ys, color=color, lw=1.4, ls=":", zorder=5)
            for x, y, who in side["nodes"]:
                if who == "jump":
                    ax.plot(x, y, marker="*", ms=13, color=color, mec="black", mew=0.6, zorder=7)
                elif who != "target":
                    ax.plot(x, y, marker="o", ms=3, color=color, zorder=6)
            for sp in side.get("shortcuts", []):
                ax.plot([p[0] for p in sp], [p[1] for p in sp], color="purple", lw=1.4, ls="-.",
                        zorder=5)
            for x, y, grp, drone in side["claims"]:
                n_claimed[grp] += 1
                marker, filled = GROUP_MARKER[GROUPS[grp]]
                color = DRONE_COLOR[drone]
                ax.plot(x, y, marker=marker, ms=5, color=color, mfc=color if filled else "none",
                        mec=color, mew=1.1, alpha=0.9, zorder=6)

        for drone, pts in fr["photos"].items():
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], color=DRONE_COLOR[drone],
                        lw=0.6, alpha=0.35, zorder=4)
                ax.plot(*pts[-1], marker="o", ms=13, mfc="none", mec=DRONE_COLOR[drone],
                        mew=2.2, zorder=8)

        two_sided = len(fr["sides"]) > 1
        status = ("sides MET" if two_sided else "reached far edge") if not open_gaps else (
            "sides closing" if two_sided else "heading for far edge")
        ax.set_title(
            f"tick {fr['tick']}   mines found {len(fr['discovered'])}/{len(mission.true_mines)}   "
            f"photos {sum(len(p) for p in fr['photos'].values())}\n"
            f"pending spans {n_pending}   {status}\n"
            "claimed: " + "   ".join(f"{g} {n}" for g, n in zip(GROUPS, n_claimed)),
            fontsize=10)
        ax.set_xlim(0, fw)
        ax.set_ylim(0, fh)
        ax.set_aspect("equal")
        ax.set_xlabel("x (ft)")
        ax.set_ylabel("y (ft)")
        ax.legend(handles=legend, loc="upper center", bbox_to_anchor=(0.5, -0.04), ncol=2,
                  fontsize=7.5, frameon=False)
        fig.tight_layout()
        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight")
        plt.close(fig)
        buf.seek(0)
        pil.append(Image.open(buf).convert("P", palette=Image.ADAPTIVE))

    pil.extend([pil[-1]] * hold_last_frames)
    pil[0].save(save_path, save_all=True, append_images=pil[1:], duration=frame_duration_ms,
                loop=0, optimize=True)
    return save_path
