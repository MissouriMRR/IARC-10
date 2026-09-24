"""GIF renderer for the path-node simulation.

Usage:
    m = PathNodeMission(mines, record_frames=True).run()
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

from flight.pathfinding.pathnode.geometry import CELL, FIELD_H, FIELD_W, SAFETY_RADIUS_FT

ROLE_COLOR = {"g1": "#c9972b", "a1": "#2e6f95", "g2": "#c0392b", "a2": "#27ae60"}
ROLE_NAME = {"g1": "gambler 1", "a1": "assistant 1", "g2": "gambler 2", "a2": "assistant 2"}


def _placer_color(placed_by):
    return ROLE_COLOR.get(placed_by, "black")


def render_mission_gif(mission, save_path, frame_duration_ms=160, hold_last_frames=10,
                       max_frames=300):
    frames = mission.frames
    if not frames:
        raise ValueError("render_mission_gif needs frames -- run with record_frames=True")
    if len(frames) > max_frames:
        idx = sorted({round(i * (len(frames) - 1) / (max_frames - 1)) for i in range(max_frames)})
        frames = [frames[i] for i in idx]

    w, h = int(FIELD_W // CELL), int(FIELD_H // CELL)
    extent = (0, FIELD_W, 0, FIELD_H)
    legend = [
        Patch(facecolor="#8fd19e", alpha=0.55, label="photographed"),
        Patch(facecolor="darkorange", alpha=0.45, edgecolor="firebrick", label="known mine zone"),
        Line2D([], [], color="gray", ls="--", lw=1, label="undiscovered mine (3ft radius)"),
        Line2D([], [], color="black", lw=2.2, label="checked link (C)"),
        Line2D([], [], color="black", lw=1.6, ls="--", label="pending span (B)"),
        Line2D([], [], color="black", lw=1.4, ls=":", label="forward gap"),
        Line2D([], [], color="black", marker="*", ls="", ms=11, label="point A after a jump"),
    ] + [Line2D([], [], color=c, marker="o", ls="", ms=6, label=ROLE_NAME[r])
         for r, c in ROLE_COLOR.items()]

    pil = []
    for fr in frames:
        seen = np.zeros((h, w), dtype=np.uint8)
        for c, r in fr["seen"].on_cells():
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

        nodes = fr["nodes"]
        for i, (kind, owner, poly) in enumerate(fr["links"]):
            xs, ys = [p[0] for p in poly], [p[1] for p in poly]
            if kind == "checked":
                ax.plot(xs, ys, color=_placer_color(nodes[i + 1][2]), lw=2.2, zorder=5)
            elif kind == "pending":
                ax.plot(xs, ys, color=ROLE_COLOR.get(owner, "black"), lw=1.6, ls="--", zorder=5)
            else:
                ax.plot(xs, ys, color="black", lw=1.4, ls=":", zorder=5)
        for x, y, who in nodes:
            if who in ("A1", "A2"):
                ax.plot(x, y, marker="*", ms=13, color=ROLE_COLOR["g" + who[1]],
                        mec="black", mew=0.6, zorder=7)
            else:
                ax.plot(x, y, marker="o", ms=3, color=_placer_color(who), zorder=6)

        for role, pts in fr["photos"].items():
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], color=ROLE_COLOR[role],
                        lw=0.6, alpha=0.35, zorder=4)
                ax.plot(*pts[-1], marker="o", ms=13, mfc="none", mec=ROLE_COLOR[role],
                        mew=2.2, zorder=8)

        n_pending = sum(1 for k, _o, _p in fr["links"] if k == "pending")
        met = not any(k == "forward" for k, _o, _p in fr["links"])
        ax.set_title(
            f"tick {fr['tick']}   mines found {len(fr['discovered'])}/{len(mission.true_mines)}   "
            f"photos {sum(len(p) for p in fr['photos'].values())}\n"
            f"pending spans {n_pending}   {'gamblers MET' if met else 'gamblers closing'}",
            fontsize=10)
        ax.set_xlim(0, FIELD_W)
        ax.set_ylim(0, FIELD_H)
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
