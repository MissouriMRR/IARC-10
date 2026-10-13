"""Regenerates every figure used by docs/priority_groups.md.

Run from the repo root:   PYTHONPATH=. python docs/img/make_figures.py
(Windows PowerShell: $env:PYTHONPATH="."; python docs/img/make_figures.py)

Every example is built with the real planner classes (flight.pathfinding.pathnode) on a
small 24 x 40 ft window, so nothing here is hand-drawn geometry: routes, no-go zones, rings,
photo positions and claims all come from the code.
"""

import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                                     # noqa: E402
from matplotlib.lines import Line2D                                 # noqa: E402
from matplotlib.patches import Patch, Polygon, Rectangle            # noqa: E402

from flight.pathfinding.pathnode.claims import ClaimBook, path_length   # noqa: E402
from flight.pathfinding.pathnode.config import Group, LinkKind, Source   # noqa: E402
from flight.pathfinding.pathnode.geometry import CELL                # noqa: E402
from flight.pathfinding.pathnode.planner import PathNodePlanner      # noqa: E402

OUT = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.size": 13, "axes.titlesize": 15, "legend.fontsize": 12,
                     "axes.labelsize": 13, "font.family": "DejaVu Sans"})

# Colour-blind-friendly palette (Okabe-Ito based); no red/green pairs.
BLUE, ORANGE, PURPLE = "#0072B2", "#E69F00", "#8E3B9A"
SKY, YELLOW, INK = "#56B4E9", "#F0E442", "#222222"
SEEN = "#DCE6F0"        # photographed square
ZONE = "#CFCFCF"        # no-go zone
W, H = 24.0, 40.0       # the example window, feet


# -- building the example with the real planner -----------------------------------------
def footprint(p, x, y):
    along, across = p.config.shape_size_ft
    return [(x - across / 2, y - along / 2), (x + across / 2, y - along / 2),
            (x + across / 2, y + along / 2), (x - across / 2, y + along / 2)]


def photograph(p, x, y):
    p.record_footprint(footprint(p, x, y))
    p.advance()


def fly_forward(p, upto_y):
    """Take the planner's own forward-gap photos, in order, while they start below upto_y."""
    while True:
        seq = dict(p.groups()[Group.PATH]).get("forward")
        if not seq or seq[0][1] > upto_y:
            return
        photograph(p, *seq[0])


def scene():
    """One side (drones 1 and 2) that has photographed up to y ~ 14, found a mine beside the
    route, then found a mine straight ahead: a checked route, a pending detour, the forward
    gap, and rings to photograph."""
    p = PathNodePlanner(W, H, 11.0, drones=(1, 2))
    fly_forward(p, 14.0)
    photograph(p, 17.0, 8.0)                       # a photo that shows a mine beside the route
    p.add_mine(17.0, 8.0, Source.PAIR)
    p.advance()
    fly_forward(p, 14.0)
    p.add_mine(11.0, 25.0, Source.PAIR)            # the mine straight ahead
    p.advance()
    return p


# -- drawing helpers --------------------------------------------------------------------
def base_axes(ax, x0=0.0, x1=W, y0=0.0, y1=H):
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xticks(range(int(x0), int(x1) + 1, 4))
    ax.set_yticks(range(int(y0), int(y1) + 1, 4))
    ax.set_xlabel("x (ft)")
    ax.set_ylabel("y (ft)")
    for v in range(int(x0), int(x1) + 1, 2):
        ax.axvline(v, color="#DDDDDD", lw=0.7, zorder=0)
    for v in range(int(y0), int(y1) + 1, 2):
        ax.axhline(v, color="#DDDDDD", lw=0.7, zorder=0)


def square(ax, c, r, colour, alpha=1.0, z=1, hatch=None):
    """One 2 ft square. A hatched square is drawn with a grey outline so the hatch shows."""
    ax.add_patch(Rectangle((c * CELL, r * CELL), CELL, CELL, facecolor=colour, alpha=alpha,
                           edgecolor="#555555" if hatch else "white", lw=0.8, zorder=z,
                           hatch=hatch))


def seen_squares(p):
    cov = p.coverage
    return [(c, r) for c in range(cov.seen.width) for r in range(cov.seen.height)
            if cov.seen.get(c, r)]


def draw_zones(ax, p):
    for geom in p.routes.zones.groups.values():
        for g in getattr(geom, "geoms", [geom]):
            ax.add_patch(Polygon(list(g.exterior.coords), closed=True, facecolor=ZONE,
                                 edgecolor=INK, lw=1.6, hatch="//", zorder=3))


def draw_mines(ax, p):
    for x, y in p.discovered:
        ax.plot(x, y, marker="X", ms=13, color=INK, mec="white", mew=1.2, zorder=6, ls="")


def draw_links(ax, p, lw=4, nodes=True):
    style = {LinkKind.CHECKED: (BLUE, "-"), LinkKind.PENDING: (ORANGE, "--"),
             LinkKind.FORWARD: (PURPLE, ":")}
    for k in p.links:
        col, ls = style[k.kind]
        xs, ys = zip(*k.poly)
        ax.plot(xs, ys, color=col, ls=ls, lw=lw, zorder=5, solid_capstyle="butt")
    if nodes:
        for n in p.nodes:
            ax.plot(*n.xy, "o", color="white", mec=INK, mew=1.8, ms=8, zorder=7)


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=110, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", name)


def link_legend(extra=()):
    return [Line2D([0], [0], color=BLUE, lw=4, label="checked link (photographed)"),
            Line2D([0], [0], color=ORANGE, lw=4, ls="--",
                   label="pending link (detour, not photographed)"),
            Line2D([0], [0], color=PURPLE, lw=4, ls=":", label="forward gap (unexplored)"),
            *extra]


def below(ax, handles, gap=-0.1, ncol=1):
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, gap), frameon=True,
              ncol=ncol)


# -- figure a: squares and a photo footprint ---------------------------------------------
def fig_squares():
    p = PathNodePlanner(W, H, 11.0)
    cx, cy = 10.0, 9.5
    corners = footprint(p, cx, cy)
    covered = p.coverage.footprint_cells(cx, cy)
    p.record_footprint(corners)
    assert set(seen_squares(p)) == covered
    fig, ax = plt.subplots(figsize=(7, 7.4))
    x0, x1, y0, y1 = 4.0, 18.0, 2.0, 16.0
    base_axes(ax, x0, x1, y0, y1)
    (xa, ya), (xb, yb) = corners[0], corners[2]
    partial = []
    for c in range(int(x0 // 2), int(x1 // 2)):
        for r in range(int(y0 // 2), int(y1 // 2)):
            if (c, r) in covered:
                square(ax, c, r, BLUE, alpha=0.55)
            elif xa < (c + 1) * 2 and c * 2 < xb and ya < (r + 1) * 2 and r * 2 < yb:
                square(ax, c, r, "none", z=4, hatch="xx")
                partial.append((c, r))
    ax.add_patch(Rectangle((xa, ya), xb - xa, yb - ya, fill=False, edgecolor=ORANGE, lw=3.5,
                           ls="--", zorder=4))
    ax.plot(cx, cy, "+", color=INK, ms=14, mew=2.5, zorder=6)
    ax.set_title("A photo counts only the squares it covers completely")
    below(ax, [
        Patch(facecolor=BLUE, alpha=0.55, label=f"photographed ({len(covered)} squares)"),
        Patch(facecolor="none", edgecolor="#777777", hatch="xx",
              label=f"partly inside the photo: NOT counted ({len(partial)})"),
        Line2D([0], [0], color=ORANGE, lw=3.5, ls="--", label="photo footprint (4 x 6 ft)"),
        Line2D([0], [0], color=INK, marker="+", ls="", ms=12, mew=2.5,
               label="photo centre (where the drone was)")], gap=-0.12)
    save(fig, "fig_a_squares_footprint.png")


# -- figure b: the route -----------------------------------------------------------------
def fig_route(p):
    fig, ax = plt.subplots(figsize=(7, 10.5))
    base_axes(ax)
    for c, r in seen_squares(p):
        square(ax, c, r, SEEN)
    draw_zones(ax, p)
    draw_links(ax, p)
    draw_mines(ax, p)
    entry, target = p.nodes[0], p.nodes[-1]
    a = p.point_a()
    ax.annotate("entry", entry.xy, xytext=(entry.x + 4, entry.y + 1.5), fontsize=13,
                arrowprops=dict(arrowstyle="->", color=INK))
    ax.plot(*target.xy, "*", color=PURPLE, ms=22, mec=INK, zorder=8)
    ax.annotate("target", target.xy, xytext=(target.x + 4.5, target.y - 3), fontsize=13,
                arrowprops=dict(arrowstyle="->", color=INK))
    ax.plot(*a, "D", color=YELLOW, mec=INK, mew=2, ms=13, zorder=9)
    ax.annotate("point A", a, xytext=(a[0] + 4.5, a[1] - 3.5), fontsize=13, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=INK))
    ax.set_title("The route: checked, pending and forward links")
    below(ax, link_legend([
        Patch(facecolor=SEEN, label="photographed ground"),
        Patch(facecolor=ZONE, edgecolor=INK, hatch="//", label="no-go zone around a known mine"),
        Line2D([0], [0], color=INK, marker="X", ls="", ms=11, label="known mine"),
        Line2D([0], [0], color="white", marker="o", mec=INK, mew=1.8, ls="", ms=8,
               label="path node (a point on the route)"),
        Line2D([0], [0], color=YELLOW, marker="D", mec=INK, ls="", ms=11,
               label="point A: where the forward gap starts"),
        Line2D([0], [0], color=PURPLE, marker="*", mec=INK, ls="", ms=16, label="target")]),
        gap=-0.07)
    save(fig, "fig_b_route.png")


# -- figure c: the rings -----------------------------------------------------------------
def rings(p):
    w = p.work
    return w.route_cells(), w.adjacent(), w.outer()


def fig_rings(p):
    route, adj, outer = rings(p)
    cov, n = p.coverage, p.config.outer_ring
    skipped = set()
    for c, r in route:
        for dc in range(-n, n + 1):
            for dr in range(-n, n + 1):
                sq = (c + dc, r + dr)
                if (cov.in_field(*sq) and not cov.seen.get(*sq) and p.routes.in_zone(*sq)):
                    skipped.add(sq)
    assert skipped, "the example should show at least one skipped no-go-zone square"
    fig, ax = plt.subplots(figsize=(7, 8.6))
    base_axes(ax, 0, W, 0, 24)
    for c, r in seen_squares(p):
        if (c, r) not in route:
            square(ax, c, r, SEEN, z=1)
    for c, r in route:
        square(ax, c, r, BLUE, alpha=0.75, z=2)
    for c, r in adj:
        square(ax, c, r, YELLOW, z=2)
    for c, r in outer:
        square(ax, c, r, SKY, z=2)
    for c, r in skipped:
        square(ax, c, r, "none", z=4, hatch="xx")
    draw_zones(ax, p)
    draw_mines(ax, p)
    draw_links(ax, p, lw=3)
    ax.set_title("Squares around the checked route: two rings")
    below(ax, [
        Patch(facecolor=BLUE, alpha=0.75, label="route squares (touched by a checked link)"),
        Patch(facecolor=YELLOW, label="ADJACENT ring: 1 square out, still to photograph"),
        Patch(facecolor=SKY, label="OUTER ring: 2 squares out, still to photograph"),
        Patch(facecolor=SEEN, label="already photographed: nothing to do"),
        Patch(facecolor=ZONE, edgecolor=INK, hatch="//", label="no-go zone around a mine"),
        Patch(facecolor="none", edgecolor="#777777", hatch="xx",
              label="would be in a ring, but inside a zone: skipped")], gap=-0.09)
    save(fig, "fig_c_rings.png")


# -- figure d: photos along a path vs tiles over scattered squares -----------------------
def draw_foot(ax, p, pt, colour, lw=2.0, alpha=0.9):
    c = footprint(p, *pt)
    (xa, ya), (xb, yb) = c[0], c[2]
    ax.add_patch(Rectangle((xa, ya), xb - xa, yb - ya, fill=False, edgecolor=colour, lw=lw,
                           alpha=alpha, zorder=6))
    ax.plot(*pt, "+", color=colour, ms=9, mew=2, zorder=7)


def fig_placement(p):
    g = p.groups()
    paths = dict(g[Group.PATH])
    span_key = next(k for k in paths if k != "forward")
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 8.6), sharey=True)
    for ax in (a1, a2):
        base_axes(ax, 0, W, 0, 34)
        for c, r in seen_squares(p):
            square(ax, c, r, SEEN)
        draw_zones(ax, p)
        draw_mines(ax, p)
        draw_links(ax, p, lw=3, nodes=False)
    a2.set_ylabel("")
    need = set()
    for k in p.chain.unchecked():
        need |= p.coverage.need(k.poly)
    for c, r in p.coverage.unseen(need):
        square(a1, c, r, ORANGE, alpha=0.45, z=2)
    for pt in paths["forward"]:
        draw_foot(a1, p, pt, PURPLE)
    for pt in paths[span_key]:
        draw_foot(a1, p, pt, "#B35F00")
    a1.set_title("Along a path:\none sequence, in flying order")
    _route, adj, outer = rings(p)
    for c, r in adj:
        square(a2, c, r, YELLOW, z=2)
    tiles = [seq[0] for _k, seq in g[Group.ADJACENT]]
    for pt in tiles:
        draw_foot(a2, p, pt, BLUE)
    a2.set_title("Over scattered squares:\nindependent photos (ADJACENT)")
    below(a1, [
        Patch(facecolor=ORANGE, alpha=0.45, label="squares the two paths still need"),
        Line2D([0], [0], color=PURPLE, lw=2, label="photo for the forward gap"),
        Line2D([0], [0], color="#B35F00", lw=2, label="photo for the detour")], gap=-0.1)
    below(a2, [
        Patch(facecolor=YELLOW, label="ADJACENT squares still to photograph"),
        Line2D([0], [0], color=BLUE, lw=2, label="one photo (each is its own job)")], gap=-0.1)
    fig.tight_layout()
    save(fig, "fig_d_placement.png")
    return len(paths["forward"]), len(paths[span_key]), len(tiles), len(adj)


# -- figure e: the worked example --------------------------------------------------------
def worked_example(p):
    """Two drones dealt the PATH group by the real ClaimBook. Replays the ranking by hand and
    asserts it agrees with ClaimBook's claims; returns what the figure and doc quote."""
    groups = p.groups()
    paths = dict(groups[Group.PATH])
    span_key = next(k for k in paths if k != "forward")
    seqs = {"forward": paths["forward"], "span": paths[span_key]}
    pos = {1: (14.0, 21.0), 2: (11.0, 12.0)}           # drone 1 is ahead, drone 2 is behind
    book = ClaimBook((1, 2))
    plan = book.update(groups, pos)
    cost, tail = {1: 0.0, 2: 0.0}, dict(pos)
    first_rank = {(d, s): math.dist(pos[d], seqs[s][0]) for d in (1, 2) for s in seqs}
    owner, order, left = {}, [], dict(seqs)
    while left:
        rank, d, s = min(((cost[d] + math.dist(tail[d], left[s][0]), d, s)
                          for d in (1, 2) for s in left), key=lambda t: (t[0], t[1]))
        owner[s] = d
        order.append((d, s, rank))
        cost[d], tail[d] = rank + path_length(left[s]), left[s][-1]
        del left[s]
    for s, d in owner.items():
        assert book.claims[(0, "forward" if s == "forward" else span_key)] == d
    return dict(pos=pos, seqs=seqs, owner=owner, order=order, first_rank=first_rank, plan=plan,
                cost=cost, tail=tail, groups=groups)


def fig_worked(p, ex):
    pos, owner = ex["pos"], ex["owner"]
    col = {1: BLUE, 2: ORANGE}
    fig, ax = plt.subplots(figsize=(7.4, 10.8))
    base_axes(ax)
    for c, r in seen_squares(p):
        square(ax, c, r, SEEN)
    draw_zones(ax, p)
    draw_links(ax, p, lw=3)
    draw_mines(ax, p)
    for s, seq in ex["seqs"].items():
        d = owner[s]
        for pt in seq:
            draw_foot(ax, p, pt, col[d], lw=1.6, alpha=0.7)
        ax.plot(*seq[0], "s", color=col[d], mec=INK, ms=11, zorder=9)
    for d in (1, 2):
        ax.plot(*pos[d], "o", color=col[d], mec=INK, mew=2.5, ms=22, zorder=10)
        ax.text(pos[d][0], pos[d][1], str(d), color="white" if d == 1 else INK, ha="center",
                va="center", fontsize=12, fontweight="bold", zorder=11)
    names = {"span": "the detour", "forward": "the forward gap"}
    where = {1: (16.3, 21.0), 2: (13.2, 14.6)}
    for n, (d, s, rank) in enumerate(ex["order"], 1):
        start = ex["seqs"][s][0]
        ax.annotate("", start, pos[d], zorder=12,
                    arrowprops=dict(arrowstyle="-|>", color=col[d], lw=3.0, shrinkA=13,
                                    shrinkB=6))
        ax.text(*where[d], f"deal {n}: drone {d} takes\n{names[s]} (rank {rank:.1f})",
                color=INK, fontsize=12, fontweight="bold", ha="left", va="center", zorder=13,
                bbox=dict(facecolor="white", edgecolor=col[d], lw=2, boxstyle="round,pad=0.25"))
    ax.set_title("Worked example: who is dealt the PATH group")
    below(ax, [
        Line2D([0], [0], color=BLUE, marker="o", mec=INK, lw=3, ms=13,
               label="drone 1, and the photos it is dealt"),
        Line2D([0], [0], color=ORANGE, marker="o", mec=INK, lw=3, ms=13,
               label="drone 2, and the photos it is dealt"),
        Line2D([0], [0], color=INK, marker="s", ls="", ms=10,
               label="first photo of a sequence (arrow = trip there)")] + link_legend(), gap=-0.07)
    save(fig, "fig_e_worked_example.png")


def main():
    fig_squares()
    p = scene()
    fig_route(p)
    fig_rings(p)
    n_f, n_s, n_t, n_adj = fig_placement(p)
    ex = worked_example(p)
    fig_worked(p, ex)
    print("forward photos", n_f, "span photos", n_s, "adjacent tiles", n_t,
          "adjacent squares", n_adj)
    print("nodes", [(n.xy, n.placed_by.value) for n in p.nodes])
    print("links", [(k.kind.value, [tuple(round(v, 1) for v in q) for q in k.poly])
                    for k in p.links])
    print("pos", ex["pos"], "first_rank", {k: round(v, 1) for k, v in ex["first_rank"].items()})
    print("order", [(d, s, round(r, 1)) for d, s, r in ex["order"]], "cost", ex["cost"])
    print("seqs", ex["seqs"])
    print("plan", {d: [(round(x, 1), round(y, 1), g) for x, y, g in q][:6]
                   for d, q in ex["plan"].items()})


if __name__ == "__main__":
    main()
