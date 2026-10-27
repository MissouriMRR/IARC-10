"""Photo centres along a polyline's unphotographed stretches (no cell-grid covering).

Entry point: path_cover_unseen(path_points, seen_field, shape_size, overlap, path_width).
Used by pathnode/placement.py (CorridorPlacement).

Steps:
1. unseen_path_runs: walk the polyline's cells, cut it into runs wherever a cell is already
   seen or the walk leaves and re-enters the field.
2. place_along_runs: along each run, a centre every `spacing` of arc length (plus extra
   parallel rows for a corridor wider than the footprint).
3. path_cover_unseen: add a centre at each real interior vertex that is still unseen.

Why the spacing is enough:
- Square S x S footprint, axis-aligned, centred on a straight segment: the segment inside it is
  at least S long. Centres S apart therefore always touch or overlap.
- Rectangle (along, across): the length inside varies with the segment angle, from `along`
  down to `across`, and never below min(along, across) (reached at sharp bends). So the spacing
  is always min(along, across) * (1 - overlap). `across` alone sets the gap between rows.
- A positive `overlap` only tightens the spacing. Prefer overlap >= 0.1:
  - overlap 0 with several rows leaves floating-point seams (razor-thin gaps).
  - overlap 0 can leave a seam cell not fully inside either neighbour, so it is not marked
    seen and costs an extra cleanup replan.

Rows (`path_width` > footprint): parallel copies of the polyline, each offset sideways by the
local normal at each vertex (averaged at bends: exact on a straight run, approximate at a bend).
"""

import math


def _clip_segment_to_bbox(p0, p1, min_corner, max_corner):
    """Liang-Barsky clip of segment p0-p1 to the rectangle [min_corner, max_corner].

    Returns: the clipped (q0, q1), or None if the segment never touches the rectangle.
    """
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    t_enter, t_exit = 0.0, 1.0
    for p, q in (
        (-dx, x0 - min_corner[0]),
        (dx, max_corner[0] - x0),
        (-dy, y0 - min_corner[1]),
        (dy, max_corner[1] - y0),
    ):
        if p == 0:
            if q < 0:
                return None
            continue
        t = q / p
        if p < 0:
            if t > t_exit:
                return None
            t_enter = max(t_enter, t)
        else:
            if t < t_enter:
                return None
            t_exit = min(t_exit, t)
    if t_enter > t_exit:
        return None
    return (x0 + t_enter * dx, y0 + t_enter * dy), (x0 + t_exit * dx, y0 + t_exit * dy)


def _first_and_last_in_bounds_points(points, min_corner, max_corner):
    """The exact points where `points` first enters and last leaves the rectangle.

    Returns: (first, last), or (None, None) if it never touches the field.
    Why: the cell walk only says which cell the path starts and ends in, and that cell's
    centre is off the true crossing by up to a cell, which would erode the coverage at the two
    ends of the path. Interior seen/unseen changes are cell-level anyway, so need no fix.
    """
    first = last = None
    for i in range(len(points) - 1):
        clipped = _clip_segment_to_bbox(points[i], points[i + 1], min_corner, max_corner)
        if clipped is None:
            continue
        if first is None:
            first = clipped[0]
        last = clipped[1]
    return first, last


def _polyline_normals_per_vertex(points):
    """One unit normal per vertex: the mean of its adjacent segments' normals (miter-style).

    Lets the whole polyline be offset sideways and stay one connected line.
    """
    n = len(points)
    seg_normals = []
    for i in range(n - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        length = math.hypot(dx, dy)
        seg_normals.append((0.0, 0.0) if length == 0 else (-dy / length, dx / length))

    vertex_normals = []
    for i in range(n):
        candidates = [seg_normals[i - 1]] if i > 0 else []
        if i < n - 1:
            candidates.append(seg_normals[i])
        avx = sum(c[0] for c in candidates) / len(candidates)
        avy = sum(c[1] for c in candidates) / len(candidates)
        norm = math.hypot(avx, avy)
        vertex_normals.append((0.0, 0.0) if norm == 0 else (avx / norm, avy / norm))
    return vertex_normals


def _offset_polyline(points, offset):
    """`points` moved sideways by `offset` along each vertex's normal."""
    if offset == 0.0:
        return list(points)
    normals = _polyline_normals_per_vertex(points)
    return [(p[0] + offset * nrm[0], p[1] + offset * nrm[1]) for p, nrm in zip(points, normals)]


def _row_offsets(path_width, shape_size, spacing):
    """Sideways offset of each parallel row needed to cover a corridor `path_width` wide.

    One centred row (offset 0.0) is enough when path_width <= shape_size.
    """
    if path_width <= shape_size:
        return [0.0]
    num_rows = math.ceil((path_width - shape_size) / spacing) + 1
    total_span = (num_rows - 1) * spacing
    return [-total_span / 2.0 + i * spacing for i in range(num_rows)]


def _normalize_shape_size(shape_size):
    """(along, across) from `shape_size`.

    - number: a square, along == across.
    - (along, across) pair: a rectangle; `along` is parallel to travel, `across` is
      perpendicular (it sets the gap between rows).
    """
    if isinstance(shape_size, tuple):
        along, across = shape_size
        return along, across
    return shape_size, shape_size


def _place_along_polyline(points, spacing):
    """Centres every `spacing` of arc length along one run.

    - First centre at spacing/2 (the first footprint's leading edge reaches the start).
    - A final centre at the last point if the last spaced one is more than spacing/2 from it.
    - No vertex pinning: `points` are cell samples (about one per 2 ft), not real corners.
      Real corners are handled by path_cover_unseen.
    """
    if len(points) == 1:
        return [points[0]]
    seg_lengths = [
        math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        for i in range(len(points) - 1)
    ]
    total = sum(seg_lengths)
    if total == 0:
        return [points[0]]

    centers = []
    target = spacing / 2.0
    seg_idx = 0
    seg_start_cum = 0.0
    while target <= total:
        while seg_idx < len(seg_lengths) and seg_start_cum + seg_lengths[seg_idx] < target:
            seg_start_cum += seg_lengths[seg_idx]
            seg_idx += 1
        if seg_idx >= len(seg_lengths):
            break
        seg_len = seg_lengths[seg_idx]
        local_t = (target - seg_start_cum) / seg_len if seg_len > 0 else 0.0
        p0, p1 = points[seg_idx], points[seg_idx + 1]
        centers.append((p0[0] + local_t * (p1[0] - p0[0]), p0[1] + local_t * (p1[1] - p0[1])))
        target += spacing

    if (
        not centers
        or math.hypot(centers[-1][0] - points[-1][0], centers[-1][1] - points[-1][1])
        > spacing / 2.0
    ):
        centers.append(points[-1])
    return centers


def place_along_runs(runs, shape_size, overlap: float = 0.0, path_width: float = 0.0):
    """Photo centres for each run on its own (runs are independent polylines of (x, y)).

    - shape_size: number (square) or (along, across); see _normalize_shape_size.
    - overlap: fraction in [0, 1) of the footprint by which neighbours overlap.
    - path_width: corridor width to cover; wider than the footprint gives parallel rows.
    Returns: list of (x, y) centres.
    """
    if not (0.0 <= overlap < 1.0):
        raise ValueError(f"overlap must satisfy 0<=overlap<1, got {overlap}")
    if path_width < 0.0:
        raise ValueError(f"path_width must be non-negative, got {path_width}")

    along, across = _normalize_shape_size(shape_size)
    # The shorter dimension, not `along`: see the module docstring.
    along_spacing = min(along, across) * (1.0 - overlap)
    across_spacing = across * (1.0 - overlap)
    row_offsets = _row_offsets(path_width, across, across_spacing)
    centers = []
    for run in runs:
        for row_offset in row_offsets:
            centers.extend(_place_along_polyline(_offset_polyline(run, row_offset), along_spacing))
    return centers


def unseen_path_runs(path_points, seen_field):
    """Split `path_points` into runs of real-world points whose cells are not yet seen.

    - A run breaks at a seen cell, and at a gap in the cell walk (the path left the field and
      re-entered; interpolating across it would be wrong).
    - Uses path_cells(include_tie_neighbors=True): the strict cardinal walk can skip a cell at an
      exact grid-corner tie that mark_path counts as on the path, which would be a permanent
      coverage gap. Ties are common because mine positions snap to the 2 ft grid.
    - The path's first and last points use the exact field crossing, not the cell centre.
    Returns: list of runs, each a list of (x, y).
    """
    cells = seen_field.path_cells(path_points, include_tie_neighbors=True)
    if not cells:
        return []

    true_first, true_last = _first_and_last_in_bounds_points(
        path_points, seen_field.min_corner, seen_field.max_corner
    )

    runs = []
    current = []
    prev_cell = None
    last_idx = len(cells) - 1
    for idx, (col, row) in enumerate(cells):
        adjacent = (
            prev_cell is not None and abs(col - prev_cell[0]) <= 1 and abs(row - prev_cell[1]) <= 1
        )
        if not adjacent and current:
            runs.append(current)
            current = []
        if seen_field.get(col, row):
            if current:
                runs.append(current)
                current = []
        else:
            if idx == 0 and true_first is not None:
                point = true_first
            elif idx == last_idx and true_last is not None:
                point = true_last
            else:
                point = seen_field.cell_to_real(col, row)
            current.append(point)
        prev_cell = (col, row)
    if current:
        runs.append(current)
    return runs


def path_cover_unseen(
    path_points,
    seen_field,
    shape_size,
    overlap: float = 0.0,
    path_width: float = 0.0,
):
    """Photo centres covering the unseen stretches of the polyline `path_points`.

    - path_points: ordered real-world (x, y) vertices, straight segments between them.
    - seen_field: CellField of already-photographed cells.
    - shape_size, overlap, path_width: see place_along_runs.
    - Corner guarantee: after the arc-spaced centres, every interior vertex still unseen and not
      within min(along, across) * (1 - overlap) / 4 of a centre gets a centre. (Short sharp
      bends can otherwise leave the corner cell out of reach of every spaced centre.)
    Returns: list of (x, y) centres (not sorted along the path).
    """
    unseen_runs = unseen_path_runs(path_points, seen_field)
    centers = place_along_runs(unseen_runs, shape_size, overlap=overlap, path_width=path_width)

    along, across = _normalize_shape_size(shape_size)
    dedup_dist = min(along, across) * (1.0 - overlap) / 4.0
    for x, y in path_points[1:-1]:
        col, row = seen_field.real_to_cell(x, y)
        if not (0 <= col < seen_field.width and 0 <= row < seen_field.height):
            continue
        if seen_field.get(col, row):
            continue
        if not any(math.hypot(x - cx, y - cy) < dedup_dist for cx, cy in centers):
            centers.append((x, y))

    return centers
