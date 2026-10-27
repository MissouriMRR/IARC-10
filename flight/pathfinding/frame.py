"""The field's local frame: lat/lon <-> feet.

Corner 1 is the origin (moved along the 1->2 edge if the 2-1-4 angle is obtuse), the 1->2 edge
is +x, and +y runs towards corners 3/4. Distances are true feet: the base edge is measured, not
assumed. Every drone builds the same frame from the same corners, so local coordinates agree
across drones; everything sent between drones stays lat/lon.

FieldFrame is the one object that converts: build it from the corners, then use `to_local` /
`to_latlon` (single points) and `points_to_local` / `points_to_latlon` (lists).
"""

import shapely

from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer

FT_PER_M = 3.28084


def field_transformer(corners_latlon):
    """(transformer, width_ft). SimToLatLonTransformer scales its base edge (corner 1 -> 2,
    from the origin it picks) to a given length; measure that edge first and give it its real
    length, so a foot in the planner is a foot on the ground."""
    probe = SimToLatLonTransformer(corners_latlon, 1.0)
    width_ft = FT_PER_M / probe.scale
    return SimToLatLonTransformer(corners_latlon, width_ft), width_ft


def local_field_corners(corners_latlon):
    """The four field corners in the local frame (true feet), in the given order."""
    t, _w = field_transformer(corners_latlon)
    return [t.latlon_to_local(lat, lon) for lat, lon in corners_latlon]


class FieldFrame:
    """The local frame of one field, built from its 4 corners (lat, lon), counter-clockwise.

    - width, height: the field's size in true feet.
    - to_local(lat, lon) -> (x, y); to_latlon(x, y) -> (lat, lon): plain floats.
    - points_to_local / points_to_latlon: the same for a list of points.
    Raises ValueError (validate_field_corners) if the corners are not a simple CCW quad.
    """

    def __init__(self, corners_latlon):
        _require_four(corners_latlon)
        self._t, self.width = field_transformer(corners_latlon)
        corners_local = [self._t.latlon_to_local(lat, lon) for lat, lon in corners_latlon]
        _check_quad(corners_local)
        self.height = max(y for _x, y in corners_local)

    def to_local(self, lat, lon):
        x, y = self._t.latlon_to_local(lat, lon)
        return float(x), float(y)

    def to_latlon(self, x, y):
        lat, lon = self._t.local_to_latlon(x, y)
        return float(lat), float(lon)

    def points_to_local(self, pts_latlon):
        return [self.to_local(lat, lon) for lat, lon in pts_latlon]

    def points_to_latlon(self, pts_local):
        return [self.to_latlon(x, y) for x, y in pts_local]


def validate_field_corners(corners_latlon):
    """
    Raise ValueError unless the corners trace a simple, counter-clockwise quadrilateral:
    origin, +x, the diagonal corner, +y. Listing them in a "Z" (the last two swapped) traces a
    bow-tie that encloses almost nothing, and clockwise corners put the field below the x
    axis -- both silently wreck the plan, so they fail here, at config load.
    """
    _require_four(corners_latlon)
    _check_quad(local_field_corners(corners_latlon))


def _require_four(corners_latlon):
    if len(corners_latlon) != 4:
        raise ValueError(f"mission_field_corners needs 4 corners, got {len(corners_latlon)}")


def _check_quad(pts):
    """validate_field_corners' checks, on the corners already in local feet."""
    if len({(round(x, 3), round(y, 3)) for x, y in pts}) < 4:
        raise ValueError("mission_field_corners has repeated corners")
    quad = shapely.Polygon(pts)
    if quad.is_valid and not quad.exterior.is_ccw:
        raise ValueError(
            "mission_field_corners go clockwise -- list them counter-clockwise seen from above:"
            " corner 1 (the origin), corner 2 along the start edge, then 3, then 4")
    hull = quad.convex_hull
    if not quad.is_valid or quad.area < 0.8 * hull.area:
        swapped = shapely.Polygon([pts[0], pts[1], pts[3], pts[2]])
        hint = (" -- the last two corners look swapped (the list reads as a 'Z', it should go"
                " around the edge)" if swapped.is_valid else "")
        raise ValueError(
            f"mission_field_corners do not trace a simple quadrilateral (encloses "
            f"{quad.area:.0f} sq ft of a {hull.area:.0f} sq ft hull){hint}")
