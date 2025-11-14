import numpy as np

def compute_affine_transform(src_pts, dst_pts):
    """
    Compute 2D affine transform matrix A that maps src_pts -> dst_pts.

    Parameters:
        src_pts: list of (x, y) tuples (length 3 or 4)
        dst_pts: list of (lat, lon) tuples (same length)

    Returns:
        3x3 affine transform matrix (NumPy array)
    """
    src = np.array([[x, y, 1] for (x, y) in src_pts])
    dst = np.array(dst_pts)

    # Solve least squares for affine matrix coefficients
    A, _, _, _ = np.linalg.lstsq(src, dst, rcond=None)
    A = np.vstack([A.T, [0, 0, 1]])
    return A

def grid_to_latlon_affine(x, y, transform):
    """
    Apply affine transform to convert grid coordinates (x, y) to (lat, lon).
    """
    vec = np.array([x, y, 1])
    lat, lon, _ = transform @ vec
    return lat, lon
