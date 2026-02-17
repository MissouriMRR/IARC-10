import numpy as np

class GridToLatLonTransformer:
    affine_transform = None
    def __init__(self, src_pts, dst_pts):
        """
        Initialize the transformer with source and destination points.

        Parameters:
            src_pts: list of (x, y) tuples (length 3 or 4)
            dst_pts: list of (lat, lon) tuples (same length)
        """
        self.affine_transform = self.compute_affine_transform(src_pts, dst_pts)

    def compute_affine_transform(self, src_pts, dst_pts):
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
    
    def grid_to_latlon(self, x, y):
        """
        Apply affine transform to convert grid coordinates (x, y) to (lat, lon).
        """
        vec = np.array([x, y, 1])
        lat, lon, _ = self.affine_transform @ vec
        return lat, lon
    
    def latlon_to_grid(self, lat, lon):
        """
        Apply inverse affine transform to convert (lat, lon) to grid coordinates (x, y).
        """
        inv_transform = np.linalg.inv(self.affine_transform)
        vec = np.array([lat, lon, 1])
        x, y, _ = inv_transform @ vec
        return x, y


src_pts = [(10, 20), (30, 40), (50, 60), (70, 80)]
dst_pts = [(35.1, -97.5), (35.2, -97.6), (35.3, -97.7), (35.4, -97.8)]

transformer = GridToLatLonTransformer(src_pts, dst_pts)
lat, lon = transformer.grid_to_latlon(25, 35)
print(lat, lon)  # Output: latitude, longitude estimate
