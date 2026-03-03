import numpy as np
from .geometry import Plane

def matrix_inverse(T):
    """Invert a matrix with a fast path for 4x4 rigid transforms.

    For SE(3) matrices with orthonormal rotation blocks, R^-1 = R^T and
    t^-1 = -R^T @ t. Falls back to np.linalg.inv for other shapes.
    """
    # Fast path: 4x4 homogeneous transform
    if T.shape == (4, 4):
        R = T[:3, :3]
        t = T[:3, 3]

        R_inv = R.T
        t_inv = -R_inv @ t

        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        return T_inv

    # Fallback for arbitrary matrices
    return np.linalg.inv(T)

def world_to_plane(plane):
    """Create a transformation matrix from world coordinates to the given plane. Or turn a plane into a matrix."""
    T = np.array([
        [plane.xaxis[0], plane.yaxis[0], plane.zaxis[0], plane.origin[0]],
        [plane.xaxis[1], plane.yaxis[1], plane.zaxis[1], plane.origin[1]],
        [plane.xaxis[2], plane.yaxis[2], plane.zaxis[2], plane.origin[2]],
        [0.0,      0.0,      0.0,      1.0],
    ])
    return T

def plane_to_world(plane):
    T = world_to_plane(plane)
    T = matrix_inverse(T)
    return T


def from_plane_to_plane(plane_from, plane_to):
    t_from = plane_to_world(plane_from)
    t_to = world_to_plane(plane_to)
    T = t_to @ t_from
    return T


def apply_T_to_plane(transformation, geometry=None) -> Plane:
    """Apply a transform to a plane; avoids deepcopy for speed."""
    if geometry is None:
        geometry = Plane(
            origin=(0.0, 0.0, 0.0),
            xaxis=(1.0, 0.0, 0.0),
            yaxis=(0.0, 1.0, 0.0),
        )

    X = transformation @ world_to_plane(geometry)
    point = X[0:3, 3]
    xaxis = X[0:3, 0]
    yaxis = X[0:3, 1]
    transformed_plane = Plane(
        origin=(point[0], point[1], point[2]),
        xaxis=(xaxis[0], xaxis[1], xaxis[2]),
        yaxis=(yaxis[0], yaxis[1], yaxis[2]),
    )
    return transformed_plane

