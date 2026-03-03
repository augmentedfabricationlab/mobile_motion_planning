"""A tiny dependency-free helper for rigid transforms between planes."""

from math import sqrt
from .geometry import Plane


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    return sqrt(max(_dot(a, a), 0.0))


def _normalize(a):
    n = _norm(a)
    if n == 0:
        raise ValueError("Zero-length vector cannot be normalized.")
    return _scale(a, 1.0 / n)


def _mat3_cols(c0, c1, c2):
    # 3x3 matrix stored as tuple of 3 column vectors
    return (c0, c1, c2)


def _mat3_transpose(matrix):
    # transpose: columns -> rows
    c0, c1, c2 = matrix
    return ((c0[0], c1[0], c2[0]), (c0[1], c1[1], c2[1]), (c0[2], c1[2], c2[2]))


def _mat3_multiplication(mat_a, mat_b):
    # A(3x3) * B(3x3)
    a_trans = _mat3_transpose(mat_a)  # rows of A
    b0, b1, b2 = mat_b
    cols = []
    for b in (b0, b1, b2):
        cols.append(
            (
                _dot(a_trans[0], b),
                _dot(a_trans[1], b),
                _dot(a_trans[2], b),
            )
        )
    return (cols[0], cols[1], cols[2])


def _mat3_times_vec(matrix, v):
    mat_trans = _mat3_transpose(matrix)
    return (_dot(mat_trans[0], v), _dot(mat_trans[1], v), _dot(mat_trans[2], v))


def _homogeneous(rotation, translation):
    """make 4x4 from R (3x3 as 3 column tuples) and t (3,)."""
    (c0, c1, c2) = rotation
    return [
        [c0[0], c1[0], c2[0], translation[0]],
        [c0[1], c1[1], c2[1], translation[1]],
        [c0[2], c1[2], c2[2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def orthonormal_frame(origin, xaxis, yaxis):
    """
    Build a right-handed orthonormal frame from origin, xaxis, yaxis.
    z = x × y, then re-orthonormalize y := z × x to ensure orthogonality.
    Returns (origin, x, y, z) with unit-length axes.
    """
    x = _normalize(xaxis)
    z = _normalize(_cross(xaxis, yaxis))
    # if xaxis and yaxis were nearly parallel, cross is unstable:
    if _norm(z) == 0:
        raise ValueError("Input axes are collinear; cannot form a plane.")
    y = _cross(z, x)  # already orthonormal if x and z are unit and orthogonal
    return (tuple(origin), x, y, z)


def plane_basis_matrix(frame):
    """
    Given (origin, x, y, z), return 3x3 rotation with columns [x y z].
    """
    _, x, y, z = frame
    return _mat3_cols(x, y, z)


def invert_transform(transformation):
    """Invert T = [R t; 0 1] given R (3x3 as 3 column tuples) and t (3,)."""
    r = (
        (transformation[0][0], transformation[0][1], transformation[0][2]),
        (transformation[1][0], transformation[1][1], transformation[1][2]),
        (transformation[2][0], transformation[2][1], transformation[2][2]),
    )
    t = (transformation[0][3], transformation[1][3], transformation[2][3])
    # transpose of 3x3 stored-as-columns
    # t_inv = -R^T t
    tx = -(r[0][0] * t[0] + r[1][0] * t[1] + r[2][0] * t[2])
    ty = -(r[0][1] * t[0] + r[1][1] * t[1] + r[2][1] * t[2])
    tz = -(r[0][2] * t[0] + r[1][2] * t[1] + r[2][2] * t[2])
    transformation_inverse = [
        [r[0][0], r[1][0], r[2][0], tx],
        [r[0][1], r[1][1], r[2][1], ty],
        [r[0][2], r[1][2], r[2][2], tz],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return transformation_inverse


# def invert_transform(R, t):
#     """Invert T = [R t; 0 1] given R (3x3 as 3 column tuples) and t (3,)."""
#     # transpose of 3x3 stored-as-columns
#     RT = ((R[0][0], R[1][0], R[2][0]),
#           (R[0][1], R[1][1], R[2][1]),
#           (R[0][2], R[1][2], R[2][2]))
#     # t_inv = -R^T t
#     tx = -(RT[0][0]*t[0] + RT[0][1]*t[1] + RT[0][2]*t[2])
#     ty = -(RT[1][0]*t[0] + RT[1][1]*t[1] + RT[1][2]*t[2])
#     tz = -(RT[2][0]*t[0] + RT[2][1]*t[1] + RT[2][2]*t[2])
#     Tinverse = [
#         [RT[0][0], RT[0][1], RT[0][2], tx],
#         [RT[1][0], RT[1][1], RT[1][2], ty],
#         [RT[2][0], RT[2][1], RT[2][2], tz],
#         [0.0, 0.0, 0.0, 1.0]
#     ]
#     return RT, (tx, ty, tz), Tinverse


def compose(transformation_1, transformation_2):
    """Matrix multiply 4x4: returns T = T1 * T2."""
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                transformation_1[i][0] * transformation_2[0][j]
                + transformation_1[i][1] * transformation_2[1][j]
                + transformation_1[i][2] * transformation_2[2][j]
                + transformation_1[i][3] * transformation_2[3][j]
            )
    return out


# def apply_transform_plane_as_frame(T, plane, renormalize=True):
#     """Apply 4x4 to plane-dict -> return Frame-like object."""
#     o = apply_transform_point(T, plane["origin"])
#     x = apply_transform_vector(T, plane["xaxis"])
#     y = apply_transform_vector(T, plane["yaxis"])
#     if renormalize:
#         _, x_n, y_n, z_n = orthonormal_frame(o, x, y)
#     else:
#         # quick z from x×y; assumes perfect rigidity
#         from math import sqrt
#         def cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
#         def norm(a): return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
#         z = cross(x, y); n = norm(z)
#         z_n = (z[0]/n, z[1]/n, z[2]/n)
#         x_n, y_n = x, y
#     Frame = type("Frame", (object,), {})
#     fr = Frame()
#     fr.point, fr.xaxis, fr.yaxis, fr.zaxis = o, x_n, y_n, z_n
#     return fr


def rigid_transform_plane_to_plane(plane_a, plane_b):
    """
    Compute rigid transform (R, t, T4x4) mapping planeA -> planeB.

    planeA, planeB format:
        {
          "origin": (x,y,z),
          "xaxis":  (x,y,z),
          "yaxis":  (x,y,z)
        }
    Assumes right-handed frame; z is constructed as x×y.

    Returns:
      R: 3x3 rotation (as 3 column vectors)
      t: 3-vector translation
      T: 4x4 homogeneous matrix (list of lists)
    """
    f_a = orthonormal_frame(plane_a.origin, plane_a.xaxis, plane_a.yaxis)
    f_b = orthonormal_frame(plane_b.origin, plane_b.xaxis, plane_b.yaxis)

    # Extract origins from frames (first element of tuple)
    p_a, *_ = f_a
    p_b, *_ = f_b

    a = plane_basis_matrix(f_a)  # columns = xA yA zA
    b = plane_basis_matrix(f_b)  # columns = xB yB zB

    # For orthonormal bases, A^{-1} = A^T ⇒ R = B * A^T
    r = _mat3_multiplication(b, _mat3_transpose(a))

    # Translation: pB = R * pA + t  ⇒  t = pB - R*pA
    p_a_rotated = _mat3_times_vec(r, p_a)
    t = _sub(p_b, p_a_rotated)

    transformation = _homogeneous(r, t)
    return transformation


def apply_transform_point(transformation, pt):
    """Apply 4x4 to a 3D point."""
    x = (
        transformation[0][0] * pt[0]
        + transformation[0][1] * pt[1]
        + transformation[0][2] * pt[2]
        + transformation[0][3]
    )
    y = (
        transformation[1][0] * pt[0]
        + transformation[1][1] * pt[1]
        + transformation[1][2] * pt[2]
        + transformation[1][3]
    )
    z = (
        transformation[2][0] * pt[0]
        + transformation[2][1] * pt[1]
        + transformation[2][2] * pt[2]
        + transformation[2][3]
    )
    return (x, y, z)


def apply_transform_vector(transformation, v):
    """Apply rotation part only (ignore translation)."""
    x = (
        transformation[0][0] * v[0]
        + transformation[0][1] * v[1]
        + transformation[0][2] * v[2]
    )
    y = (
        transformation[1][0] * v[0]
        + transformation[1][1] * v[1]
        + transformation[1][2] * v[2]
    )
    z = (
        transformation[2][0] * v[0]
        + transformation[2][1] * v[1]
        + transformation[2][2] * v[2]
    )
    return (x, y, z)


def apply_transform_plane(transformation, plane, renormalize=True):
    """
    Apply a 4x4 rigid transform T to a plane and return a Frame-like object
    with attributes: point, xaxis, yaxis, zaxis.

    plane format:
        {"origin": (x,y,z), "xaxis": (x,y,z), "yaxis": (x,y,z)}

    Returns:
        frame: object with attributes .point, .xaxis, .yaxis, .zaxis
    """

    # Transform origin as point, axes as vectors
    o_t = apply_transform_point(transformation, plane.origin)
    x_t = apply_transform_vector(transformation, plane.xaxis)
    y_t = apply_transform_vector(transformation, plane.yaxis)

    if renormalize:
        _, x_n, y_n, z_n = orthonormal_frame(o_t, x_t, y_t)
    else:
        # Construct z without renormalization — only safe for perfect rigid transforms
        z_n = _normalize(_cross(x_t, y_t))
        x_n = x_t
        y_n = y_t

    # Build the plane dictionary
    plane = Plane(origin=o_t, xaxis=x_n, yaxis=y_n, zaxis=z_n)
    return plane
