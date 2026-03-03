"""Geometry definitions for IK offline module."""

import math
from dataclasses import dataclass
from typing import Tuple
import numpy as np
np.set_printoptions(precision=16, suppress=False)

@dataclass(slots=True)
class Plane:
    """A 3D plane defined by an origin and orthonormal axes."""
    origin: Tuple[np.float64, np.float64, np.float64]
    xaxis: Tuple[np.float64, np.float64, np.float64]
    yaxis: Tuple[np.float64, np.float64, np.float64]
    zaxis: Tuple[np.float64, np.float64, np.float64] = None
    

    def __post_init__(self):
        """Calculate zaxis if not provided and validate orthogonality."""
        # Convert inputs to float64 arrays (avoid copies when possible)
        x = _as_float64_array(self.xaxis)
        y = _as_float64_array(self.yaxis)
        o = _as_float64_array(self.origin)
        object.__setattr__(self, "xaxis", x)
        object.__setattr__(self, "yaxis", y)
        object.__setattr__(self, "origin", o)

        # normalize axis lengths
        # Normalize x in-place
        _normalize_inplace(self.xaxis, name="xaxis")

        # Make y orthogonal to x, then normalize
        proj = float(np.dot(self.yaxis, self.xaxis))
        self.yaxis -= proj * self.xaxis
        _normalize_inplace(self.yaxis, name="yaxis (after orthogonalization)")

        z = np.cross(self.xaxis, self.yaxis)
        object.__setattr__(self, "zaxis", z)

def _as_float64_array(x):
    """Return x as np.ndarray[float64] avoiding copies when possible."""
    if isinstance(x, np.ndarray) and x.dtype == np.float64:
        return x
    return np.asarray(x, dtype=np.float64)

def _normalize_inplace(v: np.ndarray, *, name: str):
    """Normalize vector in-place, raising if zero-length."""
    # Use dot+sqrt for slightly lower overhead vs np.linalg.norm
    n2 = float(np.dot(v, v))
    if n2 == 0.0:
        raise ValueError(f"Plane {name} has zero length")
    inv = 1.0 / math.sqrt(n2)
    v *= inv
    return v
