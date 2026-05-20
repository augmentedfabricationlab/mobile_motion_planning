"""Export a list of Grasshopper planes to a JSON file compatible with mobile_motion_planning.

Grasshopper inputs:
    planes      - list of Rhino/Grasshopper Plane objects (required)
    filepath    - full path including filename, e.g. r"C:\\data\\planes.json" (required)
    run         - bool toggle to trigger export (required)
"""
import json
import os
import math


def _normalize(v):
    n = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if n == 0:
        raise ValueError("Zero-length axis vector encountered")
    return [v[0]/n, v[1]/n, v[2]/n]


def _cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]


def _as_plane_components(item):
    """Extract origin/x/y axes from Plane-like or Point3d-like objects.

    Point inputs are exported with world XY orientation.
    """
    is_plane_like = hasattr(item, "Origin") and hasattr(item, "XAxis") and hasattr(item, "YAxis")
    if is_plane_like:
        origin = [item.Origin.X, item.Origin.Y, item.Origin.Z]
        x_axis = [item.XAxis.X, item.XAxis.Y, item.XAxis.Z]
        y_axis = [item.YAxis.X, item.YAxis.Y, item.YAxis.Z]
        return origin, x_axis, y_axis

    is_point_like = hasattr(item, "X") and hasattr(item, "Y") and hasattr(item, "Z")
    if is_point_like:
        origin = [item.X, item.Y, item.Z]
        return origin, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]

    raise TypeError(
        "Expected Plane or Point3d-like input with X/Y/Z (got: {})".format(type(item).__name__)
    )


def planes_to_json(planes, filepath):
    """Convert a list of Grasshopper planes to mobile_motion_planning JSON format.

    Parameters
    ----------
    planes : list of Rhino.Geometry.Plane
        Planes from Grasshopper.
    filepath : str
        Full output path, e.g. r"C:\\data\\segment_planes.json".

    Returns
    -------
    str
        The filepath that was written.
    """
    data = []
    for plane in planes:
        origin, x_axis, y_axis = _as_plane_components(plane)
        x_axis = _normalize(x_axis)
        y_axis = _normalize(y_axis)
        z_axis = _cross(x_axis, y_axis)

        data.append({
            "origin": origin,
            "x_axis": x_axis,
            "y_axis": y_axis,
            "z_axis": z_axis,
        })

    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    return filepath



out = planes_to_json(planes, filepath)
print("Exported {} planes to: {}".format(len(planes), out))
