"""Export a list of Grasshopper planes to a JSON file compatible with mobile_motion_planning.

Grasshopper inputs:
    planes      - list of Rhino/Grasshopper Plane objects (required)
    filepath    - folder path where the JSON will be written (required)
    curvature   - numeric value used in filename (required)
    base_offset_x - numeric value used in filename (required)
    base_offset_y - numeric value used in filename (required)
    base_length - numeric value used in filename (required)
    type        - string value used in filename (required)
    run         - bool toggle to trigger export (required)
"""
import json
import os
import math
from datetime import datetime


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


def _format_token(value):
    """Format filename token values into compact, filesystem-safe strings."""
    numeric_value = None
    try:
        if isinstance(value, bool):
            raise ValueError
        numeric_value = float(value)
    except (TypeError, ValueError):
        numeric_value = None

    if numeric_value is not None:
        rounded = round(numeric_value, 2)
        if abs(rounded) >= 10:
            raise ValueError(
                "Numeric filename fields must be in range [-9.99, 9.99] to keep fixed width"
            )

        sign = "m" if rounded < 0 else "p"
        text = "{}{:0.2f}".format(sign, abs(rounded))
        return text.replace(".", "_")

    # Non-numeric tokens: keep '-' and make spaces/decimal points filesystem-friendly.
    text = str(value).strip()
    return text.replace(".", "_").replace(" ", "-")


def _build_filename(curvature, base_offset_x, base_offset_y, base_length, type_name):
    date_part = datetime.now().strftime("%y%m%d")
    c = _format_token(curvature)
    y = _format_token(base_offset_y)
    x = _format_token(base_offset_x)
    l = _format_token(base_length)
    t = _format_token(type_name).upper()
    return "{}-C{}-Y{}-X{}-L-base{}-{}.json".format(date_part, c, y, x, l, t)


def planes_to_json(planes, filepath, curvature, base_offset_x, base_offset_y, base_length, type_name):
    """Convert a list of Grasshopper planes to mobile_motion_planning JSON format.

    Parameters
    ----------
    planes : list of Rhino.Geometry.Plane
        Planes from Grasshopper.
    filepath : str
        Output folder path, e.g. r"C:\\data".
    curvature : int | float | str
        Curvature value used in the output filename.
    base_offset_x : int | float | str
        Base offset X value used in the output filename.
    base_offset_y : int | float | str
        Base offset Y value used in the output filename.
    base_length : int | float | str
        Base length value used in the output filename.
    type_name : str
        Type label used in the output filename.

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

    output_dir = os.path.abspath(filepath)
    os.makedirs(output_dir, exist_ok=True)

    filename = _build_filename(curvature, base_offset_x, base_offset_y, base_length, type_name)
    output_path = os.path.join(output_dir, filename)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    return output_path



_planes = globals().get("planes")
_filepath = globals().get("filepath")
_curvature = globals().get("curvature")
_base_offset_x = globals().get("base_offset_x")
_base_offset_y = globals().get("base_offset_y")
_base_length = globals().get("base_length")
_type_name = globals().get("type_name", globals().get("type"))
_run = bool(globals().get("run", True))

if _run and _planes is not None and _filepath:
    out = planes_to_json(
        _planes,
        _filepath,
        _curvature,
        _base_offset_x,
        _base_offset_y,
        _base_length,
        _type_name,
    )
    print("Exported {} planes to: {}".format(len(_planes), out))
