"""Compute flange pose and print inverse kinematics solutions without robot connection."""
# for example usage see testing file or notebook
from .geometry import Plane
from .ik import inverse_kinematics
from .trans import (
    from_plane_to_plane,
    apply_T_to_plane,
    plane_to_world,
    world_to_plane,
)


def print_matrix(mat):
    """Print any matrix nicely formatted."""
    for row in mat:
        print(" ".join(f"{val:10.6f}" for val in row))


def print_plane(p: Plane, label: str = "Plane"):
    print(f"{label}:")
    print(f"  origin: {tuple(round(x, 6) for x in p.origin)}")
    print(f"  xaxis : {tuple(round(x, 6) for x in p.xaxis)}")
    print(f"  yaxis : {tuple(round(x, 6) for x in p.yaxis)}")
    if getattr(p, "zaxis", None) is not None:
        print(f"  zaxis : {tuple(round(x, 6) for x in p.zaxis)}")


def print_transformation(transformation, rotation=None, translation=None):
    """Print rotation, translation, and homogeneous transformation."""
    if rotation is None or translation is None:
        rotation = [
            [transformation[0][0], transformation[0][1], transformation[0][2]],
            [transformation[1][0], transformation[1][1], transformation[1][2]],
            [transformation[2][0], transformation[2][1], transformation[2][2]],
        ]
        translation = [
            transformation[0][3],
            transformation[1][3],
            transformation[2][3],
        ]
    print("Rotation Matrix R:")
    print_matrix(rotation)
    print("\nTranslation Vector t:")
    print(" ".join(f"{val:10.6f}" for val in translation))
    print("\nHomogeneous Transformation Matrix T:")
    print_matrix(transformation)

def ik_with_tool_and_base_change(target: Plane, new_base: Plane):
    T=from_plane_to_plane(new_base,Plane((0,0,0),(1,0,0),(0,1,0)))
    target_in_new_base=apply_T_to_plane(T,target)
    return ik_with_tool(target_in_new_base)

def ik_no_tool_with_base_change(target: Plane, new_base: Plane):
    T=from_plane_to_plane(new_base,Plane((0,0,0),(1,0,0),(0,1,0)))
    target_in_new_base=apply_T_to_plane(T,target)
    # print_plane(target_in_new_base, "Target in New Base")
    return inverse_kinematics(target_in_new_base)


def find_matching_ik_solution(current_pose, ik_solutions):
    """Find the IK solution closest to the current pose.
    
    Args:
        current_pose: List of joint angles (in radians) representing the current robot configuration
        ik_solutions: List of possible IK solutions, each being a list of joint angles
        
    Returns:
        The IK solution (list of joint angles) that is closest to current_pose,
        or None if ik_solutions is empty
        
    Note:
        The function uses the minimum length between current_pose and each solution
        to handle potential length mismatches gracefully. For UR robots, this is
        typically 6 joint angles.
    """
    if not ik_solutions:
        return None
    
    # Calculate distance for each solution using sum of squared differences
    min_distance = float('inf')
    best_solution = None
    
    for solution in ik_solutions:
        # Calculate sum of squared differences between current pose and this solution
        # Use min length to handle potential mismatches gracefully
        # Note: If lengths differ, only the overlapping joints are compared.
        # This is designed for UR robots where all solutions have 6 joints.
        min_len = min(len(current_pose), len(solution))
        distance = sum((current_pose[j] - solution[j])**2 for j in range(min_len))
        
        if distance < min_distance:
            min_distance = distance
            best_solution = solution
    
    return best_solution

def ik_with_tool(target: Plane):
    """Compute flange pose and return inverse kinematics solutions without robot connection."""

    # tcp_in_world = Plane((0.00302, -0.29571, 0.40445), (0.0366, 0, 0), (0, 0.0051, 0.0141)) # tool 7 abele split 90deg adapter straight inlet
    tcp_in_world = Plane((0.000,-0.29786,0.361), (0.0151,0,0), (0,-0.014,0.0056)) # tool 8 abele split 90deg adapter straight inlet

    t = from_plane_to_plane(tcp_in_world, target)
    target_flange = apply_T_to_plane(t)
    solutions = inverse_kinematics(target_flange)
    return solutions




def generate_and_save_ik_solutions(
    flange_json='__251117_163017_flange_frames.json',
    base_json='__251117_163017_base_frames.json',
    output_json='ik_solutions.json',
):
    import json
    from pathlib import Path

    script_dir = Path(__file__).resolve().parent
    export_dir = (script_dir / '../../../../data/auto_generated/export').resolve()

    json_path = export_dir / flange_json
    with json_path.open('r', encoding='utf-8') as f:
        flange_planes_data = json.load(f)
    flange_planes = []
    for plane_data in flange_planes_data:
        plane = Plane(
            origin=tuple(plane_data['origin']),
            xaxis=tuple(plane_data['x_axis']),
            yaxis=tuple(plane_data['y_axis']),
        )
        flange_planes.append(plane)
    print(f"Loaded {len(flange_planes_data)} flange planes from {json_path}")

    base_json_path = export_dir / base_json
    with base_json_path.open('r', encoding='utf-8') as f:
        base_planes_data = json.load(f)
    base_planes = []
    for plane_data in base_planes_data:
        plane = Plane(
            origin=tuple(plane_data['origin']),
            xaxis=tuple(plane_data['x_axis']),
            yaxis=tuple(plane_data['y_axis']),
        )
        base_planes.append(plane)
    print(f"Loaded {len(base_planes_data)} base planes from {base_json_path}")

    ik_solutions_list = []
    for flange_plane, base_plane in zip(flange_planes, base_planes):
        ik_solutions = ik_no_tool_with_base_change(flange_plane, base_plane)
        ik_solutions_list.append(ik_solutions)

    print(f"Computed IK solutions for {len(ik_solutions_list)} flange planes.")

    output_path = export_dir / output_json
    print(f"Saving IK solutions to {output_path}")
    with output_path.open('w', encoding='utf-8') as f:
        json.dump([[[angle for angle in sol] for sol in ik_solutions] for ik_solutions in ik_solutions_list], f, indent=4)


if __name__ == '__main__':
    generate_and_save_ik_solutions()

# target_plane = Plane(
#     origin=[
#       1.0319685741916709,
#       0.6092993007114906,
#       3.093417208269365
#     ],
#     xaxis=[
#       0.7025211723906232,
#       -0.5032216223210783,
#       0.5032216223210668
#     ],
#     yaxis=[
#       -0.24206221093348979,
#       0.49598126164004036,
#       0.8339115505495439
#     ])
# base_plane = Plane(
#     origin=[
#       1.5319685741916709,
#       0.6092993007114906,
#       2
#     ],
#     xaxis=[
#       1.0,
#       0.0,
#       0.0
#     ],
#     yaxis=[
#       0.0,
#       1.0,
#       0.0
#     ]
# )

# ik_solutions = ik_no_tool_with_base_change(target_plane, base_plane)
# print("\nInverse Kinematics Solutions:")
# for sol in ik_solutions:
#     print(f"[{', '.join(f'{angle:.6f}' for angle in sol)}]")
          


# ik_solutions = inverse_kinematics(target_plane)
# ik_solutions = ik_with_tool(target_plane)
# print("\nInverse Kinematics Solutions:")
# for sol in ik_solutions:
#     print(f"[{', '.join(f'{angle:.6f}' for angle in sol)}]")
