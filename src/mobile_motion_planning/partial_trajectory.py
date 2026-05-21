"""Calculate trajectory for a subset of nodes along a path.

This module provides functionality to compute IK solutions and find optimal paths
for only a portion of the total trajectory, useful for lookahead planning and
incremental trajectory computation.
"""

import math

# # Add the src directory to the path for imports
# module_path = Path(__file__).resolve().parent.parent.parent
# sys.path.insert(0, str(module_path))
from slab_net_zero.motion_planning.shortest_path_graph_based.graph_based_optimum import (
    PathBuilder,
    )
from slab_net_zero.collision_checking.pybullet.collision_checking_pybullet import (
        CollisionCheck,
    )
from mobile_motion_planning.ik_offline.ik_offline import (
    ik_no_tool_with_base_change,
)
from mobile_motion_planning.ik_offline.geometry import Plane



def _rotate_vector_around_axis(vector, axis, angle_rad):
    """Rotate a vector around an axis using Rodrigues' rotation formula."""
    ux, uy, uz = axis
    vx, vy, vz = vector
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)

    dot = ux * vx + uy * vy + uz * vz
    cross_x = uy * vz - uz * vy
    cross_y = uz * vx - ux * vz
    cross_z = ux * vy - uy * vx

    return (
        vx * c + cross_x * s + ux * dot * (1.0 - c),
        vy * c + cross_y * s + uy * dot * (1.0 - c),
        vz * c + cross_z * s + uz * dot * (1.0 - c),
    )


def _rotation_offsets_degrees(
    rotation_mode,
    rotation_angle_deg,
    rotation_steps,
    angle_cw_deg,
    angle_ccw_deg,
):
    mode = str(rotation_mode).strip().lower()
    if mode in ('false', 'none', 'off', '0'):
        return [0.0]

    if mode == 'n_steps':
        steps = int(rotation_steps)
        if steps < 1:
            raise ValueError('rotation_steps must be >= 1 for rotation_mode=n_steps')
        step_deg = 360.0 / float(steps)
        return [i * step_deg for i in range(steps)]

    if mode == 'step_angle':
        step = float(rotation_angle_deg)
        if step <= 0:
            raise ValueError(
                'rotation_angle_deg must be > 0 for rotation_mode=step_angle'
            )
        start = -float(angle_ccw_deg)
        stop = float(angle_cw_deg)
        offsets = []
        current = start
        # Add a small epsilon to include the upper bound despite float stepping.
        while current <= stop + 1e-9:
            offsets.append(current)
            current += step
        return offsets or [0.0]

    raise ValueError(
        f"Unsupported rotation_mode '{rotation_mode}'. "
        "Use one of: False, n_steps, step_angle"
    )


def _expand_plane_rotations(
    target_plane,
    rotation_mode,
    rotation_angle_deg,
    rotation_steps,
    angle_cw_deg,
    angle_ccw_deg,
):
    offsets_deg = _rotation_offsets_degrees(
        rotation_mode=rotation_mode,
        rotation_angle_deg=rotation_angle_deg,
        rotation_steps=rotation_steps,
        angle_cw_deg=angle_cw_deg,
        angle_ccw_deg=angle_ccw_deg,
    )

    axis = tuple(float(v) for v in target_plane.zaxis)
    xaxis = tuple(float(v) for v in target_plane.xaxis)
    yaxis = tuple(float(v) for v in target_plane.yaxis)
    origin = tuple(float(v) for v in target_plane.origin)

    rotated = []
    for offset_deg in offsets_deg:
        angle_rad = math.radians(offset_deg)
        new_x = _rotate_vector_around_axis(xaxis, axis, angle_rad)
        new_y = _rotate_vector_around_axis(yaxis, axis, angle_rad)
        rotated.append(Plane(origin, new_x, new_y))
    return rotated


def _apply_slab_net_zero_collision_check(ik_solutions_list, collision_data_path=None):
    checker = CollisionCheck.from_solutions(ik_solutions_list)
    if collision_data_path:
        checker.data_path = str(collision_data_path)
    checker.collision_check(write_output=False)
    return checker.collision_free_solutions


def calculate_partial_trajectory(
    current_pose,
    list_of_targets,
    number_of_nodes_to_calculate=None,
    base_planes=None,
    rotation_mode='False',
    rotation_angle_deg=5,
    rotation_steps=35,
    angle_cw_deg=0,
    angle_ccw_deg=0,
    enable_collision_check=False,
    collision_data_path=None,
    path_builder_iterations=10,
):
    """Calculate a partial trajectory for a subset of nodes along a path."""
    num_nodes = min(number_of_nodes_to_calculate, len(list_of_targets))

    targets_subset = list_of_targets[:num_nodes]

    if base_planes is not None:
        if not isinstance(base_planes, list):
            raise TypeError("base_planes must be a list or None")
        if len(base_planes) != len(list_of_targets):
            len_base = len(base_planes)
            len_targets = len(list_of_targets)
            raise ValueError(
                f"len(base_planes) ({len_base}) must match len(list_of_targets) ({len_targets})"
            )
        base_planes_subset = base_planes[:num_nodes]
    else:
        # Use world frame (identity base) for all targets
        world_base = Plane((0, 0, 0), (1, 0, 0), (0, 1, 0))
        base_planes_subset = [world_base] * num_nodes

    # Compute IK solutions for each target
    ik_solutions_list = []
    rotation_candidates_per_node = []
    for point_index, (target_plane, base_plane) in enumerate(
        zip(targets_subset, base_planes_subset),
        start=1,
    ):
        rotated_target_planes = _expand_plane_rotations(
            target_plane,
            rotation_mode=rotation_mode,
            rotation_angle_deg=rotation_angle_deg,
            rotation_steps=rotation_steps,
            angle_cw_deg=angle_cw_deg,
            angle_ccw_deg=angle_ccw_deg,
        )
        rotation_candidates_per_node.append(len(rotated_target_planes))

        ik_solutions = []
        for rotated_target_plane in rotated_target_planes:
            ik_solutions.extend(
                ik_no_tool_with_base_change(rotated_target_plane, base_plane)
            )

        # Handle case where no IK solutions exist for a target
        if not ik_solutions or len(ik_solutions) == 0:
            # Add empty list to maintain index correspondence
            ik_solutions_list.append([])
        else:
            ik_solutions_list.append(ik_solutions)

        print(
            f"Point {point_index}/{num_nodes}: found "
            f"{len(ik_solutions_list[-1])} IK solutions"
        )

    if enable_collision_check:
        try:
            ik_solutions_list = _apply_slab_net_zero_collision_check(
                ik_solutions_list,
                collision_data_path=collision_data_path,
            )
            for point_index, ik_solutions in enumerate(ik_solutions_list, start=1):
                print(
                    f"Point {point_index}/{num_nodes}: "
                    f"{len(ik_solutions)} IK solutions after collision culling"
                )
        except Exception as e:
            print(f'Collision culling failed: {e}')

    solution_counts = [len(solutions) for solutions in ik_solutions_list]
    reachable_points = sum(1 for count in solution_counts if count > 0)
    total_solutions = sum(solution_counts)
    print(f"Reachable points: {reachable_points}/{num_nodes}")
    print(f"Total IK solutions across all points: {total_solutions}")

    empty_point_indices = [
        idx for idx, count in enumerate(solution_counts, start=1) if count == 0
    ]
    if empty_point_indices:
        print(
            "Skipping graph construction: no IK solutions for points "
            f"{empty_point_indices}"
        )
        return {
            "configurations": [],
            "path_length": float("inf"),
            "num_nodes_computed": num_nodes,
            "ik_solutions_per_node": ik_solutions_list,
            "rotation_candidates_per_node": rotation_candidates_per_node,
            "unreachable_points": empty_point_indices,
        }

    # Check if we have any valid solutions
    if all(len(sols) == 0 for sols in ik_solutions_list):
        return {
            "configurations": [],
            "path_length": float("inf"),
            "num_nodes_computed": num_nodes,
            "ik_solutions_per_node": ik_solutions_list,
            "rotation_candidates_per_node": rotation_candidates_per_node,
        }

    ik_solutions_with_start = [[current_pose]] + ik_solutions_list
    # print(ik_solutions_with_start)

    # Use PathBuilder to find optimal path through the solutions
    try:
        path_builder = PathBuilder.from_solutions(
            solutions=ik_solutions_with_start, lift=0, logging=False
        )

        # Find shortest path (using limited iterations for speed)
        shortest_path, path_length = path_builder.find_shortest_path(
            iterations=path_builder_iterations
        )

        if shortest_path is None:
            return {
                "configurations": [],
                "path_length": float("inf"),
                "num_nodes_computed": num_nodes,
                "ik_solutions_per_node": ik_solutions_list,
                "rotation_candidates_per_node": rotation_candidates_per_node,
            }

        # Extract configurations from the path
        configurations = []
        for node in shortest_path:
            config = path_builder.graph.nodes[node]["configuration"]
            configurations.append(config)

        # Remove the prepended current pose if it was added
        if len(ik_solutions_list[0]) > 0 and len(configurations) > num_nodes:
            configurations = configurations[1:]  # Skip the first (current pose)

        return {
            "configurations": configurations,
            "path_length": path_length,
            "num_nodes_computed": num_nodes,
            "ik_solutions_per_node": ik_solutions_list,
            "rotation_candidates_per_node": rotation_candidates_per_node,
        }

    except Exception as e:
        # If path building fails, return empty result with error info
        print(f"Path building failed: {e}")
        return {
            "configurations": [],
            "path_length": float("inf"),
            "num_nodes_computed": num_nodes,
            "ik_solutions_per_node": ik_solutions_list,
            "rotation_candidates_per_node": rotation_candidates_per_node,
            "error": str(e),
        }


# def select_buffer_tail_pose(
#     *,
#     exec_index,
#     replan_start_index,
#     replanned_configurations,
#     buffer_size,
# ):
#     """Return the global index/configuration that should replace the buffer tail.

#     Example with ``buffer_size=2``:
#     - UR reports ``exec_index=0``
#     - Replanning starts at global target index 1 (targets 1 and 2)
#     - Target 1 is already on robot, so only replanned target 2 is sent
#     """
#     if buffer_size < 1:
#         raise ValueError("buffer_size must be >= 1")

#     if not replanned_configurations:
#         return None

#     tail_global_index = exec_index + buffer_size
#     tail_offset = tail_global_index - replan_start_index
#     if tail_offset < 0 or tail_offset >= len(replanned_configurations):
#         return None

#     return tail_global_index, replanned_configurations[tail_offset]


if __name__ == "__main__":
    # Example usage
    # Define current robot pose (joint angles in radians)
    current_pose = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

    # Define target planes
    target_planes = [
        Plane((1.0, 0.5, 0.5), (1, 0, 0), (0, 1, 0)),
        Plane((1.0, 0.6, 0.6), (1, 0, 0), (0, 1, 0)),
        Plane((1.0, 0.7, 0.7), (1, 0, 0), (0, 1, 0)),
        Plane((1.0, 0.8, 0.8), (1, 0, 0), (0, 1, 0)),
    ]

    # Calculate trajectory for first 3 nodes
    result = calculate_partial_trajectory(
        number_of_nodes_to_calculate=3,
        current_pose=current_pose,
        list_of_targets=target_planes,
    )

    # print(f"Computed {result['num_nodes_computed']} nodes")
    # print(f"Path length: {result['path_length']}")
    # print(f"Number of configurations in path: {len(result['configurations'])}")
    # if result["configurations"]:
    #     print("\nFirst configuration:")
    #     print(result["configurations"][0])
