"""Calculate trajectory for a subset of nodes along a path.

This module provides functionality to compute IK solutions and find optimal paths
for only a portion of the total trajectory, useful for lookahead planning and
incremental trajectory computation.
"""

import sys
from pathlib import Path

# # Add the src directory to the path for imports
# module_path = Path(__file__).resolve().parent.parent.parent
# sys.path.insert(0, str(module_path))

from mobile_motion_planning.ik_offline.ik_offline import (
    ik_no_tool_with_base_change,
    find_matching_ik_solution,
)
from mobile_motion_planning.ik_offline.geometry import Plane
from slab_net_zero.motion_planning.shortest_path_graph_based.graph_based_optimum import (
    PathBuilder,
)


def calculate_partial_trajectory(
    current_pose, list_of_targets, number_of_nodes_to_calculate=10, base_planes=None
):
    num_nodes = min(number_of_nodes_to_calculate, len(list_of_targets))

    targets_subset = list_of_targets[:num_nodes]

    if base_planes is not None:
        if not isinstance(base_planes, list):
            raise TypeError("base_planes must be a list or None")
        if len(base_planes) != len(list_of_targets):
            raise ValueError(
                f"base_planes length ({len(base_planes)}) must match list_of_targets length ({len(list_of_targets)})"
            )
        base_planes_subset = base_planes[:num_nodes]
    else:
        # Use world frame (identity base) for all targets
        world_base = Plane((0, 0, 0), (1, 0, 0), (0, 1, 0))
        base_planes_subset = [world_base] * num_nodes

    # Compute IK solutions for each target
    ik_solutions_list = []
    for target_plane, base_plane in zip(targets_subset, base_planes_subset):
        ik_solutions = ik_no_tool_with_base_change(target_plane, base_plane)
        #we are using the flange frames that where precomputed from target planes

        # Handle case where no IK solutions exist for a target
        if not ik_solutions or len(ik_solutions) == 0:
            # Add empty list to maintain index correspondence
            ik_solutions_list.append([])
        else:
            ik_solutions_list.append(ik_solutions)

    # Check if we have any valid solutions
    if all(len(sols) == 0 for sols in ik_solutions_list):
        return {
            "configurations": [],
            "path_length": float("inf"),
            "num_nodes_computed": num_nodes,
            "ik_solutions_per_node": ik_solutions_list,
        }

    ik_solutions_with_start = [[current_pose]] + ik_solutions_list
    # print(ik_solutions_with_start)

    # Use PathBuilder to find optimal path through the solutions
    try:
        path_builder = PathBuilder.from_solutions(
            solutions=ik_solutions_with_start, lift=0, logging=False
        )

        # Find shortest path (using limited iterations for speed)
        shortest_path, path_length = path_builder.find_shortest_path(iterations=10)

        if shortest_path is None:
            return {
                "configurations": [],
                "path_length": float("inf"),
                "num_nodes_computed": num_nodes,
                "ik_solutions_per_node": ik_solutions_list,
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
        }

    except Exception as e:
        # If path building fails, return empty result with error info
        print(f"Path building failed: {e}")
        return {
            "configurations": [],
            "path_length": float("inf"),
            "num_nodes_computed": num_nodes,
            "ik_solutions_per_node": ik_solutions_list,
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
    from mobile_motion_planning.ik_offline.geometry import Plane

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
