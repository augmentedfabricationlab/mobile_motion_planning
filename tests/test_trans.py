import unittest
import json
import numpy as np
import sys
from pathlib import Path

module_path = Path(__file__).resolve().parent.parent / 'src'
sys.path.insert(0, str(module_path))

from mobile_motion_planning.ik_offline.trans import (
    matrix_inverse,
    world_to_plane,
    plane_to_world,
    from_plane_to_plane,
    apply_T_to_plane,
)
from mobile_motion_planning.ik_offline.trans import Plane
from mobile_motion_planning.ik_offline.ik_offline import ik_with_tool, find_matching_ik_solution
from compas.geometry import Frame, Point, Vector, Transformation
import time

test_plane_no_rot = Plane(
    origin=(0.5, 0.0, 0.5), xaxis=(1.0, 0.0, 0.0), yaxis=(0.0, 1.0, 0.0)
)

test_plane_rot = Plane(
    origin=(0.5, 0.0, 0.5), xaxis=(0.0, 0.0, 1.0), yaxis=(0.0, 1.0, 0.0)
)

world_xy_plane = Plane(
    origin=(0.0, 0.0, 0.0), xaxis=(1.0, 0.0, 0.0), yaxis=(0.0, 1.0, 0.0)
)

# Frame(point=Point(x=-0.731, y=0.695, z=0.528), xaxis=Vector(x=0.248, y=-0.723, z=-0.645), yaxis=Vector(x=0.616, y=0.632, z=-0.471))
plane_from = Plane(
    origin=(-0.731, 0.695, 0.528),
    xaxis=(0.248, -0.723, -0.645),
    yaxis=(0.616, 0.632, -0.471),
)

# Frame(point=Point(x=0.912, y=0.896, z=-0.887), xaxis=Vector(x=-0.784, y=-0.101, z=0.612), yaxis=Vector(x=0.322, y=-0.909, z=0.263))

plane_to = Plane(
    origin=(0.912, 0.896, -0.887),
    xaxis=(-0.784, -0.101, 0.612),
    yaxis=(0.322, -0.909, 0.263),
)


class TestTransformation(unittest.TestCase):
    def test_matrix_inverse_matches_numpy_inv(self):
        # Use a well-conditioned 3x3 matrix to avoid singularities
        mat = np.array([[1.0, 2.0, 3.0], [3.0, 2.0, 1.0], [2.0, 1.0, 3.0]])
        expected = np.linalg.inv(mat)
        result = matrix_inverse(mat)
        assert np.allclose(result, expected)
        assert np.allclose(mat @ result, np.eye(3))

    def test_matrix_inverse_specific_values(self):
        mat = np.array([[1.0, 2.0, 3.0], [3.0, 2.0, 1.0], [2.0, 1.0, 3.0]])
        expected = np.array(
            [
                [-0.41666667, 0.25, 0.33333333],
                [0.58333333, 0.25, -0.66666667],
                [0.08333333, -0.25, 0.33333333],
            ]
        )
        result = matrix_inverse(mat)
        assert np.allclose(result, expected)

    def test_world_to_plane_transformation(self):

        test_plane = test_plane_no_rot

        expected = np.array(
            [
                [1.0, 0.0, 0.0, 0.5],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.5],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        result = world_to_plane(test_plane)
        assert np.allclose(result, expected)

    def test_plane_to_world_transformation(self):

        test_plane = test_plane_no_rot

        expected = np.array(
            [
                [1.0, 0.0, 0.0, -0.5],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, -0.5],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        result = matrix_inverse(world_to_plane(test_plane))
        result2 = plane_to_world(test_plane)
        assert np.allclose(result, expected)
        assert np.allclose(result2, expected)

    def test_plane_to_world_rotated_plane(self):
        test_plane = test_plane_rot

        expected = np.array(
            [
                [0.0, 0.0, 1.0, -0.5],
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.5],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        result = plane_to_world(test_plane)
        assert np.allclose(result, expected)

    def test_apply_T_to_plane(self):
        T = from_plane_to_plane(world_xy_plane, test_plane_no_rot)
        transformed_plane = apply_T_to_plane(T, world_xy_plane)
        assert np.allclose(transformed_plane.origin, test_plane_no_rot.origin)
        assert np.allclose(transformed_plane.xaxis, test_plane_no_rot.xaxis)
        assert np.allclose(transformed_plane.yaxis, test_plane_no_rot.yaxis)

        T1 = from_plane_to_plane(world_xy_plane, test_plane_rot)
        transformed_plane_rot = apply_T_to_plane(T1, world_xy_plane)
        assert np.allclose(transformed_plane_rot.origin, test_plane_rot.origin)
        assert np.allclose(transformed_plane_rot.xaxis, test_plane_rot.xaxis)
        assert np.allclose(transformed_plane_rot.yaxis, test_plane_rot.yaxis)

        T2 = from_plane_to_plane(test_plane_no_rot, test_plane_rot)
        transformed_plane_rot_2 = apply_T_to_plane(T2, test_plane_no_rot)
        assert np.allclose(transformed_plane_rot_2.origin, test_plane_rot.origin)
        assert np.allclose(transformed_plane_rot_2.xaxis, test_plane_rot.xaxis)
        assert np.allclose(transformed_plane_rot_2.yaxis, test_plane_rot.yaxis)

    def create_random_plane_to_random_plane(self):
        np.random.seed(42)
        for _ in range(1):
            origin_from = tuple(np.random.uniform(-1, 1, size=3))
            xaxis_from = tuple(np.random.uniform(-1, 1, size=3))
            yaxis_from = tuple(np.random.uniform(-1, 1, size=3))
            plane_from = Plane(origin=origin_from, xaxis=xaxis_from, yaxis=yaxis_from)
            print(plane_from)

            origin_to = tuple(np.random.uniform(-1, 1, size=3))
            xaxis_to = tuple(np.random.uniform(-1, 1, size=3))
            yaxis_to = tuple(np.random.uniform(-1, 1, size=3))
            plane_to = Plane(origin=origin_to, xaxis=xaxis_to, yaxis=yaxis_to)
            print(plane_to)

    #             t_from = plane_to_world(plane_from)
    # t_to = world_to_plane(plane_to)
    # T = t_to @ t_from

    def test_plane_to_plane_transformation(self):

        T = from_plane_to_plane(plane_from, plane_to)
        transformed_plane = apply_T_to_plane(T, plane_from)
        # T =
        # [[    0.4003,    0.6217,    0.6733,    0.4176],
        #  [   -0.2837,   -0.6145,    0.7361,    0.7269],
        #  [    0.8714,   -0.4857,   -0.0696,    0.1245],
        #  [    0.0000,    0.0000,    0.0000,    1.0000]]
        print("Computed Transformation T:")
        print(T)
        expected_t = np.array(
            [
                [0.4003, 0.6217, 0.6733, 0.4176],
                [-0.2837, -0.6145, 0.7361, 0.7269],
                [0.8714, -0.4857, -0.0696, 0.1245],
                [0.0000, 0.0000, 0.0000, 1.0000],
            ]
        )
        assert np.allclose(T, expected_t, atol=1e-3)

        assert np.allclose(transformed_plane.origin, (0.912, 0.896, -0.887), atol=1e-3)
        assert np.allclose(transformed_plane.xaxis, (-0.784, -0.101, 0.612), atol=1e-3)
        assert np.allclose(transformed_plane.yaxis, (0.322, -0.909, 0.263), atol=1e-3)

    def test_ik_with_tool(self):
        # [2.121796, -0.404935, 2.300289, 1.647714, 1.056038, -0.206041]
        # [2.121796, 1.525000, -2.300289, -1.964830, 1.056038, -0.206041]
        # [2.121796, -0.870919, 2.263498, -0.991105, -1.056038, 2.935551]
        # [2.121796, 1.039412, -2.263498, 1.625561, -1.056038, 2.935551]
        # [-0.394834, 2.086848, 2.192272, 1.249410, 2.622006, 2.457034]
        # [-0.394834, -2.327387, -2.192272, -2.518181, 2.622006, 2.457034]
        # [-0.394834, 1.599835, 2.378749, -1.591647, -2.622006, -0.684559]
        # [-0.394834, -2.716563, -2.378749, 1.199064, -2.622006, -0.684559]
        expected_solutions = [
            [2.121796, -0.404935, 2.300289, 1.647714, 1.056038, -0.206041],
            [2.121796, 1.525000, -2.300289, -1.964830, 1.056038, -0.206041],
            [2.121796, -0.870919, 2.263498, -0.991105, -1.056038, 2.935551],
            [2.121796, 1.039412, -2.263498, 1.625561, -1.056038, 2.935551],
            [-0.394834, 2.086848, 2.192272, 1.249410, 2.622006, 2.457034],
            [-0.394834, -2.327387, -2.192272, -2.518181, 2.622006, 2.457034],
            [-0.394834, 1.599835, 2.378749, -1.591647, -2.622006, -0.684559],
            [-0.394834, -2.716563, -2.378749, 1.199064, -2.622006, -0.684559],
        ]
        target_plane = Plane(
            origin=(0.5, 0.0, 0.5),
            xaxis=(1.0, 0.0, 0.0),
            yaxis=(0.0, 1.0, 0.0),
        )
        ik_solutions = ik_with_tool(target_plane)
        self.assertEqual(len(ik_solutions), len(expected_solutions))
        for sol, expected in zip(ik_solutions, expected_solutions):
            self.assertTrue(
                np.allclose(sol, expected, atol=1e-5),
                f"Expected: {expected}, Got: {sol}",
            )

    def test_transformation_speed_comparison(self):
        """Compare transformation speed between our library and COMPAS."""
        # Setup test data
        num_iterations = 1000
                
        # COMPAS Frames
        frame_from = Frame(
            point=Point(*plane_from.origin),
            xaxis=Vector(*plane_from.xaxis),
            yaxis=Vector(*plane_from.yaxis),
        )
        frame_to = Frame(
            point=Point(*plane_to.origin),
            xaxis=Vector(*plane_to.xaxis),
            yaxis=Vector(*plane_to.yaxis),
        )

        # Our library transformation
        start = time.perf_counter()
        for _ in range(num_iterations):
            T = from_plane_to_plane(plane_from, plane_to)
            apply_T_to_plane(T, plane_from)
        our_time = time.perf_counter() - start

        # COMPAS transformation
        start = time.perf_counter()
        for _ in range(num_iterations):
            # T_compas = Transformation.from_frame(frame_from).inverse() * Transformation.from_frame(frame_to)
            T_compas = Transformation.from_frame_to_frame(frame_from, frame_to)
            frame_from.transformed(T_compas)
        compas_time = time.perf_counter() - start
        
        # Calculate percentage (COMPAS as reference at 100%)
        our_speed_percent = (our_time / compas_time) *100
        
        print(f"Our library: {our_time:.4f}s")
        print(f"COMPAS: {compas_time:.4f}s")
        print(f"Our speed: {our_speed_percent:.2f}% (COMPAS = 100%)")
        print(f"Speed ratio (COMPAS / Our library): {compas_time / our_time:.2f}x")
        
        # Assert that our implementation is reasonably performant
        print(T)
        print(T_compas.matrix)
        self.assertLess(our_speed_percent, 50, "Our library should be at least 50% as fast as COMPAS")

    def test_find_matching_ik_solution_basic(self):
        """Test find_matching_ik_solution with basic cases."""
        # Test with empty solutions list
        result = find_matching_ik_solution([0, 0, 0, 0, 0, 0], [])
        self.assertIsNone(result)
        
        # Test with single solution
        current_pose = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        solutions = [[1.5, 2.5, 3.5, 4.5, 5.5, 6.5]]
        result = find_matching_ik_solution(current_pose, solutions)
        self.assertEqual(result, solutions[0])
        
        # Test with multiple solutions - should select the closest one
        current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        solutions = [
            [5.0, 5.0, 5.0, 5.0, 5.0, 5.0],  # farther from current_pose
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # closest to current_pose
            [3.0, 3.0, 3.0, 3.0, 3.0, 3.0],  # medium distance from current_pose
        ]
        result = find_matching_ik_solution(current_pose, solutions)
        self.assertEqual(result, solutions[1])

    def test_find_matching_ik_solution_with_real_ik(self):
        """Test find_matching_ik_solution with real IK solutions."""
        # Get real IK solutions for a target plane
        target_plane = Plane(
            origin=(0.5, 0.0, 0.5),
            xaxis=(1.0, 0.0, 0.0),
            yaxis=(0.0, 1.0, 0.0),
        )
        ik_solutions = ik_with_tool(target_plane)
        
        # Verify we have solutions
        self.assertGreater(len(ik_solutions), 0)
        
        # Use one of the solutions as the current pose and verify we get it back
        current_pose = ik_solutions[0]
        result = find_matching_ik_solution(current_pose, ik_solutions)
        
        # Should get the exact same solution (or very close)
        self.assertTrue(np.allclose(result, current_pose))
        
        # Test with a slight variation of a solution
        # Should still select the closest one
        # Ensure we have at least 3 solutions before testing
        if len(ik_solutions) > 2:
            slightly_modified = [angle + 0.01 for angle in ik_solutions[2]]
            result = find_matching_ik_solution(slightly_modified, ik_solutions)
            
            # The result should be closest to ik_solutions[2]
            num_joints = len(slightly_modified)
            distance_to_result = sum((result[j] - slightly_modified[j])**2 for j in range(num_joints))
            for i, sol in enumerate(ik_solutions):
                if sol != result:
                    distance_to_other = sum((sol[j] - slightly_modified[j])**2 for j in range(num_joints))
                    self.assertLessEqual(distance_to_result, distance_to_other)

if __name__ == "__main__":
    unittest.main()
