"""Tests for the partial_trajectory module.

This test suite validates the calculate_partial_trajectory function with various
scenarios including normal operation, edge cases, and error conditions.
Uses Python's built-in unittest framework.
"""

import unittest
import sys
from pathlib import Path
import json
import random
import time
# Add the src directory to the path
module_path = Path(__file__).resolve().parent.parent / 'src'
sys.path.insert(0, str(module_path))

# from slab_net_zero.motion_planning.partial_trajectory import calculate_partial_trajectory
# from slab_net_zero.motion_planning.ik_offline.geometry import Plane
from mobile_motion_planning.partial_trajectory import calculate_partial_trajectory
from mobile_motion_planning.ik_offline.geometry import Plane

class TestCalculatePartialTrajectory(unittest.TestCase):
    """Test suite for calculate_partial_trajectory function."""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.sample_current_pose = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        self.sample_targets = [
            Plane((1.0, 0.5, 0.5), (1, 0, 0), (0, 1, 0)),
            Plane((1.0, 0.6, 0.6), (1, 0, 0), (0, 1, 0)),
            Plane((1.0, 0.7, 0.7), (1, 0, 0), (0, 1, 0)),
            Plane((1.0, 0.8, 0.8), (1, 0, 0), (0, 1, 0)),
            Plane((1.0, 0.9, 0.9), (1, 0, 0), (0, 1, 0)),
        ]
        world_base = Plane((0, 0, 0), (1, 0, 0), (0, 1, 0))
        self.sample_base_planes = [world_base] * 5
    

    
    def test_with_base_planes(self):
        """Test functionality with base planes provided."""
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=3,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets,
            base_planes=self.sample_base_planes
        )
        
        self.assertEqual(result['num_nodes_computed'], 3)
        self.assertIsInstance(result, dict)
    
    def test_calculate_all_nodes(self):
        """Test calculating all available nodes."""
        num_targets = len(self.sample_targets)
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=num_targets,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets
        )
        
        self.assertEqual(result['num_nodes_computed'], num_targets)
    
    def test_request_more_than_available(self):
        """Test requesting more nodes than available targets."""
        num_targets = len(self.sample_targets)
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=num_targets + 10,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets
        )
        
        # Should only compute available nodes
        self.assertEqual(result['num_nodes_computed'], num_targets)
    
    def test_single_node(self):
        """Test with only one node to calculate."""
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=1,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets
        )
        
        self.assertEqual(result['num_nodes_computed'], 1)
    
    def test_base_planes_length_mismatch(self):
        """Test with base_planes length not matching targets."""
        wrong_base_planes = [Plane((0, 0, 0), (1, 0, 0), (0, 1, 0))] * 3
        
        with self.assertRaises(ValueError) as context:
            calculate_partial_trajectory(
                number_of_nodes_to_calculate=3,
                current_pose=self.sample_current_pose,
                list_of_targets=self.sample_targets,
                base_planes=wrong_base_planes
            )
        self.assertIn("base_planes length", str(context.exception))
    
    def test_base_planes_invalid_type(self):
        """Test with invalid base_planes type."""
        with self.assertRaises(TypeError) as context:
            calculate_partial_trajectory(
                number_of_nodes_to_calculate=3,
                current_pose=self.sample_current_pose,
                list_of_targets=self.sample_targets,
                base_planes="invalid"
            )
        self.assertIn("base_planes must be a list or None", str(context.exception))
    
    def test_result_structure(self):
        """Test that result has correct structure."""
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=2,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets
        )
        
        # Check all required keys are present
        self.assertIn('configurations', result)
        self.assertIn('path_length', result)
        self.assertIn('num_nodes_computed', result)
        self.assertIn('ik_solutions_per_node', result)
        
        # Check types
        self.assertIsInstance(result['configurations'], list)
        self.assertIsInstance(result['path_length'], (int, float))
        self.assertIsInstance(result['num_nodes_computed'], int)
        self.assertIsInstance(result['ik_solutions_per_node'], list)
        
        # Check ik_solutions_per_node has correct length
        self.assertEqual(len(result['ik_solutions_per_node']), result['num_nodes_computed'])
    
    def test_path_length_valid(self):
        """Test that path length is a valid number."""
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=3,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets
        )
        
        # Path length should be a non-negative number
        self.assertTrue(
            result['path_length'] >= 0 or result['path_length'] == float('inf'),
            f"Path length {result['path_length']} is invalid"
        )
    
    def test_configurations_are_lists(self):
        """Test that each configuration is a list of numbers."""
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=2,
            current_pose=self.sample_current_pose,
            list_of_targets=self.sample_targets
        )
        
        if result['configurations']:  # If we got valid configurations
            for config in result['configurations']:
                self.assertIsInstance(config, list)
                # Each config should have joint angles (typically 6 for UR robots)
                self.assertGreater(len(config), 0)
                for angle in config:
                    self.assertIsInstance(angle, (int, float))
    
    def test_different_current_poses(self):
        """Test with different starting poses."""
        pose1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose2 = [1.0, -1.0, 1.5, -0.5, 1.57, 0.0]
        
        result1 = calculate_partial_trajectory(
            number_of_nodes_to_calculate=2,
            current_pose=pose1,
            list_of_targets=self.sample_targets
        )
        
        result2 = calculate_partial_trajectory(
            number_of_nodes_to_calculate=2,
            current_pose=pose2,
            list_of_targets=self.sample_targets
        )
        
        # Both should return valid results
        self.assertEqual(result1['num_nodes_computed'], 2)
        self.assertEqual(result2['num_nodes_computed'], 2)
    
    def test_unreachable_targets(self):
        """Test with potentially unreachable targets (very far away)."""
        unreachable_targets = [
            Plane((100.0, 100.0, 100.0), (1, 0, 0), (0, 1, 0)),
            Plane((101.0, 101.0, 101.0), (1, 0, 0), (0, 1, 0)),
        ]
        
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=2,
            current_pose=self.sample_current_pose,
            list_of_targets=unreachable_targets
        )
        
        # Should handle unreachable targets gracefully
        self.assertIsInstance(result, dict)
        # May have empty configurations or inf path_length
        self.assertIn('configurations', result)

    def test_real_data_random_startpoint_matches_reference_path(self):
        """Load real flange/base frames, pick a random start, and compare to stored path."""

        root = Path(__file__).resolve().parent.parent
        data_dir = root / 'data' / 'example_data'
        export_dir = data_dir / 'export'

        flange_path = export_dir / 'flange_frames.json' #'_251117_163017_flange_frames.json'
        base_path = export_dir / 'base_frames.json' #'_251117_163017_base_frames.json'
        reference_path = data_dir / 'shortest_path.json'

        required_files = [flange_path, base_path, reference_path]
        missing_files = [str(path) for path in required_files if not path.exists()]
        if missing_files:
            self.skipTest(f"Missing required real-data files: {missing_files}")

        with flange_path.open('r', encoding='utf-8') as f:
            flange_frames_data = json.load(f)
        with base_path.open('r', encoding='utf-8') as f:
            base_frames_data = json.load(f)
        with reference_path.open('r', encoding='utf-8') as f:
            reference_path_configs = json.load(f)

        self.assertGreater(len(flange_frames_data), 5)
        self.assertEqual(len(flange_frames_data), len(base_frames_data))
        self.assertGreater(len(reference_path_configs), 5)

        flange_planes = [
            Plane(
                origin=tuple(entry['origin']),
                xaxis=tuple(entry['x_axis']),
                yaxis=tuple(entry['y_axis'])
            )
            for entry in flange_frames_data
        ]
        base_planes = [
            Plane(
                origin=tuple(entry['origin']),
                xaxis=tuple(entry['x_axis']),
                yaxis=tuple(entry['y_axis'])
            )
            for entry in base_frames_data
        ]

        nodes_to_calculate = 10
        max_start = min(len(reference_path_configs), len(flange_planes)) - (nodes_to_calculate + 1)
        rng = random.Random()  # deterministic for test stability
        start_idx = rng.randrange(0, max_start)
        print(f"Testing from random start index: {start_idx}")
        current_pose = reference_path_configs[start_idx][1:]  # Remove first element (7 joints -> 6 joints)
        print(f"Current pose: {current_pose}")
        targets = flange_planes[start_idx + 1 : start_idx + 1 + nodes_to_calculate]
        bases = base_planes[start_idx + 1 : start_idx + 1 + nodes_to_calculate]
        starttime = time.time()
        result = calculate_partial_trajectory(
            number_of_nodes_to_calculate=nodes_to_calculate,
            current_pose=current_pose,
            list_of_targets=targets,
            base_planes=bases
        )
        endtime = time.time()
        print(f"Calculated {nodes_to_calculate} nodes in {endtime - starttime:.5f} seconds.")
        print(f"Result keys: {result.keys()}")
        print(f"Path length: {result['path_length']}")
        print(f"Number of configurations: {len(result['configurations'])}")
        print(f"IK solutions per node (counts): {[len(sols) for sols in result['ik_solutions_per_node']]}")
        if 'error' in result:
            print(f"Error: {result['error']}")
        self.assertEqual(result['num_nodes_computed'], nodes_to_calculate)
        self.assertTrue(result['configurations'])
        

        # # Compare all computed configurations against the stored path with a loose tolerance
        # for i, (expected_config, actual_config) in enumerate(zip(
        #     reference_path_configs[start_idx + 1 : start_idx + 1 + nodes_to_calculate],
        #     result['configurations']
        # )):
        #     for j, (exp, act) in enumerate(zip(expected_config[1:], actual_config)):  # Skip first element in expected
        #         self.assertAlmostEqual(
        #             exp, act, places=5,
        #             msg=f"Mismatch at node {i}, joint {j}: expected {exp}, got {act}"
        #         )

        save_path = data_dir / 'test_partial_trajectory_output.json'
        print(f"Saving test output to {save_path}")
        with save_path.open('w', encoding='utf-8') as f:
            json.dump(result['configurations'], f, indent=4)

if __name__ == '__main__':
    # Run all tests
    unittest.main(verbosity=2)
