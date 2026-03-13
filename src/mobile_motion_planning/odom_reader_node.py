#!/usr/bin/env python3
"""
ROS 2 node that:
- Subscribes to base odometry, executed trajectory index (`exec_index`), and arm joint states.
- Loads target/base planes from JSON.
- Seeds initial UR buffer targets.
- Replans a short lookahead trajectory on execution progress updates.
- Publishes the replanned buffer-tail joint target as `Float64MultiArray` so the ur_pose_streamer_live can use it.
"""
import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

from mobile_motion_planning.ik_offline.geometry import Plane


def select_buffer_tail_pose(
    *,
    exec_index,
    replan_start_index,
    replanned_configurations,
    buffer_size,
):
    if buffer_size < 1:
        raise ValueError('buffer_size must be >= 1')

    if not replanned_configurations:
        return None

    tail_global_index = exec_index + buffer_size
    tail_offset = tail_global_index - replan_start_index
    if tail_offset < 0 or tail_offset >= len(replanned_configurations):
        return None

    return tail_global_index, replanned_configurations[tail_offset]


class OdomReaderNode(Node):
    """Node that reads odometry data and processes it at 10Hz."""

    def __init__(self):
        super().__init__('odom_reader_node')

        self.declare_parameter('odom_topic', '/robot/robotnik_base_control/odom')
        self.declare_parameter('exec_index_topic', 'ur_pose_streamer/exec_index')
        self.declare_parameter('joint_state_topic', '/robot/joint_states')
        self.declare_parameter('replanned_target_topic', 'ur_pose_streamer/replanned_target')
        self.declare_parameter('target_planes_json', '')
        self.declare_parameter('base_planes_json', '')
        self.declare_parameter('lookahead_nodes', 2)
        self.declare_parameter('robot_buffer_size', 2)
        self.declare_parameter(
            'joint_names',
            [
                'robot_arm_shoulder_pan_joint',
                'robot_arm_shoulder_lift_joint',
                'robot_arm_elbow_joint',
                'robot_arm_wrist_1_joint',
                'robot_arm_wrist_2_joint',
                'robot_arm_wrist_3_joint',
            ],
        )

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.exec_index_topic = (
            self.get_parameter('exec_index_topic').get_parameter_value().string_value
        )
        self.joint_state_topic = (
            self.get_parameter('joint_state_topic').get_parameter_value().string_value
        )
        self.replanned_target_topic = (
            self.get_parameter('replanned_target_topic').get_parameter_value().string_value
        )
        self.target_planes_json = (
            self.get_parameter('target_planes_json').get_parameter_value().string_value
        )
        self.base_planes_json = (
            self.get_parameter('base_planes_json').get_parameter_value().string_value
        )
        self.lookahead_nodes = (
            self.get_parameter('lookahead_nodes').get_parameter_value().integer_value
        )
        self.robot_buffer_size = (
            self.get_parameter('robot_buffer_size').get_parameter_value().integer_value
        )
        self.joint_names = list(
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        
        # Initialize position variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.data_received = False
        self.exec_index = -1
        self.latest_exec_index = -1
        self.last_published_replanned_index = -1
        self.current_joint_pose: Optional[List[float]] = None
        self.planned_joint_pose_by_index = {}
        self._replan_import_warned = False
        self._initial_seed_done = False
        self._replan_path_injected = False
        self.target_planes = self._load_planes_from_json(self.target_planes_json)
        self.base_planes = self._load_planes_from_json(self.base_planes_json)

        if self.base_planes and len(self.base_planes) != len(self.target_planes):
            raise ValueError(
                f'base_planes length ({len(self.base_planes)}) must match target_planes length ({len(self.target_planes)})'
            )
        
        # Create subscriber to odometry topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # Subscribe to the UR executed pose index published by ur_pose_streamer_node_live
        self.exec_index_subscriber = self.create_subscription(
            Int32,
            self.exec_index_topic,
            self.exec_index_callback,
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.replanned_target_publisher = self.create_publisher(
            Float64MultiArray,
            self.replanned_target_topic,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        
        # Create timer for 10Hz processing (0.1 seconds = 100ms)
        self.timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info(
            f'Odom Reader Node started. odom={self.odom_topic} exec_index={self.exec_index_topic} replanned_target={self.replanned_target_topic}'
        )

    def _ensure_replanning_path(self) -> None:
        if self._replan_path_injected:
            return

        pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
        roots = []

        # 1) Current shell cwd (typical when started from workspace root)
        roots.append(Path.cwd().resolve())

        # 2) Derive workspace root from installed file path (.../robot_ws/install/...)
        resolved = Path(__file__).resolve()
        install_root = None
        for parent in resolved.parents:
            if parent.name == 'install':
                install_root = parent
                break
        if install_root is not None:
            roots.append(install_root.parent)

        # 3) Fallback from environment variable loaded by robot bringup scripts
        workspace_env = os.environ.get('WORKSPACE', '').strip()
        if workspace_env:
            ws_path = Path(workspace_env)
            if ws_path.name == 'setup.bash':
                ws_path = ws_path.parent.parent
            roots.append(ws_path.resolve())

        checked = []
        seen = set()
        for root in roots:
            if str(root) in seen:
                continue
            seen.add(str(root))

            # Try both common venv locations.
            for candidate in (
                root / 'src' / '.venv' / 'lib' / pyver / 'site-packages',
                root / '.venv' / 'lib' / pyver / 'site-packages',
            ):
                checked.append(str(candidate))
                if candidate.exists():
                    sys.path.insert(0, str(candidate))
                    self._replan_path_injected = True
                    self.get_logger().info(f'Added replanning site-packages path: {candidate}')
                    return

        self.get_logger().warning(
            'Could not locate venv site-packages for replanning backend. Checked: '
            + ', '.join(checked)
        )

    def _load_planes_from_json(self, json_path: str) -> List[Plane]:
        if not json_path.strip():
            return []

        path = Path(json_path).expanduser()
        with path.open('r', encoding='utf-8') as handle:
            payload = json.load(handle)

        rows = payload.get('planes') if isinstance(payload, dict) else payload
        if not isinstance(rows, list):
            raise ValueError(f'Expected a list of planes in {path}')

        planes = []
        for idx, row in enumerate(rows):
            if isinstance(row, dict):
                # Try new format first (origin, x_axis, y_axis)
                point = row.get('origin') or row.get('point')
                xaxis = row.get('x_axis') or row.get('xaxis')
                yaxis = row.get('y_axis') or row.get('yaxis')
            elif isinstance(row, list) and len(row) == 3:
                point, xaxis, yaxis = row
            else:
                raise ValueError(f'Invalid plane at index {idx} in {path}')

            if not point or not xaxis or not yaxis:
                raise ValueError(
                    f'Plane at index {idx} missing origin/x_axis/y_axis in {path}'
                )
            planes.append(Plane(tuple(point), tuple(xaxis), tuple(yaxis)))

        self.get_logger().info(f'Loaded {len(planes)} planes from {path}')
        return planes
    
    def odom_callback(self, msg):
        """Callback function for odometry messages."""
        # Extract x, y, z position from the odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.data_received = True

    def exec_index_callback(self, msg):
        """Callback for the UR executed pose index."""
        new_exec_index = msg.data
        if new_exec_index == self.latest_exec_index:
            return

        if new_exec_index < self.latest_exec_index:
            # New run likely started, allow publishing from the beginning again.
            self.last_published_replanned_index = -1
            self.planned_joint_pose_by_index = {}

        self.exec_index = new_exec_index
        self.latest_exec_index = new_exec_index
        self._replan_and_publish()

    def joint_state_callback(self, msg: JointState):
        """Read the current UR joint pose used as planning seed."""
        if len(msg.position) < 6:
            return

        if msg.name and self.joint_names:
            index_map = {name: idx for idx, name in enumerate(msg.name)}
            if not all(name in index_map for name in self.joint_names):
                return  # message is not from the arm - ignore
            self.current_joint_pose = [
                float(msg.position[index_map[name]]) for name in self.joint_names
            ]
        else:
            self.current_joint_pose = [float(v) for v in msg.position[:6]]

        if not self._initial_seed_done and self.target_planes:
            self._seed_initial_targets()

    def _seed_initial_targets(self) -> None:
        """Publish the first robot_buffer_size targets so the streamer can seed the UR buffer."""
        self._initial_seed_done = True
        try:
            from mobile_motion_planning.partial_trajectory import calculate_partial_trajectory
        except ModuleNotFoundError as exc:
            self._ensure_replanning_path()
            try:
                from mobile_motion_planning.partial_trajectory import calculate_partial_trajectory
            except ModuleNotFoundError:
                if not self._replan_import_warned:
                    self.get_logger().error(
                        f'Replanning backend unavailable ({exc}). Cannot seed initial targets.'
                    )
                    self._replan_import_warned = True
                return

        number_of_nodes = min(self.robot_buffer_size, len(self.target_planes))
        base_for_seed = self.base_planes if self.base_planes else None
        result = calculate_partial_trajectory(
            list_of_targets=self.target_planes,
            base_planes=base_for_seed,
            current_pose=self.current_joint_pose,
            number_of_nodes_to_calculate=number_of_nodes,

        )

        configs = result.get('configurations', [])
        if not configs:
            if base_for_seed is not None:
                self.get_logger().warning(
                    'Initial seed planning with base_planes returned no configurations; retrying in world frame.'
                )
                result = calculate_partial_trajectory(
                    list_of_targets=self.target_planes,
                    base_planes=None,
                    number_of_nodes_to_calculate=number_of_nodes,
                )
                configs = result.get('configurations', [])

            if not configs:
                ik_counts = [
                    len(s) for s in result.get('ik_solutions_per_node', [])
                ]
                self.get_logger().error(
                    f'Initial seed planning returned no configurations. IK counts per node: {ik_counts}'
                )
                return

        for i, config in enumerate(configs[:self.robot_buffer_size]):
            if len(config) < 6:
                continue
            msg = Float64MultiArray()
            msg.data = [float(i)] + [float(v) for v in config[:6]]
            self.replanned_target_publisher.publish(msg)
            self.planned_joint_pose_by_index[i] = [float(v) for v in config[:6]]
            self.last_published_replanned_index = i

        self.get_logger().info(
            f'Seeded {min(len(configs), self.robot_buffer_size)} initial targets for UR buffer.'
        )

    def _replan_and_publish(self) -> None:
        if not self.data_received:
            self.get_logger().warning('Skipping replan: no odometry received yet')
            return
        if not self.target_planes:
            self.get_logger().warning(
                'Skipping replan: target_planes_json is not configured or empty'
            )
            return

        reference_pose = self.planned_joint_pose_by_index.get(self.exec_index)
        if reference_pose is None:
            reference_pose = self.current_joint_pose
            if reference_pose is None:
                self.get_logger().warning(
                    f'Skipping replan: no indexed executed pose for exec index {self.exec_index} and no joint state fallback yet'
                )
                return
            self.get_logger().warning(
                f'No cached executed pose for exec index {self.exec_index}; using current joint state fallback'
            )

        replan_start_index = self.exec_index + 1
        if replan_start_index >= len(self.target_planes):
            self.get_logger().info('No remaining targets to replan')
            return

        remaining_targets = self.target_planes[replan_start_index:]
        remaining_base_planes = (
            self.base_planes[replan_start_index:] if self.base_planes else None
        )

        number_of_nodes = min(self.lookahead_nodes, len(remaining_targets))
        if number_of_nodes <= 0:
            return

        try:
            from mobile_motion_planning.partial_trajectory import calculate_partial_trajectory
        except ModuleNotFoundError as exc:
            self._ensure_replanning_path()
            try:
                from mobile_motion_planning.partial_trajectory import calculate_partial_trajectory
            except ModuleNotFoundError:
                if not self._replan_import_warned:
                    self.get_logger().error(
                        f'Replanning backend unavailable ({exc}). Install missing dependency and retry.'
                    )
                    self._replan_import_warned = True
                return

        result = calculate_partial_trajectory(
            list_of_targets=remaining_targets,
            base_planes=remaining_base_planes,
            current_pose=reference_pose,
            number_of_nodes_to_calculate=number_of_nodes,
        )

        chosen = select_buffer_tail_pose(
            exec_index=self.exec_index,
            replan_start_index=replan_start_index,
            replanned_configurations=result.get('configurations', []),
            buffer_size=self.robot_buffer_size,
        )
        if chosen is None:
            self.get_logger().warning(
                f'Replan produced no publishable buffer-tail pose for exec index {self.exec_index}'
            )
            return

        target_index, target_joint_values = chosen
        if target_index <= self.last_published_replanned_index:
            return

        if len(target_joint_values) < 6:
            self.get_logger().warning(
                f'Replanned target for index {target_index} has fewer than 6 joints'
            )
            return

        msg = Float64MultiArray()
        msg.data = [float(target_index)] + [float(v) for v in target_joint_values[:6]]
        self.replanned_target_publisher.publish(msg)
        self.planned_joint_pose_by_index[target_index] = [
            float(v) for v in target_joint_values[:6]
        ]
        self.last_published_replanned_index = target_index
        self.get_logger().info(
            f'Published replanned target index {target_index} based on exec index {self.exec_index}'
        )
    
    def process_data(self):
        """Process odometry data at 10Hz (called by timer)."""
        if not self.data_received:
            self.get_logger().warn('No odometry data received yet', throttle_duration_sec=5.0)
            return
        
        # Get current timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Print the current position values with timestamp and UR exec index
        # print(f"[{timestamp}] exec_idx={self.exec_index} Position: x={self.current_x:.6f}, y={self.current_y:.6f}, z={self.current_z:.6f}")


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = OdomReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
