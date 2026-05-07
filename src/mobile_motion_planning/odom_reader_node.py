#!/usr/bin/env python3
# pylint: disable=import-error
"""
ROS 2 node that:
- Subscribes to base odometry, executed trajectory index (`exec_index`), and arm joint states.
- Loads target planes from JSON; takes a single initial base plane from JSON and transforms it
  using live odometry to compute the current base frame for replanning.
- Seeds initial UR buffer targets.
- Replans a short lookahead trajectory on execution progress updates.
- Publishes the replanned joint target so the ur_pose_streamer_live can use it.
"""
import csv
import json
import math
import os
import sys
import time
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime
from io import StringIO
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

from mobile_motion_planning.ik_offline.geometry import Plane
from mobile_motion_planning.partial_trajectory import calculate_partial_trajectory


def select_buffer_tail_pose(
    *,
    exec_index,
    replan_start_index,
    replanned_configurations,
    buffer_size,
):
    """Return the buffer-tail target index and joint values for publication."""
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

        default_data_root = (
            '/home/robot/robot_ws/src/print_while_driving_packages/' \
            'mobile_motion_planning/data/example_data/260311_Segment_4'
        )

        self.declare_parameter('odom_topic', '/robot/robotnik_base_control/odom')
        self.declare_parameter('exec_index_topic', 'ur_pose_streamer/exec_index')
        self.declare_parameter('joint_state_topic', '/robot/joint_states')
        self.declare_parameter('replanned_target_topic', 'ur_pose_streamer/replanned_target')
        self.declare_parameter('move_base_cmd_topic', '/robot/move_base/cmd_vel')
        self.declare_parameter('move_base_linear_x', -0.001)
        self.declare_parameter('move_base_rate_hz', 100.0)
        self.declare_parameter(
            'target_planes_json',
            f'{default_data_root}/260311_150455_flange_frames.json',
        )
        self.declare_parameter(
            'base_planes_json',
            f'{default_data_root}/260311_150455_base_frames.json',
        )
        self.declare_parameter('lookahead_nodes', 2)
        self.declare_parameter('robot_buffer_size', 2)
        self.declare_parameter('rotation_mode', 'False')
        self.declare_parameter('rotation_angle_deg', 5.0)
        self.declare_parameter('rotation_steps', 35)
        self.declare_parameter('rotation_angle_cw_deg', 0.0)
        self.declare_parameter('rotation_angle_ccw_deg', 0.0)
        self.declare_parameter('path_builder_iterations', 10)
        self.declare_parameter('enable_collision_check', False)
        self.declare_parameter('collision_data_path', '')
        self.declare_parameter('suppress_motion_planning_messages', True)
        self.declare_parameter('log_current_base_plane', True)
        self.declare_parameter('base_plane_log_rate_hz', 1.0)
        self.declare_parameter('record_metrics_csv', True)
        self.declare_parameter(
            'metrics_csv_dir',
            '/home/robot/robot_ws/src/print_while_driving_packages/' \
            'mobile_motion_planning/data/recordings',
        )
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
        self.rotation_mode = (
            self.get_parameter('rotation_mode').get_parameter_value().string_value
        )
        self.rotation_angle_deg = (
            self.get_parameter('rotation_angle_deg').get_parameter_value().double_value
        )
        self.rotation_steps = (
            self.get_parameter('rotation_steps').get_parameter_value().integer_value
        )
        self.rotation_angle_cw_deg = (
            self.get_parameter('rotation_angle_cw_deg').get_parameter_value().double_value
        )
        self.rotation_angle_ccw_deg = (
            self.get_parameter('rotation_angle_ccw_deg').get_parameter_value().double_value
        )
        self.path_builder_iterations = (
            self.get_parameter('path_builder_iterations').get_parameter_value().integer_value
        )
        self.enable_collision_check = (
            self.get_parameter('enable_collision_check').get_parameter_value().bool_value
        )
        self.collision_data_path = (
            self.get_parameter('collision_data_path').get_parameter_value().string_value
        )
        self.suppress_motion_planning_messages = (
            self.get_parameter('suppress_motion_planning_messages')
            .get_parameter_value()
            .bool_value
        )
        self.log_current_base_plane = (
            self.get_parameter('log_current_base_plane').get_parameter_value().bool_value
        )
        self.base_plane_log_rate_hz = (
            self.get_parameter('base_plane_log_rate_hz').get_parameter_value().double_value
        )
        self.record_metrics_csv = (
            self.get_parameter('record_metrics_csv').get_parameter_value().bool_value
        )
        self.move_base_cmd_topic = (
            self.get_parameter('move_base_cmd_topic').get_parameter_value().string_value
        )
        self.move_base_linear_x = (
            self.get_parameter('move_base_linear_x').get_parameter_value().double_value
        )
        self.move_base_rate_hz = (
            self.get_parameter('move_base_rate_hz').get_parameter_value().double_value
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
        self._initial_base_plane: Optional[Plane] = self._load_initial_base_plane_from_json(
            self.base_planes_json
        )
        self._initial_odom_position: Optional[Tuple[float, float, float]] = None
        self._current_odom_position: Optional[Tuple[float, float, float]] = None
        self._base_motion_started = False
        self._last_base_plane_log_ns = 0
        self._replan_total = 0
        self._replan_failed = 0
        self._metrics_csv_file = None
        self._metrics_csv_writer = None
        if self.record_metrics_csv:
            self._open_metrics_csv()

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
        self.move_base_cmd_publisher = self.create_publisher(
            Twist,
            self.move_base_cmd_topic,
            10,
        )
        self._base_move_msg = Twist()
        self._base_move_msg.linear.x = float(self.move_base_linear_x)
        self._base_move_timer = self.create_timer(
            1.0 / max(self.move_base_rate_hz, 1.0),
            self._publish_base_move_cmd,
        )

        # Create timer for 10Hz processing (0.1 seconds = 100ms)
        self.timer = self.create_timer(0.1, self.process_data)

        self.get_logger().info(
            'Odom Reader Node started. '
            f'odom={self.odom_topic} '
            f'exec_index={self.exec_index_topic} '
            f'replanned_target={self.replanned_target_topic}'
        )
        self.get_logger().info(
            'Planning options: '
            f'rotation_mode={self.rotation_mode} '
            f'rotation_angle_deg={self.rotation_angle_deg:.3f} '
            f'rotation_steps={self.rotation_steps} '
            f'rotation_cw_deg={self.rotation_angle_cw_deg:.3f} '
            f'rotation_ccw_deg={self.rotation_angle_ccw_deg:.3f} '
            f'path_builder_iterations={self.path_builder_iterations} '
            f'enable_collision_check={self.enable_collision_check}'
        )

    def _planning_kwargs(self):
        collision_data_path = self.collision_data_path.strip()
        return {
            'rotation_mode': self.rotation_mode,
            'rotation_angle_deg': float(self.rotation_angle_deg),
            'rotation_steps': int(self.rotation_steps),
            'angle_cw_deg': float(self.rotation_angle_cw_deg),
            'angle_ccw_deg': float(self.rotation_angle_ccw_deg),
            'enable_collision_check': bool(self.enable_collision_check),
            'collision_data_path': collision_data_path if collision_data_path else None,
            'path_builder_iterations': max(1, int(self.path_builder_iterations)),
        }

    def _open_metrics_csv(self) -> None:
        csv_dir_str = (
            self.get_parameter('metrics_csv_dir').get_parameter_value().string_value.strip()
        )
        csv_dir = Path(csv_dir_str).expanduser() if csv_dir_str else Path.home()
        csv_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = csv_dir / f'replan_metrics_{ts}.csv'
        self._metrics_csv_file = open(csv_path, 'w', newline='', encoding='utf-8')  # noqa: SIM115
        self._metrics_csv_writer = csv.writer(self._metrics_csv_file)
        self._metrics_csv_writer.writerow([
            'ros_time_s', 'exec_index', 'target_index', 'buffer_lead',
            'latency_ms', 'compute_ms', 'path_length',
            'ik_counts', 'base_dx', 'base_dy', 'base_dz', 'base_disp_m', 'success',
        ])
        self._metrics_csv_file.flush()
        self.get_logger().info(f'Metrics CSV opened: {csv_path}')

    def close_metrics_csv(self) -> None:
        """Flush and close the metrics CSV file if it is open."""
        if self._metrics_csv_file is not None:
            self._metrics_csv_file.flush()
            self._metrics_csv_file.close()
            self._metrics_csv_file = None
            self._metrics_csv_writer = None

    def _write_metrics_row(
        self,
        *,
        exec_index: int,
        target_index: int,
        latency_ms: float,
        compute_ms: float,
        path_length: float,
        ik_solutions_per_node,
        success: bool,
    ) -> None:
        if self._metrics_csv_writer is None:
            return
        dx, dy, dz = 0.0, 0.0, 0.0
        if self._initial_odom_position is not None and self._current_odom_position is not None:
            dx = self._current_odom_position[0] - self._initial_odom_position[0]
            dy = self._current_odom_position[1] - self._initial_odom_position[1]
            dz = self._current_odom_position[2] - self._initial_odom_position[2]
        disp = math.sqrt(dx * dx + dy * dy + dz * dz)
        buffer_lead = target_index - exec_index if target_index >= 0 else -1
        ik_counts_str = ';'.join(str(len(s)) for s in ik_solutions_per_node)
        ros_time_s = self.get_clock().now().nanoseconds * 1e-9
        self._metrics_csv_writer.writerow([
            f'{ros_time_s:.6f}', exec_index, target_index, buffer_lead,
            f'{latency_ms:.3f}', f'{compute_ms:.3f}', f'{path_length:.6f}',
            ik_counts_str, f'{dx:.6f}', f'{dy:.6f}', f'{dz:.6f}', f'{disp:.6f}',
            int(success),
        ])
        self._metrics_csv_file.flush()

    def _calculate_partial_trajectory(self, **kwargs):

        if not self.suppress_motion_planning_messages:
            return calculate_partial_trajectory(**kwargs)

        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            return calculate_partial_trajectory(**kwargs)

    def _publish_base_move_cmd(self) -> None:
        if not self._base_motion_started:
            return
        self.move_base_cmd_publisher.publish(self._base_move_msg)

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

    def _load_initial_base_plane_from_json(self, json_path: str) -> Optional[Plane]:
        """Load a single initial base plane from JSON (only the first entry is used)."""
        planes = self._load_planes_from_json(json_path)
        if not planes:
            return None
        if len(planes) > 1:
            self.get_logger().warning(
                f'base_planes_json contains {len(planes)} planes; '
                'using only the first as the initial base plane.'
            )
        return planes[0]

    def _compute_current_base_plane(self) -> Optional[Plane]:
        """Translate the initial base plane by the odometry position delta."""
        if self._initial_base_plane is None:
            return None
        if self._initial_odom_position is None:
            return self._initial_base_plane

        dx = self._current_odom_position[0] - self._initial_odom_position[0]
        # dy = self._current_odom_position[1] - self._initial_odom_position[1]
        # dz = self._current_odom_position[2] - self._initial_odom_position[2]

        new_origin = (
            float(self._initial_base_plane.origin[0]),# - dy),
            float(self._initial_base_plane.origin[1] + dx),
            float(self._initial_base_plane.origin[2]), # - dz),
        )
        new_xaxis = tuple(float(v) for v in self._initial_base_plane.xaxis)
        new_yaxis = tuple(float(v) for v in self._initial_base_plane.yaxis)
        return Plane(new_origin, new_xaxis, new_yaxis)

    def _log_current_base_plane_if_needed(self) -> None:
        if not self.log_current_base_plane:
            return

        rate_hz = max(float(self.base_plane_log_rate_hz), 0.0)
        if rate_hz <= 0.0:
            return

        period_ns = int(1e9 / rate_hz)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_base_plane_log_ns < period_ns:
            return

        base_plane = self._compute_current_base_plane()
        if base_plane is None:
            return

        dx = 0.0
        dy = 0.0
        dz = 0.0
        if self._initial_odom_position is not None and self._current_odom_position is not None:
            dx = self._current_odom_position[0] - self._initial_odom_position[0]
            dy = self._current_odom_position[1] - self._initial_odom_position[1]
            dz = self._current_odom_position[2] - self._initial_odom_position[2]

        self._last_base_plane_log_ns = now_ns
        self.get_logger().info(
            'Current base plane origin='
            f'({base_plane.origin[0]:.6f}, {base_plane.origin[1]:.6f}, {base_plane.origin[2]:.6f}) '
            f'delta =({dx:.6f}, {dy:.6f}, {dz:.6f})'
        )

    def odom_callback(self, msg):
        """Callback function for odometry messages."""
        pos = msg.pose.pose.position
        self._current_odom_position = (float(pos.x), float(pos.y), float(pos.z))

        if self._initial_odom_position is None:
            self._initial_odom_position = self._current_odom_position
            self.get_logger().info(
                f'Initial odometry recorded: x={pos.x:.3f} y={pos.y:.3f} z={pos.z:.3f}'
            )

        self.current_x = pos.x
        self.current_y = pos.y
        self.current_z = pos.z
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
            self._base_motion_started = False

        if new_exec_index >= 0 and not self._base_motion_started:
            self._base_motion_started = True
            self.get_logger().info(
                f'First point reached (exec_index={new_exec_index}). '
                f'Starting base cmd publish on {self.move_base_cmd_topic} '
                f'at {self.move_base_rate_hz:.1f} Hz '
                f'with linear.x={self.move_base_linear_x:.6f}'
            )

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

        number_of_nodes = min(self.robot_buffer_size, len(self.target_planes))
        current_base_plane = self._compute_current_base_plane()
        base_for_seed = (
            [current_base_plane] * len(self.target_planes)
            if current_base_plane is not None
            else None
        )
        result = self._calculate_partial_trajectory(
            list_of_targets=self.target_planes,
            base_planes=base_for_seed,
            current_pose=self.current_joint_pose,
            number_of_nodes_to_calculate=number_of_nodes,
            **self._planning_kwargs(),
        )

        configs = result.get('configurations', [])
        if not configs:
            if base_for_seed is not None:
                self.get_logger().warning(
                    'Initial seed planning with base_planes returned no configurations;' \
                    ' retrying in world frame.'
                )
                result = self._calculate_partial_trajectory(
                    list_of_targets=self.target_planes,
                    base_planes=None,
                    current_pose=self.current_joint_pose,
                    number_of_nodes_to_calculate=number_of_nodes,
                    **self._planning_kwargs(),
                )
                configs = result.get('configurations', [])

            if not configs:
                ik_counts = [
                    len(s) for s in result.get('ik_solutions_per_node', [])
                ]
                self.get_logger().error(
                    f'Initial seed planning returned no configurations.' 
                    f'IK counts per node: {ik_counts}'
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
                    'Skipping replan: no indexed executed pose for exec '
                    f'index {self.exec_index} and no joint state fallback yet'
                )
                return
            self.get_logger().warning(
                'No cached executed pose for '
                f'exec index {self.exec_index}; using current joint state fallback'
            )

        replan_start_index = self.exec_index + 1
        if replan_start_index >= len(self.target_planes):
            self.get_logger().info('No remaining targets to replan')
            return

        remaining_targets = self.target_planes[replan_start_index:]
        current_base_plane = self._compute_current_base_plane()
        remaining_base_planes = (
            [current_base_plane] * len(remaining_targets)
            if current_base_plane is not None
            else None
        )

        number_of_nodes = min(self.lookahead_nodes, len(remaining_targets))
        if number_of_nodes <= 0:
            return

        _t0 = time.perf_counter()
        _t_compute_start = time.perf_counter()
        result = self._calculate_partial_trajectory(
            list_of_targets=remaining_targets,
            base_planes=remaining_base_planes,
            current_pose=reference_pose,
            number_of_nodes_to_calculate=number_of_nodes,
            **self._planning_kwargs(),
        )
        _compute_ms = (time.perf_counter() - _t_compute_start) * 1e3
        self._replan_total += 1

        chosen = select_buffer_tail_pose(
            exec_index=self.exec_index,
            replan_start_index=replan_start_index,
            replanned_configurations=result.get('configurations', []),
            buffer_size=self.robot_buffer_size,
        )
        if chosen is None:
            self._replan_failed += 1
            _latency_ms = (time.perf_counter() - _t0) * 1e3
            self._write_metrics_row(
                exec_index=self.exec_index,
                target_index=-1,
                latency_ms=_latency_ms,
                compute_ms=_compute_ms,
                path_length=result.get('path_length', float('inf')),
                ik_solutions_per_node=result.get('ik_solutions_per_node', []),
                success=False,
            )
            self.get_logger().warning(
                f'Replan produced no publishable buffer-tail pose for exec index {self.exec_index} '
                f'(failed {self._replan_failed}/{self._replan_total})'
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
        _latency_ms = (time.perf_counter() - _t0) * 1e3
        self._write_metrics_row(
            exec_index=self.exec_index,
            target_index=target_index,
            latency_ms=_latency_ms,
            compute_ms=_compute_ms,
            path_length=result.get('path_length', float('inf')),
            ik_solutions_per_node=result.get('ik_solutions_per_node', []),
            success=True,
        )
        self.planned_joint_pose_by_index[target_index] = [
            float(v) for v in target_joint_values[:6]
        ]
        self.last_published_replanned_index = target_index
        self.get_logger().info(
            f'Published replanned index {target_index} based on exec index {self.exec_index} '
            f'(latency={_latency_ms:.1f}ms compute={_compute_ms:.1f}ms)'
        )

    def process_data(self):
        """Process odometry data at 10Hz (called by timer)."""
        if not self.data_received:
            self.get_logger().warn(
                'No odometry data received yet',
                throttle_duration_sec=5.0,
            )
            return
        self._log_current_base_plane_if_needed()


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = OdomReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_metrics_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
