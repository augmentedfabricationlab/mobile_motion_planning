# Rolling Replanning Pipeline

Enable live trajectory replanning with a 2-pose robot buffer. The pipeline streams poses to the UR robot, listens for executed index feedback, and recomputes the next segment on-the-fly.

## System Overview

- **ur_pose_streamer_live**: Accepts live replanned targets over ROS topic, streams them to UR robot via TCP socket, buffer size = 2
- **odom_reader_node**: Listens to odometry + exec index feedback, triggers replanning of next 2 poses, publishes only buffer-tail target, and publishes base cmd_vel after first executed pose
- **partial_trajectory.py**: IK solver + path optimization for the lookahead segment

## Startup Order

### 1. Start UR Pose Streamer (Terminal A)

```bash
cd ~/robot_ws && source install/setup.bash
ros2 run ur_pose_streamer ur_pose_streamer_live \
  --ros-args \
  -p joint_targets_live:=true \
  -p initial_buffer:=2 \
  -p replanned_target_topic:=ur_pose_streamer/replanned_target
```

**Waits for UR TCP connection on `0.0.0.0:50012`**

### 2. Start Odometry Reader (Terminal B)

```bash
cd ~/robot_ws && source install/setup.bash
ros2 run mobile_motion_planning odom_reader_node \
  --ros-args \
  -p odom_topic:=/robot/robotnik_base_control/odom \
  -p exec_index_topic:=ur_pose_streamer/exec_index \
  -p joint_state_topic:=/robot/joint_states \
  -p move_base_cmd_topic:=/robot/move_base/cmd_vel \  
  -p replanned_target_topic:=ur_pose_streamer/replanned_target \
  -p target_planes_json:=/path/to/target_planes.json \
  -p base_planes_json:=/path/to/base_planes.json \
  -p lookahead_nodes:=2 \
  -p robot_buffer_size:=2 \
  -p move_base_linear_x:=-0.001 \
  -p move_base_rate_hz:=100.0 \
  -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]"


ros2 run mobile_motion_planning odom_reader_node --ros-args -p rotation_mode:=step_angle -p rotation_angle_cw_deg:=2.0 -p rotation_angle_ccw_deg:=2.0
```

### 2a. Rotation ON, Collision Checking OFF

Use this when you want orientation search around each target plane but no PyBullet collision culling:

```bash
cd ~/robot_ws && source install/setup.bash
ros2 run mobile_motion_planning odom_reader_node \
  --ros-args \
  -p target_planes_json:=/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/example_data/260311_Segment_4/260311_150455_flange_frames.json \
  -p base_planes_json:=/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/example_data/260311_Segment_4/260311_150455_base_frames.json \
  -p rotation_mode:=step_angle \
  -p rotation_angle_deg:=5.0 \
  -p rotation_angle_cw_deg:=45.0 \
  -p rotation_angle_ccw_deg:=45.0 \
  -p enable_collision_check:=false
```

For full-circle sampling instead of bounded angle sampling, use:

```bash
-p rotation_mode:=n_steps -p rotation_steps:=35
```

### Reader Behavior

Subscribes to odom + exec index, publishes replanned targets.

The initial seed targets are now published with transient-local durability, so the streamer can still receive them if it subscribes after `odom_reader_node` has already seeded the first two poses.

`base_planes_json` is interpreted as an initial base frame reference. Only the first base plane is used, and then translated using odometry delta (no rotation applied).

The base velocity command is published from `odom_reader_node` automatically at 100 Hz after the first executed robot index is received (`exec_index >= 0`).

`rotation_mode` is a string parameter. In ROS 2 CLI, `False` may still be parsed as YAML boolean even when quoted.
Use one of these:

```bash
ros2 run mobile_motion_planning odom_reader_node

# or 
ros2 run mobile_motion_planning odom_reader_node --ros-args -p rotation_mode:=none
```

### 3. Connect UR Robot

Ensure UR robot connects to streamer socket at `localhost:50012`. Streamer logs "UR connected from ...".

## Data Flow

```
UR Robot 
  ├─ executes pose, sends current index
  │
  └─→ ur_pose_streamer publishes exec_index
        │
        └─→ odom_reader exec_index_callback triggered
              ├─ reads current joint state
              ├─ calls calculate_partial_trajectory (2 lookahead poses)
              ├─ selects buffer-tail pose (index = exec_idx + buffer_size)
              └─ publishes replanned target [index, j1..j6]
                    │
                    └─→ ur_pose_streamer replanned_target_callback
                          └─ inserts pose at global index
                              └─ streams to UR when buffer window allows
```

## Key Parameters

| Parameter | Default | Notes |
| --------- | ------- | ----- |
| `joint_targets_live` | false | Set to **true** for live replanning mode |
| `initial_buffer` | 2 | Number of poses to preload on connect |
| `lookahead_nodes` | 2 | Number of poses to replan per trigger |
| `robot_buffer_size` | 2 | UR buffer constraint (2 = one initial + one tail) |
| `target_planes_json` | "" | Path to target planes file (required for replanning) |
| `base_planes_json` | "" | Path to base planes JSON (first entry used as initial base plane) |
| `move_base_cmd_topic` | /robot/move_base/cmd_vel | Twist topic for mobile base motion |
| `move_base_linear_x` | -0.001 | Linear x command sent after first executed point |
| `move_base_rate_hz` | 100.0 | Base cmd_vel publish rate |
| `rotation_mode` | `False` | Rotation search mode: `False`, `step_angle`, or `n_steps` |
| `rotation_angle_deg` | 5.0 | Step size in degrees for `step_angle` mode |
| `rotation_angle_cw_deg` | 0.0 | Clockwise bound for `step_angle` mode |
| `rotation_angle_ccw_deg` | 0.0 | Counter-clockwise bound for `step_angle` mode |
| `rotation_steps` | 35 | Number of full-circle samples when `rotation_mode:=n_steps` |
| `enable_collision_check` | false | Enable slab_net_zero PyBullet collision culling |
| `collision_data_path` | "" | Optional slab_net_zero data path override for URDF/meshes |
| `path_builder_iterations` | 10 | Random start/end samples used by graph shortest-path search |

## CSV Metrics

When `record_metrics_csv:=true`, `odom_reader_node` writes a timestamped CSV file named `replan_metrics_YYYYMMDD_HHMMSS.csv` into `/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/recordings` by default.

| Column | Meaning |
| ------ | ------- |
| `ros_time_s` | ROS time when the row is written |
| `exec_index` | Executed trajectory index that triggered replanning |
| `target_index` | Published replanned target index; `-1` if replanning failed |
| `buffer_lead` | `target_index - exec_index`; expected to match `robot_buffer_size` on success |
| `latency_ms` | End-to-end replanning latency from timing start in `_replan_and_publish()` to publish/failure handling |
| `compute_ms` | Time spent inside `calculate_partial_trajectory()` only |
| `path_length` | Joint-space path length returned by the path builder |
| `ik_counts` | Number of IK solutions per replanned node, stored as a semicolon-separated string |
| `base_dx` | Base odometry delta in x relative to initial odometry |
| `base_dy` | Base odometry delta in y relative to initial odometry |
| `base_dz` | Base odometry delta in z relative to initial odometry |
| `base_disp_m` | Euclidean norm of `(base_dx, base_dy, base_dz)` |
| `success` | `1` if a replanned target was published, `0` if replanning failed |

## Troubleshooting

- **No UR connection**: Check firewall allows TCP 50012
- **Replanning error**: Ensure `slab_net_zero` is installed; odom_reader will warn and skip replanning
- **No position output**: Confirm `/robot/robotnik_base_control/odom` is published
- **Poses not sent**: Verify exec index feedback from UR (check socket protocol)

## Notes

- Initial 2 poses are seeded from replanned topic before robot starts moving
- Only the buffer-tail pose is sent per exec_index event (not previous poses)
- Late/duplicate replanned targets are safely ignored by streamer
- Manual `ros2 topic pub /robot/move_base/cmd_vel ...` is no longer required in this pipeline; `odom_reader_node` publishes the configured command automatically after the first executed point
- The UR program must use `pose_buffered_movep.script` or equivalent logic that sends the executed index back over the socket; `pose_buffer.script` only receives poses and is not sufficient for live replanning
- The UR live script must also use a 2-pose startup buffer (`g_buf_size = 2`, `min_start = 2`) to match `initial_buffer:=2`; if it still waits for 10 poses, the robot will never start and no exec index feedback will be produced
