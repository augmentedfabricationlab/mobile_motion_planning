============================================================
mobile_motion_planning: Mobile motion Planning
============================================================

.. start-badges

.. image:: https://img.shields.io/badge/License-MIT-blue.svg
    :target: https://github.com/augmentedfabricationlab/mobile_motion_planning/blob/master/LICENSE
    :alt: License MIT

.. image:: https://travis-ci.org/augmentedfabricationlab/mobile_motion_planning.svg?branch=master
    :target: https://travis-ci.org/augmentedfabricationlab/mobile_motion_planning
    :alt: Travis CI

.. end-badges

**mobile_motion_planning** provides trajectory generation and rolling replanning
for a UR manipulator mounted on a moving base. It includes an odometry-driven
ROS 2 node that replans short lookahead segments online and streams updated
targets to a live UR pose streamer.


Main features
-------------

* Rolling replanning instruction buffer.
* Online odometry + execution-index feedback integration.
* Partial-trajectory IK solving and shortest-path optimization.
* Optional orientation search around target planes.
* Replanning metrics logging to CSV for latency and path quality analysis.


Package entry point
-------------------

* ``rolling_replan_node`` (ROS 2 executable)

Requirements
------------

System/runtime requirements:

* Python 3
* ROS 2 with ``rclpy``
* ROS message packages used by this node:

    * ``nav_msgs``
    * ``sensor_msgs``
    * ``std_msgs``

Python dependencies:

* ``compas``
* ``slab_net_zero`` (required for collision checking paths)


Installation
------------

Clone and build in a ROS 2 workspace:

::

    mkdir -p ~/robot_ws/src/print_while_driving_packages
    cd ~/robot_ws/src/print_while_driving_packages
    git clone https://github.com/augmentedfabricationlab/mobile_motion_planning.git
        cd ~/robot_ws
        colcon build --packages-select mobile_motion_planning
        source install/setup.bash

Optional: local editable install for pure Python workflows:

::

        pip install -e .

    or

        pip install "git+https://github.com/augmentedfabricationlab/mobile_motion_planning.git@master"


Usage: Rolling Replanning Pipeline
----------------------------------

    I usually run:
        # Start UR Pose Streamer (Terminal A):
        ros2 run ur_pose_streamer ur_pose_streamer_live_all_moves --ros-args -p joint_targets_live:=true

        # Start rolling replanning node (Terminal B):
        ros2 run mobile_motion_planning rolling_replan_node --ros-args -p rotation_mode:=step_angle -p rotation_angle_cw_deg:=10.0 -p rotation_angle_ccw_deg:=10.0

        # move base:
        ros2 topic pub /robot/move_base/cmd_vel geometry_msgs/Twist "{linear: {x: -0.001, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 100

This pipeline enables live trajectory replanning with a 2-pose robot buffer.
The system streams poses to the UR robot, listens for executed-index feedback,
and recomputes the next segment online.

System overview:

* ``ur_pose_streamer_live`` accepts replanned targets and streams to UR over TCP.
* ``rolling_replan_node`` listens to odometry + exec index and publishes the next
    buffer-tail target.
* ``partial_trajectory.py`` performs IK solving and path optimization.

Startup order
^^^^^^^^^^^^^

1. Start UR Pose Streamer (Terminal A):

::

        cd ~/robot_ws && source install/setup.bash
        ros2 run ur_pose_streamer ur_pose_streamer_live \
            --ros-args \
            -p joint_targets_live:=true \
            -p initial_buffer:=2 \
            -p replanned_target_topic:=ur_pose_streamer/replanned_target


     The streamer waits for UR TCP connection on ``0.0.0.0:50012``.

2. Start odometry reader (Terminal B):

::

        cd ~/robot_ws && source install/setup.bash
        ros2 run mobile_motion_planning rolling_replan_node \
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

3. Optional rotation search settings:

::

        ros2 run mobile_motion_planning rolling_replan_node --ros-args \
            -p target_planes_json:=/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/example_data/260311_Segment_4/260311_150455_flange_frames.json \
            -p base_planes_json:=/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/example_data/260311_Segment_4/260311_150455_base_frames.json \
            -p rotation_mode:=step_angle \
            -p rotation_angle_cw_deg:=2.0 \
            -p rotation_angle_ccw_deg:=2.0

4. Connect UR robot:

     Ensure the UR robot connects to ``localhost:50012``.

Rotation ON, collision checking OFF example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

        cd ~/robot_ws && source install/setup.bash
        ros2 run mobile_motion_planning rolling_replan_node \
            --ros-args \
            -p target_planes_json:=/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/example_data/260311_Segment_4/260311_150455_flange_frames.json \
            -p base_planes_json:=/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/example_data/260311_Segment_4/260311_150455_base_frames.json \
            -p rotation_mode:=step_angle \
            -p rotation_angle_deg:=5.0 \
            -p rotation_angle_cw_deg:=45.0 \
            -p rotation_angle_ccw_deg:=45.0 \
            -p enable_collision_check:=false

For full-circle sampling:

::

        -p rotation_mode:=n_steps -p rotation_steps:=35

Reader behavior
^^^^^^^^^^^^^^^

* Subscribes to odometry + execution index and publishes replanned targets.
* Initial seed targets are published with transient-local durability so the
    streamer can still receive them if it subscribes late.
* ``base_planes_json`` is treated as an initial base-frame reference. Only the
    first base plane is used, then translated by odometry delta (no rotation).
* The base velocity command is published automatically at 100 Hz after the
    first executed index is received (``exec_index >= 0``).

``rotation_mode`` is a string parameter. In ROS 2 CLI, ``False`` may still be
parsed as YAML boolean even when quoted. Use one of these:

::

        ros2 run mobile_motion_planning rolling_replan_node

or:

::

        ros2 run mobile_motion_planning rolling_replan_node --ros-args -p rotation_mode:=none

Data flow
^^^^^^^^^

::

        UR Robot
            executes pose, sends current index
            -> ur_pose_streamer publishes exec_index
            -> odom_reader exec_index_callback
                 - reads current joint state
                 - calls calculate_partial_trajectory (2 lookahead poses)
                 - selects buffer-tail pose (index = exec_idx + buffer_size)
                 - publishes replanned target [index, j1..j6]
            -> ur_pose_streamer replanned_target_callback inserts pose at global index
                 and streams to UR when buffer window allows

Key parameters
^^^^^^^^^^^^^^

.. list-table::
     :header-rows: 1
     :widths: 26 16 58

     * - Parameter
         - Default
         - Notes
     * - ``joint_targets_live``
         - ``false``
         - Set to ``true`` for live replanning mode.
     * - ``initial_buffer``
         - ``2``
         - Number of poses to preload on connect.
     * - ``lookahead_nodes``
         - ``2``
         - Number of poses to replan per trigger.
     * - ``robot_buffer_size``
         - ``2``
         - UR buffer constraint (one initial + one tail pose).
     * - ``target_planes_json``
         - ``""``
         - Path to target planes file (required for replanning).
     * - ``base_planes_json``
         - ``""``
         - Path to base planes JSON (first entry used as initial base plane).
     * - ``move_base_cmd_topic``
         - ``/robot/move_base/cmd_vel``
         - Twist topic for mobile base motion.
     * - ``move_base_linear_x``
         - ``-0.001``
         - Linear x command sent after first executed point.
     * - ``move_base_rate_hz``
         - ``100.0``
         - Base ``cmd_vel`` publish rate.
     * - ``rotation_mode``
         - ``False``
         - Rotation search mode: ``False``, ``step_angle``, or ``n_steps``.
     * - ``rotation_angle_deg``
         - ``5.0``
         - Step size in degrees for ``step_angle`` mode.
     * - ``rotation_angle_cw_deg``
         - ``0.0``
         - Clockwise bound for ``step_angle`` mode.
     * - ``rotation_angle_ccw_deg``
         - ``0.0``
         - Counter-clockwise bound for ``step_angle`` mode.
     * - ``rotation_steps``
         - ``35``
         - Full-circle sample count for ``rotation_mode:=n_steps``.
     * - ``enable_collision_check``
         - ``false``
         - Enable ``slab_net_zero`` PyBullet collision culling.
     * - ``collision_data_path``
         - ``""``
         - Optional data path override for URDF and meshes.
     * - ``path_builder_iterations``
         - ``10``
         - Random start/end samples used by shortest-path search.

CSV metrics
^^^^^^^^^^^

When ``record_metrics_csv:=true``, ``rolling_replan_node`` writes a timestamped CSV
named ``replan_metrics_YYYYMMDD_HHMMSS.csv`` into
``/home/robot/robot_ws/src/print_while_driving_packages/mobile_motion_planning/data/recordings``
by default.

.. list-table::
     :header-rows: 1
     :widths: 26 74

     * - Column
         - Meaning
     * - ``ros_time_s``
         - ROS time when the row is written.
     * - ``exec_index``
         - Executed trajectory index that triggered replanning.
     * - ``target_index``
         - Published replanned target index, or ``-1`` on failure.
     * - ``buffer_lead``
         - ``target_index - exec_index``; expected to match ``robot_buffer_size``.
     * - ``latency_ms``
         - End-to-end replanning latency in milliseconds.
     * - ``compute_ms``
         - Time spent inside ``calculate_partial_trajectory()`` only.
     * - ``path_length``
         - Joint-space path length returned by the path builder.
     * - ``ik_counts``
         - IK solution counts per replanned node (semicolon-separated string).
     * - ``base_dx``
         - Base odometry delta in x relative to initial odometry.
     * - ``base_dy``
         - Base odometry delta in y relative to initial odometry.
     * - ``base_dz``
         - Base odometry delta in z relative to initial odometry.
     * - ``base_disp_m``
         - Euclidean norm of ``(base_dx, base_dy, base_dz)``.
     * - ``success``
         - ``1`` when a replanned target was published, ``0`` otherwise.

Troubleshooting
^^^^^^^^^^^^^^^

* No UR connection: verify firewall and socket access for TCP port ``50012``.
* Replanning errors: ensure ``slab_net_zero`` is installed and importable.
* No base position updates: confirm
    ``/robot/robotnik_base_control/odom`` is published.
* Poses are not sent: verify UR executed-index feedback over the socket protocol.

Operational notes
^^^^^^^^^^^^^^^^^

* Initial 2 poses are seeded before robot motion starts.
* Only the buffer-tail pose is sent per ``exec_index`` event.
* Late or duplicate replanned targets are ignored by the streamer.
* Manual ``ros2 topic pub /robot/move_base/cmd_vel ...`` is not required in this
    pipeline.
* The UR program must send executed indices back over the socket (for example,
    ``pose_buffered_movep.script``-compatible behavior).
* UR live script startup buffer must match ``initial_buffer:=2``
    (for example ``g_buf_size = 2`` and ``min_start = 2``).


Contributing
------------

Set up the local development environment:

* Clone the `mobile_motion_planning <https://github.com/augmentedfabricationlab/mobile_motion_planning>`_ repository.
* Install development dependencies (for Rhino use the script):

::

    pip install -r requirements-dev.txt

Useful development tasks:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke add-to-rhino``: Optional helper for Rhino IronPython search paths.
* ``invoke``: Show available tasks.

For more details, see the `Contributor's Guide <CONTRIBUTING.rst>`_.


Releasing this project
----------------------

Release workflow (semantic versioning):

::

        invoke release patch

Replace ``patch`` with ``minor`` or ``major`` as needed.

This task bumps version metadata, builds docs/tests, creates source/wheel
artifacts, and guides upload to PyPI.


Credits
-------------

This package is maintained by the Augmented Fabrication Lab.
