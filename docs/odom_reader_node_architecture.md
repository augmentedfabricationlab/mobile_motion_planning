# OdomReaderNode Architecture

## Complete System UML Diagram

```mermaid
graph LR
    subgraph INPUT["📥 INPUT"]
        Odom["Odometry<br/>/robot/robotnik_base_control/odom"]
        ExecIdx["Exec Index<br/>ur_pose_streamer/exec_index"]
        JointState["Joint States<br/>/robot/joint_states"]
    end

    subgraph CALLBACKS["📍 CALLBACKS"]
        OdomCB["odom_callback()<br/>Extract position<br/>+ set initial pose"]
        ExecIdxCB["exec_index_callback()<br/>Update index<br/>+ trigger replan"]
        JointStateCB["joint_state_callback()<br/>Extract joint config<br/>+ seed targets"]
    end

    subgraph STATE["💾 STATE"]
        OdomState["Odom State<br/>current_x/y/z<br/>positions"]
        ExecState["Exec State<br/>exec_index<br/>motion_started"]
        JointState_S["Joint State<br/>current_pose<br/>pose_by_index"]
        PlanState["Plan State<br/>replan_total<br/>replan_failed"]
        BaseState["Base State<br/>initial_plane<br/>target_planes"]
    end

    subgraph PROCESSING["⚙️ PROCESSING"]
        ReplanPub["replan_and_<br/>publish()<br/>Plan+Publish"]
        SeedInit["seed_initial_<br/>targets()<br/>Init Seed"]
        ComputeBase["compute_base_<br/>plane()<br/>Transform"]
    end

    subgraph HELPERS["🔧 HELPERS"]
        CalcPartial["calc_partial_<br/>trajectory()<br/>Call Planner"]
        PlanKwargs["planning_<br/>kwargs()<br/>Config"]
        EnsurePath["ensure_<br/>path()<br/>Inject Paths"]
        LoadPlanes["load_planes_<br/>json()<br/>Parse JSON"]
        LoadBase["load_base_<br/>plane()<br/>Init Plane"]
    end

    subgraph EXTERNAL["🔗 EXTERNAL"]
        SelectBuffer["select_buffer_<br/>tail_pose()"]
        CalcPartialExt["calculate_<br/>partial_<br/>trajectory()"]
        Plane["Plane<br/>geometry"]
    end

    subgraph TIMERS["⏱️ TIMERS"]
        MainTimer["10Hz Timer"]
        ProcessData["process_<br/>data()"]
        BaseTimer["Base Rate<br/>Timer"]
        PublishCmd["publish_<br/>base_cmd()"]
    end

    subgraph METRICS["📊 METRICS"]
        OpenCSV["open_<br/>metrics_csv()"]
        WriteRow["write_<br/>metrics_row()"]
        CloseCSV["close_<br/>metrics_csv()"]
    end

    subgraph OUTPUT["📤 OUTPUT"]
        ReplanPub_Out["Replanned Target<br/>ur_pose_streamer/replanned_target"]
        BaseCmdPub["Base Cmd<br/>/robot/move_base/cmd_vel"]
    end

    subgraph DATA_FILES["📁 FILES"]
        TargetJSON["target_planes.json"]
        BaseJSON["base_planes.json"]
        MetricsFile["metrics_*.csv"]
    end

    subgraph CONFIG["⚙️ CONFIG"]
        Params["ROS2 Params<br/>(22 total)"]
    end

    %% Input flows
    Odom -->|message| OdomCB
    ExecIdx -->|message| ExecIdxCB
    JointState -->|message| JointStateCB

    %% Callback to State
    OdomCB -->|updates| OdomState
    ExecIdxCB -->|updates| ExecState
    JointStateCB -->|updates| JointState_S

    %% State to Processing
    OdomState -->|input| ReplanPub
    OdomState -->|input| ComputeBase
    ExecState -->|input| ReplanPub
    JointState_S -->|input| ReplanPub
    JointState_S -->|input| SeedInit
    BaseState -->|input| ReplanPub
    BaseState -->|input| SeedInit
    PlanState -->|input| ReplanPub

    %% Core relationships
    ReplanPub -->|calls| ComputeBase
    ReplanPub -->|calls| CalcPartial
    ReplanPub -->|calls| SelectBuffer
    ReplanPub -->|updates| PlanState
    ReplanPub -->|writes| WriteRow

    SeedInit -->|calls| ComputeBase
    SeedInit -->|calls| CalcPartial
    SeedInit -->|updates| JointState_S
    SeedInit -->|writes| WriteRow

    %% Triggers
    ExecIdxCB -->|trigger| ReplanPub
    JointStateCB -->|trigger| SeedInit

    %% Helpers
    CalcPartial -->|calls| CalcPartialExt
    CalcPartial -->|calls| EnsurePath
    CalcPartial -->|uses| PlanKwargs

    %% Data files
    LoadPlanes -->|reads| TargetJSON
    LoadBase -->|reads| BaseJSON
    LoadBase -->|calls| LoadPlanes

    %% Timers
    MainTimer -->|trigger| ProcessData
    ProcessData -->|calls| ComputeBase
    BaseTimer -->|trigger| PublishCmd
    PublishCmd -->|checks| ExecState

    %% Metrics
    OpenCSV -->|writes| MetricsFile
    WriteRow -->|writes| MetricsFile
    CloseCSV -->|closes| MetricsFile
    ReplanPub -->|record| WriteRow
    SeedInit -->|record| WriteRow

    %% Outputs
    ReplanPub -->|publish| ReplanPub_Out
    SeedInit -->|publish| ReplanPub_Out
    PublishCmd -->|publish| BaseCmdPub

    %% Config
    Params -.->|configure| OdomCB
    Params -.->|configure| ExecIdxCB
    Params -.->|configure| JointStateCB
    Params -.->|configure| ReplanPub
    Params -.->|configure| SeedInit

    %% Styling
    classDef input fill:#FF6B6B,stroke:#C92A2A,color:#fff,stroke-width:2px
    classDef callback fill:#4ECDC4,stroke:#089AAC,color:#000,stroke-width:2px
    classDef state fill:#E8F4F8,stroke:#90CAF9,color:#000,stroke-width:1px
    classDef processing fill:#95E1D3,stroke:#38ADA9,color:#000,stroke-width:2px
    classDef helpers fill:#FFE66D,stroke:#FFBA08,color:#000,stroke-width:1px
    classDef external fill:#D4A5FF,stroke:#8C39C3,color:#fff,stroke-width:1px
    classDef timers fill:#FFA07A,stroke:#FF6347,color:#000,stroke-width:2px
    classDef metrics fill:#A8DADC,stroke:#457B9D,color:#000,stroke-width:1px
    classDef output fill:#B4E7FF,stroke:#0066CC,color:#000,stroke-width:2px
    classDef datafiles fill:#C1E1A6,stroke:#558B2F,color:#000,stroke-width:1px
    classDef config fill:#F0E6FF,stroke:#6B4C8A,color:#000,stroke-width:1px

    class Odom,ExecIdx,JointState input
    class OdomCB,ExecIdxCB,JointStateCB callback
    class OdomState,ExecState,JointState_S,PlanState,BaseState state
    class ReplanPub,SeedInit,ComputeBase processing
    class CalcPartial,PlanKwargs,EnsurePath,LoadPlanes,LoadBase helpers
    class SelectBuffer,CalcPartialExt,Plane external
    class MainTimer,ProcessData,BaseTimer,PublishCmd timers
    class OpenCSV,WriteRow,CloseCSV metrics
    class ReplanPub_Out,BaseCmdPub output
    class TargetJSON,BaseJSON,MetricsFile datafiles
    class Params config
```

## Data Flow Summary

### Input → Processing → Output Pipeline

1. **Input Reception**: ROS topics publish messages
   - Odometry updates robot position
   - Execution index triggers replan cycles
   - Joint states initialize planning seed

2. **State Management**: Messages update internal state variables
   - Position tracking for odometry deltas
   - Execution progress tracking
   - Joint configuration caching

3. **Core Processing**: Main logic executes based on triggers
   - `_replan_and_publish()`: Triggered by exec_index updates
   - `_seed_initial_targets()`: Triggered by joint state callback
   - `_compute_current_base_plane()`: Applies odometry transform

4. **Helper Functions**: Support core processing
   - `_calculate_partial_trajectory()`: Invokes external IK/path planner
   - `_planning_kwargs()`: Prepares planner configuration
   - `_ensure_replanning_path()`: Manages Python import paths

5. **Metrics Recording**: All operations logged to CSV
   - Execution index, target index, latency
   - Compute time, path length, success flag

6. **Output Publishing**: Results sent to ROS topics
   - Replanned targets → ur_pose_streamer/replanned_target
   - Base commands → /robot/move_base/cmd_vel

7. **Timer Loops**: Scheduled background tasks
   - 10Hz: Process odometry and log base plane
   - Configurable rate: Publish base motion commands

## Function Call Hierarchy

```
OdomReaderNode.__init__()
├── _open_metrics_csv()
├── _load_planes_from_json() → target_planes
├── _load_initial_base_plane_from_json()
│   └── _load_planes_from_json() → base_planes
├── create_subscription(odom_topic, odom_callback)
├── create_subscription(exec_index_topic, exec_index_callback)
├── create_subscription(joint_state_topic, joint_state_callback)
├── create_publisher(replanned_target_topic)
├── create_publisher(move_base_cmd_topic)
└── create_timer(10Hz, process_data)

odom_callback(msg)
└── Updates: _current_odom_position, current_x/y/z

exec_index_callback(msg)
├── Updates: exec_index, latest_exec_index
└── Calls: _replan_and_publish()

joint_state_callback(msg)
├── Updates: current_joint_pose
└── Calls: _seed_initial_targets()

_seed_initial_targets()
├── _compute_current_base_plane()
├── _planning_kwargs()
├── _calculate_partial_trajectory()
│   ├── _ensure_replanning_path()
│   └── calculate_partial_trajectory() [EXTERNAL]
├── Publish to: replanned_target_topic
└── _write_metrics_row()

_replan_and_publish()
├── _compute_current_base_plane()
├── _planning_kwargs()
├── _calculate_partial_trajectory() [same as above]
├── select_buffer_tail_pose() [EXTERNAL]
├── Publish to: replanned_target_topic
└── _write_metrics_row()

process_data() [10Hz Timer]
├── _log_current_base_plane_if_needed()
│   └── _compute_current_base_plane()
└── Logs: base plane position deltas

_publish_base_move_cmd() [Base Rate Timer]
└── Publish to: move_base_cmd_topic

close_metrics_csv()
└── Flush and close metrics file
```

## Parameter Configuration

All behavior controlled via ROS2 parameters:

| Parameter | Type | Purpose |
|-----------|------|---------|
| `odom_topic` | string | Odometry subscription topic |
| `exec_index_topic` | string | Execution index subscription topic |
| `joint_state_topic` | string | Joint states subscription topic |
| `replanned_target_topic` | string | Replanned target publication topic |
| `move_base_cmd_topic` | string | Base movement command publication topic |
| `target_planes_json` | string | Path to target planes JSON file |
| `base_planes_json` | string | Path to initial base plane JSON file |
| `lookahead_nodes` | int | Number of nodes to plan ahead |
| `robot_buffer_size` | int | UR buffer size for replanning |
| `rotation_mode` | string | Planning rotation mode ('True'/'False') |
| `rotation_angle_deg` | float | Rotation angle for planning |
| `rotation_steps` | int | Number of rotation steps |
| `rotation_angle_cw_deg` | float | Clockwise rotation angle |
| `rotation_angle_ccw_deg` | float | Counter-clockwise rotation angle |
| `path_builder_iterations` | int | Iterations for path building |
| `enable_collision_check` | bool | Enable collision checking |
| `collision_data_path` | string | Path to collision data |
| `suppress_motion_planning_messages` | bool | Suppress planner console output |
| `log_current_base_plane` | bool | Log base plane updates |
| `base_plane_log_rate_hz` | float | Frequency for base plane logging |
| `record_metrics_csv` | bool | Enable metrics CSV recording |
| `metrics_csv_dir` | string | Directory for metrics CSV files |
| `move_base_linear_x` | float | Linear velocity for base motion |
| `move_base_rate_hz` | float | Frequency for base motion commands |
| `joint_names` | string[] | UR joint names to track |

## Key State Variables

- **Position Tracking**: `current_x`, `current_y`, `current_z`, `_current_odom_position`
- **Execution Tracking**: `exec_index`, `latest_exec_index`, `_base_motion_started`
- **Joint Tracking**: `current_joint_pose`, `planned_joint_pose_by_index`
- **Planning State**: `_replan_total`, `_replan_failed`, `last_published_replanned_index`
- **Base Frames**: `_initial_base_plane`, `_initial_odom_position`, `target_planes`
- **Metrics**: `_metrics_csv_file`, `_metrics_csv_writer`
- **Flags**: `data_received`, `_initial_seed_done`, `_replan_path_injected`

## I/O Summary

### Inputs
- **ROS Topics**: 3 subscribers (odometry, execution index, joint states)
- **JSON Files**: 2 files (target planes, base planes)

### Outputs
- **ROS Topics**: 2 publishers (replanned targets, base motion commands)
- **CSV Files**: 1 metrics recording file per run

### Processing
- **12 main functions** in OdomReaderNode class
- **2 external dependencies**: `calculate_partial_trajectory()`, `select_buffer_tail_pose()`
- **2 timer loops**: 10Hz main loop + configurable base command rate
