# General

## Robot Platform

**Starting Point**: Existing working Dome2 robot with basic capabilities

**Hardware**:
- Dome2 chassis with differential drive
- Raspberry Pi 5
- Oak-d-lite camera (RGB + depth)
- Lidar sensor
- ROS2 Jazzy

**Current Capabilities**:
- Can make maps (SLAM)
- Can navigate to goals
- Basic teleoperation
- Sensor data publishing

**Project Goal**: Add autonomous exploration and object detection capabilities to this working platform

## Mission Specification

### Primary Mission
Autonomously explore and map an unknown indoor environment while identifying and cataloging objects of interest.

### Mission Objectives
1. **Autonomous Exploration** - Navigate and avoid obstacles autonomously
2. **Mapping & Localization** - Build 2D occupancy map with accurate localization
3. **Object Detection & Cataloging** - Identify and record object locations on map

### Success Criteria
- [ ] Navigate autonomously without collision
- [ ] Generate usable map of explored space
- [ ] Detect and identify target objects with >80% accuracy
- [ ] Log object positions on map

### Environment & Constraints
- One floor of a multi-room home (~1000-2500 sq ft)
- Wood and carpet flooring
- Standard doorways (doors assumed open)
- Stairs treated as boundaries (no-go zones)
- Adequate lighting for vision systems
- Mission duration: 15-30 minutes
- Battery: 45-60 min runtime minimum

### Design Priority: Reliability First
**Core Principle**: Autonomous operation without human intervention

- Better to explore 60% reliably than 100% with failures
- Conservative navigation with wider safety margins
- Stop safely when uncertain rather than risk collision
- Log issues for post-mission review

### Mission Behavior

**Starting Position**:
- Ad-hoc placement (no dock initially)
- Robot placed anywhere, starts at pose (0,0,0)
- SLAM builds map from starting position
- Option to add AprilTag dock later for improved return-to-home reliability

**Mission Completion**:
- Return to home position (starting location)
- Save map and object catalog

**Exploration Completion Criteria**:
Mission is "done" when ANY of the following occurs:
1. **No accessible frontiers** - All frontiers explored or unreachable
2. **Time limit reached** - Maximum 30-40 minutes
3. **Manual stop** - User terminates mission
4. **Frontier size threshold** - Only tiny frontiers remain (< min size)
5. **Failure condition** - Localization loss or critical sensor failure

**Failure Modes**:
- **Localization loss**: Declare failure, stop safely in place
- **Stuck**: Attempt recovery (back up, rotate, try alt path); if fails, stop and log
- **Sensor failure**: Continue if possible; abort if critical sensors fail

**Not Handled**:
- Low battery management (sufficient runtime assumed)

### Stuck Detection Strategy

**Definition**: Robot is "stuck" when it cannot make progress toward its navigation goal despite being commanded to move.

**Detection Methods** (multiple indicators used together):

**1. Nav2 Action Status Monitoring**
```python
# Monitor NavigateToPose action feedback
if action_status == GoalStatus.STATUS_ABORTED:
    # Nav2 gave up on this goal
    stuck_detected = True
```
- Nav2 reports when it cannot reach a goal
- Indicates path planning failed or goal unreachable
- Most reliable indicator

**2. Progress Tracking**
```python
# Check if robot position changed in last N seconds
current_position = get_robot_position()
time_since_last_progress = now() - last_progress_time

if distance(current_position, last_position) < threshold:
    if time_since_last_progress > stuck_timeout:  # e.g., 30 seconds
        stuck_detected = True
```
- Monitor actual robot position over time
- If position hasn't changed significantly in `stuck_timeout` period → stuck
- Catches cases where Nav2 is still trying but physically blocked

**3. Velocity Feedback Mismatch**
```python
# Compare commanded vs actual velocity
cmd_vel = get_commanded_velocity()
actual_vel = get_odometry_velocity()

if cmd_vel.linear.x > threshold and actual_vel.linear.x < threshold:
    if mismatch_duration > timeout:  # e.g., 5 seconds
        stuck_detected = True
```
- Robot commanded to move but not moving
- Indicates physical obstruction (caught on carpet, wedged against furniture)
- Requires odometry feedback

**4. Recovery Behavior Counter**
```python
# Track how many times Nav2 entered recovery mode
if recovery_behavior_triggered:
    recovery_count += 1
    if recovery_count > max_recovery_attempts:  # e.g., 3
        stuck_detected = True
```
- Nav2 has built-in recovery behaviors (rotate, back up, wait)
- Repeated recovery attempts indicate persistent problem
- Configured via `max_retries` in config.yaml

**5. Goal Timeout**
```python
# Overall time to reach goal
time_to_goal = now() - goal_start_time
if time_to_goal > goal_timeout:  # e.g., 120 seconds
    stuck_detected = True
```
- Timeout for reaching any single frontier
- Catches slow progress or wandering
- Prevents infinite attempts

**Combined Detection Logic**:
```python
def is_robot_stuck():
    # Use multiple indicators for reliability
    nav2_failed = (action_status == ABORTED)
    no_progress = (time_since_movement > stuck_timeout)
    too_many_recoveries = (recovery_count > max_recovery_attempts)
    goal_timeout_exceeded = (time_to_goal > goal_timeout)

    # Stuck if ANY critical indicator triggers
    return nav2_failed or no_progress or too_many_recoveries or goal_timeout_exceeded
```

**Recovery Actions When Stuck**:
```
1. First attempt: Let Nav2 recovery behaviors handle it
   - rotate_recovery: Spin in place to clear costmap
   - back_up_recovery: Back away from obstacle
   - wait_recovery: Wait for dynamic obstacle to move

2. Second attempt: Blacklist this frontier, try different goal
   - Add frontier to blacklist for 60 seconds
   - Select next best frontier
   - Continue exploration

3. Third attempt (if stuck again): Declare frontier unreachable
   - Permanently blacklist frontier
   - Log location for debugging
   - Continue with remaining frontiers

4. If repeatedly stuck everywhere: Mission failure
   - No progress possible
   - Stop safely and log diagnostics
   - Wait for human intervention
```

**Key Configuration Parameters** (from config.yaml):
```yaml
mission:
  failure:
    stuck_timeout: 30.0                # seconds of no movement
    max_recovery_attempts: 3           # before giving up on goal

navigation:
  recovery:
    retry_timeout: 30.0                # Nav2 recovery timeout
    max_retries: 3                     # Nav2 recovery attempts

exploration:
  blacklist_timeout: 60.0              # retry blacklisted frontier after
  max_blacklist_size: 20               # max unreachable frontiers
```

**Telemetry Logged**:
- Timestamp of stuck detection
- Robot position when stuck
- Goal frontier coordinates
- Number of recovery attempts
- Sensor data (lidar scan, camera image)
- Map state at time of stuck

**Prevention Strategies**:
- **Conservative costmap inflation**: Wider safety margins around obstacles
- **Lower velocities**: More time to react to obstacles
- **Frontier filtering**: Don't select frontiers too close to obstacles
- **Max goal distance**: Don't attempt very distant goals

## Software Architecture

### Overview
ROS2-based modular architecture with standard navigation stack (Nav2) and custom mission control.

### Core Components

**Prerequisites** (provided by base Dome2 system):
- **Hardware Interface Layer**: Motor control, sensor drivers (lidar, camera), odometry publishing
- **Base topics**: `/scan`, `/odom`, `/camera/rgb`, `/camera/depth`, `/cmd_vel` already available
- This application layer builds on top of the working base Dome2 platform

**1. Perception Layer**
- **SLAM Node**: SLAM Toolbox for map building and localization
- **Object Detection Node**: MobileNet-SSD on Oak-d-lite for object identification
- **Obstacle Detection**: Combines lidar + depth for obstacle avoidance

**2. Navigation Layer**
- **Nav2 Stack**: Path planning, trajectory control, behavior trees
- **Costmap**: Static map + dynamic obstacles
- **Recovery Behaviors**: Stuck detection and recovery routines

**3. Mission Control Layer**
- **Mission Manager Node**: High-level state machine (exploring, returning home, failed)
- **Exploration Node**: Autonomous Explorer package for frontier-based exploration
- **Object Catalog**: Maintains detected objects with positions

**4. Monitoring & Logging**
- **Diagnostics**: System health monitoring
- **Data Logger**: Mission telemetry for post-analysis

### Key Data Flows
```
Lidar → SLAM Toolbox → Map → Nav2 → Motor Control
Camera → Object Detection → Object Catalog
Mission Manager → Nav2 Goals → Navigation
Exploration Node → Map Analysis → Nav2 Goals
All Nodes → Diagnostics → Logger
```

### ROS2 Topics

**Prerequisites** (provided by base Dome2):
- `/scan` - Lidar data (sensor_msgs/LaserScan)
- `/camera/rgb` - RGB image (sensor_msgs/Image)
- `/camera/depth` - Depth image (sensor_msgs/Image)
- `/odom` - Odometry (nav_msgs/Odometry)
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist) - consumed by motors

**Provided by this application**:
- `/map` - Occupancy grid from SLAM Toolbox (nav_msgs/OccupancyGrid)
- `/goal_pose` - Navigation goals (geometry_msgs/PoseStamped)
- `/detected_objects` - Object detections (custom message)
- `/mission_state` - Current mission status (custom message)
- `/exploration_status` - Exploration progress (custom message)

### Technology Stack

**Confirmed Decisions**:
- **SLAM**: SLAM Toolbox (async mode)
- **Navigation**: Nav2 stack
- **Exploration**: Autonomous-Explorer-and-Mapper-ros2-nav2
- **Object Detection**: MobileNet-SSD (Luxonis model zoo, runs on Oak-d-lite Myriad X)
- **Control Interface**: TUI (textual library) + Visualization
- **Mission Logic**: Simple state machine (not behavior tree)
- **ROS2 Distribution**: Jazzy

### Control Logic Architecture

**Behavior Trees**: Used internally by Nav2 only
- Nav2 uses BehaviorTree.CPP for navigation logic
- Pre-configured behavior trees handle path planning, following, recovery
- We use these as-is, no custom behavior trees needed
- Provides robust navigation behaviors automatically

**State Machine**: Used for mission control and exploration
- Simple procedural logic for mission states (IDLE, EXPLORING, RETURNING_HOME, FAILED)
- Exploration uses timer-based sequential logic (find frontiers → pick → navigate)
- Easier to understand, debug, and maintain
- Matches reliability-first philosophy

**Why not behavior trees for mission control?**
- Mission logic is straightforward (linear flow: explore → return → done)
- State machine is simpler and more transparent
- Less complexity = fewer failure modes
- Behavior trees add overhead without clear benefit for this use case

**Architecture layering**:
```
Mission Manager (State Machine)                    ← This app
    ↓
Exploration Node (Sequential Logic)                ← This app
    ↓
Nav2 (Behavior Trees - internal)                   ← This app
    ↓
Base Dome2 Hardware Layer (Motor/Sensor drivers)   ← Prerequisite
```

**Exploration Package Decision**:

**Selected: Autonomous-Explorer-and-Mapper-ros2-nav2**
- Specifically designed for SLAM Toolbox + Nav2 + ROS2 Humble/Jazzy
- Handles dynamic map updates from SLAM Toolbox correctly
- Frontier-based exploration with dynamic goal selection
- Built-in recovery behaviors for localization failures
- Lightweight, focused implementation
- Source: https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2

**Why not m-explore-ros2**:
- Known incompatibility with SLAM Toolbox (Issue #10 since Dec 2021)
- Designed for gmapping, doesn't handle SLAM Toolbox's dynamic map growth
- Map merging algorithm breaks with SLAM Toolbox
- Source: https://github.com/robo-friends/m-explore-ros2/issues/10

**Alternative Options**:
- Custom implementation (~200-300 lines Python)
- Other community packages (ROS2-FrontierBaseExplorationForAutonomousRobot, AutoFrontierSearch_ros2-humble)

### Deployment & Development Workflow

**Installation Approach**: Native ROS2 installation
- Install ROS2 Jazzy directly on robot (Raspberry Pi 5)
- No Docker containers (simpler, better performance on embedded platform)
- Standard apt package management for ROS2 packages

**Source Control**: Git + GitHub
- Git for version control of all robot code
- GitHub as remote repository
- Deploy updates by pulling from GitHub on robot

**Update Process**:
```bash
# On robot:
cd ~/dome2_ws/src/dome2_explorer
git pull origin main
cd ~/dome2_ws
colcon build --packages-select dome2_*
source install/setup.bash
# Restart relevant nodes
```

**Development Workflow**:
- Develop and test code on development machine or robot
- Commit and push to GitHub
- Pull updates on robot for deployment
- Use branches for experimental features

### How Frontier Exploration Works

**Component Integration**:

1. **SLAM Toolbox** (runs continuously):
   - Subscribes to `/scan` and `/odom`
   - Publishes `/map` (occupancy grid: free/occupied/unknown cells)
   - Provides localization service

2. **Nav2** (navigation stack):
   - Uses `/map` from SLAM Toolbox
   - Plans paths through free space
   - Drives robot to goal poses

3. **Autonomous Explorer** (frontier detection):
   - Subscribes to `/map` from SLAM Toolbox
   - Analyzes map to find frontiers (edges between known free space and unknown)
   - Scores/ranks frontiers (closest frontier for efficiency)
   - Publishes goal poses to Nav2's NavigateToPose action

**The Exploration Loop**:
```
SLAM Toolbox publishes updated map
    ↓
Autonomous Explorer detects frontiers in map
    ↓
Autonomous Explorer picks closest frontier
    ↓
Autonomous Explorer sends goal to Nav2 NavigateToPose
    ↓
Nav2 plans path and drives robot to frontier
    ↓
Robot reaches frontier, lidar sees new area
    ↓
SLAM Toolbox updates map with new data
    ↓
[Loop repeats until no frontiers remain]
```

## Control & Monitoring

### Control Interface

**Approach**: TUI (Text User Interface) + Visualization

**Primary Control - TUI**:
- Terminal-based dashboard using Python `textual` library
- Live updating telemetry display
- Interactive keyboard controls
- Event log with recent activity
- Color-coded status indicators
- Runs via SSH (no GUI needed)

**TUI Layout**:
```
╔═══════════════════════════════════════════════════════════════╗
║                     DOME2 MISSION CONTROL                     ║
╠═══════════════════════════════════════════════════════════════╣
║ Mission State:  EXPLORING                    Uptime: 12:34    ║
║ Coverage:       68%                          Objects: 5       ║
║ Position:       (3.2, -1.5, 45°)                              ║
╠═══════════════════════════════════════════════════════════════╣
║ CONTROLS:                                                     ║
║   [S] Start Mission    [T] Stop & Return    [E] Emergency    ║
║   [P] Pause            [R] Resume           [Q] Quit TUI     ║
╠═══════════════════════════════════════════════════════════════╣
║ RECENT EVENTS:                                                ║
║  12:34:15  Frontier reached at (2.5, -3.1)                   ║
║  12:32:08  Object detected: bottle (0.89)                    ║
║  12:30:45  Navigation recovery: obstacle cleared             ║
╠═══════════════════════════════════════════════════════════════╣
║ SENSOR STATUS:  LIDAR: ✓  CAMERA: ✓  IMU: ✓  MOTORS: ✓      ║
╚═══════════════════════════════════════════════════════════════╝
```

**Visualization Tool**:
- Foxglove Studio (primary) or RViz2
- Browser-based access for remote monitoring
- View map, robot position, sensor data
- Runs on networked laptop/desktop

**Remote Access**:
- SSH to Pi 5 to launch TUI
- Foxglove/RViz2 for detailed visualization
- Log files for post-mission analysis

## Directory Structure

### Prerequisites

This autonomous exploration application assumes the base Dome2 hardware setup is already installed and working separately:
- Motor control and odometry publishing on `/cmd_vel` and `/odom`
- Lidar driver publishing on `/scan`
- Camera driver publishing on `/camera/rgb` and `/camera/depth`

The directory structure below contains only the autonomous exploration application code, not the base hardware layer.

### ROS2 Workspace Layout

```
~/dome2_ws/                              # ROS2 workspace root
├── src/
│   ├── dome2_explorer/                  # Autonomous exploration application
│   │   ├── dome2_bringup/               # Launch files and startup
│   │   │   ├── launch/
│   │   │   │   ├── exploration_full.launch.py # Complete exploration system
│   │   │   │   ├── slam.launch.py             # SLAM Toolbox only
│   │   │   │   ├── nav2.launch.py             # Nav2 stack only
│   │   │   │   └── exploration.launch.py      # Exploration only
│   │   │   ├── config/
│   │   │   │   ├── config.yaml                # Main configuration
│   │   │   │   ├── config_test.yaml           # Testing overrides
│   │   │   │   ├── slam_params.yaml           # SLAM Toolbox params
│   │   │   │   ├── nav2_params.yaml           # Nav2 params
│   │   │   │   └── robot_description.urdf     # Robot model
│   │   │   ├── rviz/
│   │   │   │   └── dome2.rviz                 # RViz configuration
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── dome2_msgs/                  # Custom message definitions
│   │   │   ├── msg/
│   │   │   │   ├── MissionState.msg           # Mission status
│   │   │   │   ├── ObjectDetection.msg        # Detected object
│   │   │   │   ├── ObjectCatalog.msg          # List of objects
│   │   │   │   └── ExplorationStatus.msg      # Exploration progress
│   │   │   ├── srv/
│   │   │   │   ├── StartMission.srv           # Start exploration
│   │   │   │   ├── StopMission.srv            # Stop and return
│   │   │   │   └── GetObjectCatalog.srv       # Retrieve detected objects
│   │   │   ├── action/
│   │   │   │   └── ExploreMission.action      # Full mission action
│   │   │   ├── package.xml
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── dome2_navigation/             # Navigation extensions
│   │   │   ├── dome2_navigation/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── stuck_detector.py          # Stuck detection logic
│   │   │   │   ├── recovery_manager.py        # Recovery coordination
│   │   │   │   └── nav_monitor.py             # Nav2 status monitoring
│   │   │   ├── test/
│   │   │   │   └── test_stuck_detector.py
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── README.md
│   │   │
│   │   ├── dome2_exploration/            # Exploration logic
│   │   │   ├── dome2_exploration/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── explorer_node.py           # Main exploration node
│   │   │   │   ├── frontier_detector.py       # Frontier identification
│   │   │   │   ├── frontier_selector.py       # Goal selection
│   │   │   │   └── exploration_manager.py     # State coordination
│   │   │   ├── test/
│   │   │   │   ├── test_frontier_detector.py
│   │   │   │   └── test_frontier_selector.py
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── README.md
│   │   │
│   │   ├── dome2_perception/             # Object detection
│   │   │   ├── dome2_perception/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── object_detector.py         # MobileNet-SSD detector
│   │   │   │   ├── object_catalog.py          # Object tracking
│   │   │   │   └── depth_processor.py         # 3D position estimation
│   │   │   ├── models/
│   │   │   │   └── mobilenet_ssd/             # Model files
│   │   │   ├── test/
│   │   │   │   └── test_object_detector.py
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── README.md
│   │   │
│   │   ├── dome2_mission/                # Mission control
│   │   │   ├── dome2_mission/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── mission_manager.py         # State machine
│   │   │   │   ├── return_to_home.py          # Return navigation
│   │   │   │   └── diagnostics.py             # System health monitoring
│   │   │   ├── test/
│   │   │   │   └── test_mission_manager.py
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── README.md
│   │   │
│   │   ├── dome2_tui/                    # Text User Interface
│   │   │   ├── dome2_tui/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── main_app.py                # Textual app
│   │   │   │   ├── widgets/
│   │   │   │   │   ├── status_panel.py        # Mission status display
│   │   │   │   │   ├── map_preview.py         # ASCII map view
│   │   │   │   │   ├── control_panel.py       # Control buttons
│   │   │   │   │   └── event_log.py           # Event history
│   │   │   │   └── ros_bridge.py              # ROS2 interface
│   │   │   ├── test/
│   │   │   │   └── test_tui.py
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── README.md
│   │   │
│   │   ├── dome2_utils/                  # Shared utilities
│   │   │   ├── dome2_utils/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── config_loader.py           # YAML config loading
│   │   │   │   ├── transforms.py              # TF utilities
│   │   │   │   ├── logging.py                 # Logging setup
│   │   │   │   └── math_utils.py              # Common calculations
│   │   │   ├── test/
│   │   │   │   └── test_config_loader.py
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── README.md
│   │   │
│   │   ├── docs/                         # Documentation
│   │   │   ├── README.md                      # Project overview
│   │   │   ├── ARCHITECTURE.md                # System architecture
│   │   │   ├── SETUP.md                       # Installation guide
│   │   │   ├── USAGE.md                       # User guide
│   │   │   ├── DEVELOPMENT.md                 # Developer guide
│   │   │   ├── TESTING.md                     # Testing strategy
│   │   │   └── TROUBLESHOOTING.md             # Common issues
│   │   │
│   │   ├── scripts/                      # Utility scripts
│   │   │   ├── install_dependencies.sh        # Dependency setup
│   │   │   ├── build_workspace.sh             # Colcon build
│   │   │   ├── start_mission.sh               # Launch wrapper
│   │   │   ├── calibrate_robot.sh             # Calibration helper
│   │   │   └── backup_config.sh               # Config backup
│   │   │
│   │   ├── tests/                        # Integration tests
│   │   │   ├── integration/
│   │   │   │   ├── test_full_mission.py       # End-to-end test
│   │   │   │   ├── test_slam_nav2.py          # SLAM + Nav2
│   │   │   │   └── test_exploration.py        # Exploration loop
│   │   │   └── fixtures/
│   │   │       ├── test_maps/                 # Sample maps
│   │   │       └── test_configs/              # Test configs
│   │   │
│   │   ├── .github/                      # GitHub workflows
│   │   │   └── workflows/
│   │   │       ├── build_test.yml             # CI/CD pipeline
│   │   │       └── lint.yml                   # Code quality
│   │   │
│   │   ├── .gitignore
│   │   ├── README.md
│   │   ├── LICENSE
│   │   └── requirements.txt              # Python dependencies
│   │
│   └── external/                         # Third-party packages
│       ├── slam_toolbox/                 # (apt install recommended)
│       ├── nav2/                         # (apt install recommended)
│       └── autonomous_explorer/          # Cloned from GitHub
│
├── build/                                # Build artifacts (gitignored)
├── install/                              # Install space (gitignored)
└── log/                                  # Build logs (gitignored)

# Runtime directories (outside workspace)
/var/log/dome2/                          # Runtime logs
    ├── missions/                         # Mission-specific logs
    │   ├── mission_20260122_143025/
    │   │   ├── mission.log
    │   │   ├── nav2.log
    │   │   ├── slam.log
    │   │   └── rosbag/
    │   │       └── mission.db3
    │   └── mission_20260122_154512/
    │       └── ...
    └── system.log                        # System-wide log

/home/dome2/data/                        # Persistent data
    ├── maps/                             # Saved maps
    │   ├── home_floor1_20260122.yaml
    │   └── home_floor1_20260122.pgm
    ├── object_detections/                # Detected objects
    │   ├── catalog_20260122.json
    │   └── images/
    │       ├── bottle_001.jpg
    │       └── chair_005.jpg
    └── configs/                          # Config backups
        └── config_20260122.yaml.bak

/tmp/dome2/                              # Temporary runtime data
    ├── current_mission_state.json
    └── diagnostics.json
```

### Package Responsibilities

**Note**: Hardware interface (motors, sensors, odometry) is provided by base Dome2 and not included in this application.

**dome2_bringup**: System startup and configuration
- Launch files for different system configurations
- Central configuration management
- Robot description (URDF)

**dome2_msgs**: Message/Service/Action definitions
- Custom messages for mission state, objects, exploration
- Service definitions for control commands
- Action definitions for long-running operations

**dome2_navigation**: Navigation extensions
- Stuck detection logic
- Recovery behavior coordination
- Nav2 monitoring and feedback

**dome2_exploration**: Autonomous exploration
- Frontier detection and clustering
- Frontier selection strategies
- Exploration state management
- Integration with Autonomous-Explorer package

**dome2_perception**: Object detection and tracking
- MobileNet-SSD detector on Oak-d-lite
- Object catalog management
- 3D position estimation using depth

**dome2_mission**: Mission control
- High-level state machine
- Return-to-home logic
- Mission completion detection
- System diagnostics

**dome2_tui**: User interface
- Terminal dashboard (textual library)
- Real-time telemetry display
- Mission control commands
- Event logging

**dome2_utils**: Shared utilities
- Configuration loading
- Transform utilities
- Logging setup
- Common math functions

### Key Files

**Configuration**:
- `config/config.yaml` - Single source of truth for all parameters
- `config/slam_params.yaml` - SLAM Toolbox specific parameters
- `config/nav2_params.yaml` - Nav2 specific parameters

**Launch**:
- `launch/exploration_full.launch.py` - Starts complete exploration system
- Individual component launches for testing (SLAM, Nav2, exploration separately)
- Assumes base Dome2 hardware nodes are already running

**Documentation**:
- `docs/ARCHITECTURE.md` - Matches this design document
- `docs/SETUP.md` - Installation and setup instructions
- `docs/USAGE.md` - How to run missions

**Scripts**:
- `scripts/start_mission.sh` - User-friendly mission launcher
- `scripts/install_dependencies.sh` - One-step dependency install

### Development Workflow Files

**.gitignore**:
```
# ROS2 build artifacts
build/
install/
log/

# Python
__pycache__/
*.pyc
*.egg-info/

# IDE
.vscode/
.idea/

# Runtime data
/tmp/dome2/
*.bag
*.db3

# Logs
*.log
```

**requirements.txt**:
```
textual>=0.40.0      # TUI library
pyyaml>=6.0          # Config loading
numpy>=1.24.0        # Math operations
opencv-python>=4.8.0 # Image processing
```

### File Organization Principles

**Separation of Concerns**:
- Each package has single, clear responsibility
- Hardware interface isolated from high-level logic
- Configuration separate from code

**Testing**:
- Unit tests alongside source files
- Integration tests in top-level tests/
- Test fixtures centralized

**Documentation**:
- README.md in each package
- Top-level docs/ for system-wide documentation
- Inline code comments for complex logic

**Configuration Management**:
- All parameters in config.yaml
- Environment-specific overrides (test, sim)
- Runtime parameter updates via ROS2 parameters

## Project Plan

### Overall Approach
Incremental development with continuous testing. Each phase is independently testable.

### Phase 1: Baseline Verification & Configuration
**Goal**: Verify base Dome2 system is working and establish baseline configuration

**Prerequisites**: Base Dome2 hardware setup complete with working sensors and motor control

**Tasks**:
- Verify ROS2 Jazzy installation and configuration
- Verify base Dome2 hardware topics (`/scan`, `/odom`, `/camera/*`, `/cmd_vel`) publishing correctly
- Test existing SLAM capability (if available, or install SLAM Toolbox)
- Test existing navigation capability (if available, or install Nav2)
- Document current system state and topic structure
- Create initial config.yaml for exploration application
- Set up development environment (Foxglove, git repo for exploration app)

**Success Criteria**:
- [ ] Base Dome2 sensors publishing on expected topics
- [ ] Robot responds to `/cmd_vel` commands
- [ ] Can teleoperate robot and see sensor data
- [ ] Git repository created for dome2_explorer application
- [ ] Development tools (Foxglove, git) set up

**Duration**: 1 week

### Phase 2: SLAM & Navigation Stack
**Goal**: Robot navigates autonomously to goals while avoiding obstacles

**Tasks**:
- Configure SLAM Toolbox for 2D mapping
- Set up Nav2 navigation stack
- Tune costmap parameters (inflation, obstacle marking)
- Configure path planner
- Tune controller parameters for differential drive
- Test navigation in controlled environment

**Success Criteria**:
- [ ] SLAM builds accurate map while teleoperating
- [ ] Can set goal in Foxglove and robot navigates there
- [ ] Robot avoids static obstacles reliably
- [ ] Localization remains stable during navigation
- [ ] Robot navigates through doorways

**Duration**: 2-3 weeks

### Phase 3: Autonomous Exploration
**Goal**: Robot explores autonomously without manual goals

**Tasks**:
- Integrate Autonomous-Explorer-and-Mapper-ros2-nav2 package
- Configure exploration parameters (frontier detection, goal selection)
- Implement mission manager state machine
- Implement return-to-home behavior
- Add exploration completion detection
- Test full exploration run in test environment

**Success Criteria**:
- [ ] Robot autonomously explores unknown space
- [ ] Frontiers identified and prioritized correctly
- [ ] Exploration terminates when complete
- [ ] Robot returns to starting position
- [ ] Map saved successfully after mission

**Duration**: 2 weeks

### Phase 4: Object Detection & Cataloging
**Goal**: Detect and catalog objects during exploration

**Tasks**:
- Set up MobileNet-SSD on Oak-d-lite
- Implement object detection ROS2 node
- Create object catalog data structure
- Integrate detection with mission manager
- Add object position calculation (map coordinates)
- Store detected objects with timestamps and images

**Success Criteria**:
- [ ] Objects detected in real-time during exploration
- [ ] Object positions accurately recorded on map
- [ ] Object catalog saved with map data
- [ ] False positive rate acceptable

**Duration**: 1-2 weeks

### Phase 5: Mission Control & Monitoring
**Goal**: Complete user interface for mission control

**Tasks**:
- Implement TUI with real-time telemetry display
- Add mission control commands (start/stop/emergency)
- Implement diagnostics monitoring
- Add event logging system
- Create data export functionality
- Test full mission workflow end-to-end

**Success Criteria**:
- [ ] TUI provides clear mission status
- [ ] All control commands work reliably
- [ ] Mission data easily accessible post-mission
- [ ] User can run full mission without technical intervention

**Duration**: 1 week

### Testing Strategy

**Unit Testing**:
- Individual node functionality (motors, sensors, detection)
- State machine transitions
- Data structure operations

**Integration Testing**:
- SLAM + Navigation together
- Exploration + Object Detection
- Full system with all components

**Field Testing**:
- Multiple rooms with varied layouts
- Different lighting conditions
- Wood/carpet transitions
- Doorway navigation scenarios
- Recovery behavior validation

### Phase Dependencies
1. Phase 1 must complete before Phase 2 (need working sensors for SLAM)
2. Phase 2 must complete before Phase 3 (need navigation for exploration)
3. Phase 3 and Phase 4 can partially overlap (exploration works without object detection)
4. Phase 5 can be developed incrementally alongside other phases

## Configuration Management

### Configuration Philosophy
Centralize all tunable parameters in a single `config.yaml` file for easy adjustment without code changes. This supports iterative tuning and field deployment.

### Proposed config.yaml Structure

```yaml
# Dome2 Robot Configuration
# All customizable parameters for autonomous exploration

robot:
  name: "dome2"
  base_frame: "base_link"
  odom_frame: "odom"
  map_frame: "map"

# Hardware Configuration
hardware:
  differential_drive:
    wheel_separation: 0.28  # meters
    wheel_radius: 0.065     # meters
    max_linear_velocity: 0.5   # m/s (conservative for reliability)
    max_angular_velocity: 1.0  # rad/s
    acceleration_limit: 0.3    # m/s^2

  lidar:
    topic: "/scan"
    frame_id: "lidar_link"
    min_range: 0.15      # meters
    max_range: 12.0      # meters

  camera:
    topic_rgb: "/camera/rgb"
    topic_depth: "/camera/depth"
    frame_id: "camera_link"
    fps: 15              # frames per second
    resolution_width: 640
    resolution_height: 480

# SLAM Toolbox Configuration
slam:
  mode: "async"          # async or sync
  resolution: 0.05       # meters per cell (5cm resolution)
  update_rate: 5.0       # Hz

  # Loop closure
  loop_closure_enabled: true
  min_loop_closure_distance: 3.0  # meters
  loop_search_maximum_distance: 5.0

  # Scan matching
  scan_buffer_size: 10
  minimum_travel_distance: 0.2    # meters before updating
  minimum_travel_heading: 0.2     # radians before updating

  # Map management
  map_update_interval: 1.0        # seconds

# Nav2 Configuration
navigation:
  # Controller parameters
  controller:
    type: "dwb"          # DWB local planner
    max_vel_x: 0.4       # m/s (conservative)
    min_vel_x: -0.2      # m/s (backing up)
    max_vel_theta: 0.8   # rad/s
    min_speed_xy: 0.1    # m/s (minimum to maintain)
    acc_lim_x: 0.3       # m/s^2
    acc_lim_theta: 1.0   # rad/s^2

  # Planner parameters
  planner:
    type: "NavFn"        # or "SmacPlanner"
    tolerance: 0.2       # meters to goal
    use_astar: false     # Dijkstra for reliability

  # Costmap parameters
  costmap:
    global:
      width: 50.0           # meters
      height: 50.0          # meters
      resolution: 0.05      # meters per cell
      robot_radius: 0.20    # meters (conservative)
      inflation_radius: 0.4 # meters
      cost_scaling_factor: 3.0

    local:
      width: 5.0
      height: 5.0
      resolution: 0.05
      robot_radius: 0.20
      inflation_radius: 0.3

    obstacle_layer:
      enabled: true
      max_obstacle_height: 2.0
      combination_method: 1  # Max

  # Recovery behaviors
  recovery:
    enabled: true
    max_retries: 3
    retry_timeout: 30.0        # seconds
    behaviors:
      - rotate_recovery
      - back_up_recovery
      - wait_recovery

# Exploration Configuration
exploration:
  update_frequency: 5.0          # seconds (timer interval)

  # Frontier detection
  frontier:
    min_size: 5                  # minimum frontier cells
    min_distance: 0.5            # meters (filter nearby frontiers)
    cluster_tolerance: 0.3       # meters (group nearby cells)

  # Frontier selection
  selection:
    strategy: "closest"          # closest, largest, or weighted
    distance_weight: 1.0         # for weighted strategy
    size_weight: 0.5             # for weighted strategy

  # Safety
  max_goal_distance: 10.0        # meters (don't go too far)
  blacklist_timeout: 60.0        # seconds (retry failed frontiers)
  max_blacklist_size: 20         # max unreachable frontiers

  # Completion criteria
  completion:
    no_frontiers_timeout: 10.0   # seconds to confirm done
    min_frontier_threshold: 3    # ignore tiny frontiers at end

# Object Detection Configuration
object_detection:
  enabled: true
  model: "mobilenet_ssd"
  confidence_threshold: 0.7      # 0.0-1.0
  detection_rate: 2.0            # Hz (detections per second)

  # Classes to detect (COCO dataset)
  target_classes:
    - "bottle"
    - "cup"
    - "bowl"
    - "chair"
    - "couch"

  # Catalog settings
  catalog:
    min_distance_between: 0.5    # meters (deduplicate nearby detections)
    max_catalog_size: 100        # maximum objects to track
    save_images: true            # capture image of detected objects

# Mission Configuration
mission:
  # Timing
  max_duration: 1800             # seconds (30 minutes)
  safety_margin_duration: 300    # seconds (5 min buffer)

  # Starting position
  home_position:
    x: 0.0
    y: 0.0
    theta: 0.0

  # Return to home
  return_home:
    enabled: true
    distance_tolerance: 0.3      # meters
    angular_tolerance: 0.2       # radians

  # Failure handling
  failure:
    localization_loss_timeout: 5.0    # seconds
    stuck_timeout: 30.0                # seconds
    max_recovery_attempts: 3

# Monitoring & Diagnostics
monitoring:
  # Status publishing
  status_rate: 1.0               # Hz

  # Health checks
  health_check_interval: 5.0     # seconds

  # Thresholds
  thresholds:
    min_battery_voltage: 11.0    # volts (if monitoring)
    max_cpu_usage: 85.0          # percent
    max_memory_usage: 80.0       # percent
    min_lidar_points: 100        # per scan

  # Logging
  logging:
    level: "INFO"                # DEBUG, INFO, WARN, ERROR
    log_to_file: true
    log_directory: "/var/log/dome2"
    max_log_size_mb: 100

# TUI Configuration
tui:
  refresh_rate: 2.0              # Hz
  show_map_preview: false        # ASCII map in TUI (resource intensive)
  color_enabled: true

  # Display options
  display:
    position_precision: 2        # decimal places
    percentage_precision: 1      # decimal places

# Development & Testing
development:
  simulation_mode: false         # use simulated sensors
  debug_visualization: false     # extra RViz markers
  record_rosbag: false           # auto-record missions
  rosbag_topics:
    - "/scan"
    - "/map"
    - "/odom"
    - "/camera/rgb"
```

### Configuration Usage

**Loading Configuration**:
```python
import yaml
from pathlib import Path

def load_config():
    config_path = Path(__file__).parent / 'config.yaml'
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

config = load_config()
max_velocity = config['navigation']['controller']['max_vel_x']
```

**Environment-Specific Overrides**:
- `config.yaml` - Default production values
- `config_test.yaml` - Testing environment (faster rates, smaller areas)
- `config_sim.yaml` - Simulation parameters (different sensor models)

**Runtime Parameter Updates**:
Use ROS2 parameters for values that might change during operation:
```bash
ros2 param set /explorer exploration.update_frequency 2.0
```

### Configuration Validation

**Pre-flight Checks**:
- Validate all required parameters present
- Check value ranges (e.g., velocities > 0)
- Verify topic names don't conflict
- Ensure file paths exist and are writable

**Configuration Schema** (optional):
Use JSON Schema or similar to validate config.yaml structure before loading.

## Open Issues & Decisions

### Software
- [ ] **Path planner**: NavFn (simple, fast) vs Smac Planner (more sophisticated)

### Configuration
- [ ] **SLAM parameters**: Resolution, update rates, loop closure settings
- [ ] **Nav2 tuning**: Costmap inflation radius, max velocity, acceleration limits
- [ ] **Exploration parameters**: Frontier size threshold, goal timeout, blacklist distance
- [ ] **Object detection**: Confidence threshold, detection rate, classes to detect

### Development Environment
- [ ] **Simulation**: Use Gazebo for testing before hardware ready?
- [ ] **Development workflow**: Develop on robot vs remote computer
- [ ] **Testing environment**: Build test area with furniture/obstacles?

### Future Enhancements (Post-MVP)
- [ ] AprilTag dock for improved return-to-home
- [ ] Battery monitoring and low-power return behavior
- [ ] Multi-floor exploration capability
- [ ] More sophisticated object detection (custom trained models)
- [ ] Web dashboard as alternative to TUI
- [ ] Mission replay and analysis tools
