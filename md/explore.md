# Explore: Semantic Mapping Stack for Dome2

## Robot Platform

The Explore project builds upon an existing Dome2 robot that provides core ROS2 functionality. The Dome2 platform features a differential drive chassis powered by a Raspberry Pi 5, equipped with an Oak-d-lite camera (providing both RGB and depth sensing), and a lidar sensor. The entire system runs on ROS2 Jazzy.

The Dome2 base platform handles motion control and odometry, IMU sensor integration, camera and lidar drivers, basic sensor data publishing, and teleoperation. The Explore stack adds a semantic mapping layer on top of this foundation, implementing autonomous exploration and object detection/cataloging capabilities.

## Mission Specification

### Primary Mission

The robot will autonomously explore and map an unknown indoor environment while identifying and cataloging objects of interest.

### Mission Objectives

The mission has three core objectives: **Autonomous Exploration** to navigate and avoid obstacles autonomously; **Mapping & Localization** to build a 2D occupancy map with accurate localization; and **Object Detection & Cataloging** to identify and record object locations on the map.

### Success Criteria

Mission success requires the robot to navigate autonomously without collision, generate a usable map of the explored space, detect and identify target objects with greater than 80% accuracy, and log object positions on the map.

### Environment & Constraints

The target environment is one floor of a multi-room home, typically 1000-2500 square feet, with wood and carpet flooring. Standard doorways are assumed to be open, while stairs are treated as boundaries and marked as no-go zones. Adequate lighting for vision systems is required. Mission duration is expected to be 15-30 minutes, with battery runtime providing a minimum of 45-60 minutes of operation.

### Design Priority: Reliability First

The core principle is autonomous operation without human intervention. It's better to explore 60% reliably than 100% with failures. The system employs conservative navigation with wider safety margins, stopping safely when uncertain rather than risking collision, and logging issues for post-mission review.

### Mission Behavior

**Starting Position:** The robot uses ad-hoc placement without requiring a dock initially. It can be placed anywhere and starts at pose (0,0,0), with SLAM building the map from the starting position. An AprilTag dock can optionally be added later for improved return-to-home reliability.

**Mission Completion:** Upon completion, the robot returns to the home position (starting location) and saves the map and object catalog.

**Exploration Completion Criteria:** The mission is considered "done" when any of the following occurs:
- **No accessible frontiers** - All frontiers explored or unreachable
- **Time limit reached** - Maximum 30-40 minutes
- **Manual stop** - User terminates mission
- **Frontier size threshold** - Only tiny frontiers remain (< min size)
- **Failure condition** - Localization loss or critical sensor failure

**Failure Modes:** In case of localization loss, the system declares failure and stops safely in place. If stuck, it attempts recovery (back up, rotate, try alternate path); if recovery fails, it stops and logs the condition. For sensor failures, the robot continues if possible but aborts if critical sensors fail. Low battery management is not currently handled, as sufficient runtime is assumed.

### Stuck Detection Strategy

The robot is considered "stuck" when it cannot make progress toward its navigation goal despite being commanded to move. Multiple indicators are used together for reliable detection.

**Detection Methods:**

**Nav2 Action Status Monitoring:** The system monitors the NavigateToPose action feedback. When the action status returns STATUS_ABORTED, Nav2 has given up on the goal, indicating path planning failed or the goal is unreachable. This is the most reliable indicator.

**Progress Tracking:** The system checks if the robot position changed in the last N seconds. If the distance between current and last position is below a threshold and time since last progress exceeds the stuck timeout (e.g., 30 seconds), the robot is considered stuck. This catches cases where Nav2 is still trying but the robot is physically blocked.

**Velocity Feedback Mismatch:** By comparing commanded versus actual velocity, the system can detect when the robot is commanded to move but isn't moving. If this mismatch persists beyond a timeout (e.g., 5 seconds), it indicates physical obstruction such as being caught on carpet or wedged against furniture. This requires odometry feedback.

**Recovery Behavior Counter:** The system tracks how many times Nav2 enters recovery mode. Repeated recovery attempts indicate a persistent problem. After exceeding max_recovery_attempts (e.g., 3), the robot is considered stuck.

**Goal Timeout:** An overall timeout for reaching any single frontier prevents infinite attempts and catches slow progress or wandering behavior.

**Combined Detection Logic:** The system uses multiple indicators for reliability. The robot is stuck if any critical indicator triggers: Nav2 failed, no progress detected, too many recoveries attempted, or goal timeout exceeded.

**Recovery Actions When Stuck:**

First attempt: Let Nav2's built-in recovery behaviors handle it (rotate_recovery spins in place to clear costmap, back_up_recovery backs away from obstacles, wait_recovery waits for dynamic obstacles to move).

Second attempt: Blacklist this frontier for 60 seconds and try a different goal to continue exploration.

Third attempt (if stuck again): Permanently blacklist the frontier, log the location for debugging, and continue with remaining frontiers.

If repeatedly stuck everywhere: Declare mission failure, as no progress is possible. Stop safely and log diagnostics while waiting for human intervention.

**Key Configuration Parameters:**
- `mission.failure.stuck_timeout`: 30.0 seconds of no movement
- `mission.failure.max_recovery_attempts`: 3 before giving up on goal
- `navigation.recovery.retry_timeout`: 30.0 Nav2 recovery timeout
- `navigation.recovery.max_retries`: 3 Nav2 recovery attempts
- `exploration.blacklist_timeout`: 60.0 seconds to retry blacklisted frontier
- `exploration.max_blacklist_size`: 20 maximum unreachable frontiers

**Telemetry Logged:** Timestamp of stuck detection, robot position when stuck, goal frontier coordinates, number of recovery attempts, sensor data (lidar scan, camera image), and map state at time of stuck event.

**Prevention Strategies:** Conservative costmap inflation provides wider safety margins around obstacles. Lower velocities give more time to react. Frontier filtering avoids selecting frontiers too close to obstacles. Maximum goal distance prevents attempting very distant goals.

## Software Architecture

### Overview

The system employs a ROS2-based modular architecture with the standard navigation stack (Nav2) and custom mission control.

### Core Components

**Prerequisites** provided by the Dome2 base platform include the hardware interface layer (motor control, sensor drivers for lidar, camera, and IMU, and odometry publishing). Base topics `/scan`, `/odom`, `/camera/rgb`, `/camera/depth`, `/cmd_vel`, and `/imu` are already available. The Explore semantic mapping stack builds on top of the working Dome2 platform.

**Perception Layer:** The SLAM Node uses SLAM Toolbox for map building and localization. The Object Detection Node runs MobileNet-SSD on the Oak-d-lite for object identification. Obstacle Detection combines lidar and depth data for obstacle avoidance.

**Navigation Layer:** The Nav2 Stack provides path planning, trajectory control, and behavior trees. The Costmap combines static maps with dynamic obstacles. Recovery Behaviors handle stuck detection and recovery routines.

**Mission Control Layer:** The Mission Manager Node implements a high-level state machine (exploring, returning home, failed). The Exploration Node uses the Autonomous Explorer package for frontier-based exploration. The Object Catalog maintains detected objects with positions.

**Monitoring & Logging:** The Diagnostics system monitors system health, while the Data Logger captures mission telemetry for post-analysis.

### Key Data Flows

The system flow works as follows: Lidar feeds SLAM Toolbox which generates the Map, which Nav2 uses to send commands to Motor Control. The Camera feeds Object Detection which populates the Object Catalog. The Mission Manager sends Nav2 Goals to Navigation. The Exploration Node performs Map Analysis and sends Nav2 Goals. All Nodes feed Diagnostics which sends data to the Logger.

### ROS2 Topics

**Prerequisites** provided by Dome2:
- `/scan` - Lidar data (sensor_msgs/LaserScan)
- `/camera/rgb` - RGB image (sensor_msgs/Image)
- `/camera/depth` - Depth image (sensor_msgs/Image)
- `/odom` - Odometry (nav_msgs/Odometry)
- `/imu` - IMU data (sensor_msgs/Imu)
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist) - consumed by motors

**Provided by Explore:**
- `/map` - Occupancy grid from SLAM Toolbox (nav_msgs/OccupancyGrid)
- `/goal_pose` - Navigation goals (geometry_msgs/PoseStamped)
- `/detected_objects` - Object detections (custom message)
- `/mission_state` - Current mission status (custom message)
- `/exploration_status` - Exploration progress (custom message)

### Technology Stack

**Confirmed Decisions:**
- **SLAM**: SLAM Toolbox (async mode)
- **Navigation**: Nav2 stack
- **Exploration**: Autonomous-Explorer-and-Mapper-ros2-nav2
- **Object Detection**: MobileNet-SSD (Luxonis model zoo, runs on Oak-d-lite Myriad X)
- **Control Interface**: TUI (textual library) + Visualization
- **Mission Logic**: Simple state machine (not behavior tree)
- **ROS2 Distribution**: Jazzy

### Control Logic Architecture

**Behavior Trees** are used internally by Nav2 only. Nav2 uses BehaviorTree.CPP for navigation logic with pre-configured behavior trees that handle path planning, following, and recovery. These are used as-is without requiring custom behavior trees, providing robust navigation behaviors automatically.

**State Machine** is used for mission control and exploration. Simple procedural logic manages mission states (IDLE, EXPLORING, RETURNING_HOME, FAILED). Exploration uses timer-based sequential logic (find frontiers → pick → navigate). This approach is easier to understand, debug, and maintain, matching the reliability-first philosophy.

**Why not behavior trees for mission control?** The mission logic is straightforward with a linear flow: explore → return → done. A state machine is simpler and more transparent. Less complexity means fewer failure modes. Behavior trees would add overhead without clear benefit for this use case.

**Architecture layering:**
```
Mission Manager (State Machine)                    ← Explore stack
    ↓
Exploration Node (Sequential Logic)                ← Explore stack
    ↓
Nav2 (Behavior Trees - internal)                   ← Explore stack
    ↓
Dome2 Platform (Motion, Odometry, IMU, Camera)     ← Base platform
```

### Exploration Package Decision

**Selected: Autonomous-Explorer-and-Mapper-ros2-nav2**

This package was specifically designed for SLAM Toolbox + Nav2 + ROS2 Humble/Jazzy. It handles dynamic map updates from SLAM Toolbox correctly, implements frontier-based exploration with dynamic goal selection, includes built-in recovery behaviors for localization failures, and provides a lightweight, focused implementation. Source: https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2

**Why not m-explore-ros2:** This package has a known incompatibility with SLAM Toolbox (Issue #10 since Dec 2021). It was designed for gmapping and doesn't handle SLAM Toolbox's dynamic map growth. The map merging algorithm breaks with SLAM Toolbox. Source: https://github.com/robo-friends/m-explore-ros2/issues/10

**Alternative Options:** Custom implementation (~200-300 lines Python) or other community packages (ROS2-FrontierBaseExplorationForAutonomousRobot, AutoFrontierSearch_ros2-humble).

### Deployment & Development Workflow

**Installation Approach:** Native ROS2 installation is used. ROS2 Jazzy is installed directly on the robot (Raspberry Pi 5) without Docker containers, providing simpler setup and better performance on the embedded platform. Standard apt package management handles ROS2 packages.

**Source Control:** Git and GitHub are used for version control. Git manages all robot code, GitHub serves as the remote repository, and updates are deployed by pulling from GitHub on the robot.

**Update Process:**
```bash
# On robot:
cd ~/ros2_ws/src/explore
git pull origin main
cd ~/ros2_ws
colcon build --packages-select explore_*
source install/setup.bash
# Restart relevant nodes
```

**Development Workflow:** Development and testing occur on a development machine or directly on the robot. Changes are committed and pushed to GitHub, then pulled on the robot for deployment. Branches are used for experimental features.

### How Frontier Exploration Works

**Component Integration:**

SLAM Toolbox runs continuously, subscribing to `/scan` and `/odom`, publishing `/map` (occupancy grid with free/occupied/unknown cells), and providing localization services.

Nav2 (the navigation stack) uses `/map` from SLAM Toolbox, plans paths through free space, and drives the robot to goal poses.

Autonomous Explorer handles frontier detection by subscribing to `/map` from SLAM Toolbox, analyzing the map to find frontiers (edges between known free space and unknown), scoring/ranking frontiers (preferring closest frontier for efficiency), and publishing goal poses to Nav2's NavigateToPose action.

**The Exploration Loop:**

SLAM Toolbox publishes an updated map → Autonomous Explorer detects frontiers in the map → Autonomous Explorer picks the closest frontier → Autonomous Explorer sends goal to Nav2 NavigateToPose → Nav2 plans path and drives robot to frontier → Robot reaches frontier, lidar sees new area → SLAM Toolbox updates map with new data → Loop repeats until no frontiers remain.

## Control & Monitoring

### Control Interface

**Approach:** The system uses a TUI (Text User Interface) combined with visualization tools.

**Primary Control - TUI:** A terminal-based dashboard using Python's `textual` library provides live updating telemetry display, interactive keyboard controls, an event log with recent activity, and color-coded status indicators. It runs via SSH, requiring no GUI.

**TUI Layout:**
```
╔═══════════════════════════════════════════════════════════════╗
║                     EXPLORE MISSION CONTROL                   ║
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

**Visualization Tool:** Foxglove Studio (primary) or RViz2 provides browser-based access for remote monitoring. Users can view the map, robot position, and sensor data from a networked laptop or desktop.

**Remote Access:** SSH to the Pi 5 launches the TUI. Foxglove/RViz2 provides detailed visualization. Log files enable post-mission analysis.

## Directory Structure

### Prerequisites

This semantic mapping application (Explore) assumes the Dome2 base platform is already installed and working. The Dome2 platform provides motion control and odometry publishing on `/cmd_vel` and `/odom`, lidar driver publishing on `/scan`, camera driver publishing on `/camera/rgb` and `/camera/depth`, and IMU publishing on `/imu`.

The directory structure below contains only the Explore semantic mapping stack, not the Dome2 base platform.

### ROS2 Workspace Layout

```
~/ros2_ws/                               # ROS2 workspace root
├── src/
│   ├── dome2/                           # (Prerequisite) Base robot platform
│   │   └── [motion, odometry, IMU, camera drivers - not shown]
│   │
│   ├── explore/                         # Semantic mapping application stack
│   │   ├── explore_bringup/             # Launch files and startup
│   │   │   ├── launch/
│   │   │   │   ├── exploration_full.launch.py # Complete exploration system
│   │   │   │   ├── slam.launch.py             # SLAM Toolbox only
│   │   │   │   ├── nav2.launch.py             # Nav2 stack only
│   │   │   │   └── exploration.launch.py      # Exploration only
│   │   │   ├── config/
│   │   │   │   ├── config.yaml                # Unified config (overrides Nav2/SLAM defaults)
│   │   │   │   ├── config_test.yaml           # Testing overrides
│   │   │   │   └── robot_description.urdf     # Robot model
│   │   │   ├── rviz/
│   │   │   │   └── explore.rviz               # RViz configuration
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── explore_msgs/                # Custom message definitions
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
│   │   ├── explore_navigation/           # Navigation extensions
│   │   │   ├── explore_navigation/
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
│   │   ├── explore_exploration/          # Exploration logic
│   │   │   ├── explore_exploration/
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
│   │   ├── explore_perception/           # Object detection
│   │   │   ├── explore_perception/
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
│   │   ├── explore_mission/              # Mission control
│   │   │   ├── explore_mission/
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
│   │   ├── explore_tui/                  # Text User Interface
│   │   │   ├── explore_tui/
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
│   │   ├── explore_utils/                # Shared utilities
│   │   │   ├── explore_utils/
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
/var/log/explore/                        # Runtime logs
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

~/data/explore/                          # Persistent data
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

/tmp/explore/                            # Temporary runtime data
    ├── current_mission_state.json
    └── diagnostics.json
```

### Package Responsibilities

Note that the hardware interface (motors, sensors, odometry, IMU, camera) is provided by the Dome2 base platform and not included in the Explore stack.

**explore_bringup** handles system startup and configuration. It contains launch files for different system configurations, manages central configuration, and stores the robot description (URDF).

**explore_msgs** defines custom messages for mission state, objects, and exploration; service definitions for control commands; and action definitions for long-running operations.

**explore_navigation** provides navigation extensions including stuck detection logic, recovery behavior coordination, and Nav2 monitoring and feedback.

**explore_exploration** implements autonomous exploration with frontier detection and clustering, frontier selection strategies, exploration state management, and integration with the Autonomous-Explorer package.

**explore_perception** handles object detection and tracking using the MobileNet-SSD detector on Oak-d-lite, object catalog management, and 3D position estimation using depth data.

**explore_mission** manages mission control with a high-level state machine, return-to-home logic, mission completion detection, and system diagnostics.

**explore_tui** provides the user interface via a terminal dashboard (textual library), real-time telemetry display, mission control commands, and event logging.

**explore_utils** contains shared utilities for configuration loading, transform utilities, logging setup, and common math functions.

### Key Files

**Configuration:** The `config/config.yaml` file serves as the single source of truth for all parameter overrides. It uses ROS2 node namespacing format (`ros__parameters`) so it can be passed directly to Nav2, SLAM Toolbox, and Explore nodes. Default parameters from Nav2 and SLAM Toolbox packages are used as the base, with `config.yaml` overriding only values that need customization.

**Launch:** The `launch/exploration_full.launch.py` file starts the complete Explore semantic mapping system. Individual component launches are available for testing SLAM, Nav2, and exploration separately. These assume Dome2 base platform nodes are already running.

**Documentation:** The `docs/ARCHITECTURE.md` file matches this design document. Installation and setup instructions are in `docs/SETUP.md`, while `docs/USAGE.md` explains how to run missions.

**Scripts:** The `scripts/start_mission.sh` script provides a user-friendly mission launcher, while `scripts/install_dependencies.sh` enables one-step dependency installation.

### Development Workflow Files

**.gitignore:**
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
/tmp/explore/
*.bag
*.db3

# Logs
*.log
```

**requirements.txt:**
```
textual>=0.40.0      # TUI library
pyyaml>=6.0          # Config loading
numpy>=1.24.0        # Math operations
opencv-python>=4.8.0 # Image processing
```

### File Organization Principles

**Separation of Concerns:** Each package has a single, clear responsibility. The hardware interface is isolated from high-level logic, and configuration is separate from code.

**Testing:** Unit tests reside alongside source files. Integration tests are in the top-level tests/ directory. Test fixtures are centralized.

**Documentation:** Each package includes a README.md. Top-level docs/ contains system-wide documentation. Inline code comments explain complex logic.

**Configuration Management:** All parameters are in config.yaml. Environment-specific overrides handle test and simulation scenarios. Runtime parameter updates use ROS2 parameters.

## Project Plan

### Overall Approach

Development follows an incremental approach with continuous testing. Each phase is independently testable.

### Phase 1: Baseline Verification & Configuration

**Goal:** Verify Dome2 base platform is working and establish baseline configuration for Explore stack.

**Prerequisites:** Dome2 base platform complete with working sensors and motor control.

**Tasks:**
- Verify ROS2 Jazzy installation and configuration
- Verify Dome2 hardware topics (`/scan`, `/odom`, `/imu`, `/camera/*`, `/cmd_vel`) publishing correctly
- Test Dome2 teleoperation and sensor data
- Install SLAM Toolbox and Nav2 for Explore stack
- Document current Dome2 system state and topic structure
- Create initial config.yaml for Explore semantic mapping application
- Set up development environment (Foxglove, git repo for Explore)

**Success Criteria:**
- Dome2 sensors publishing on expected topics
- Robot responds to `/cmd_vel` commands via Dome2
- Can teleoperate robot and see sensor data
- Git repository created for Explore semantic mapping stack
- Development tools (Foxglove, git) set up

**Duration:** 1 week

### Phase 2: SLAM & Navigation Stack

**Goal:** Robot navigates autonomously to goals while avoiding obstacles.

**Tasks:**
- Configure SLAM Toolbox for 2D mapping
- Set up Nav2 navigation stack
- Tune costmap parameters (inflation, obstacle marking)
- Configure path planner
- Tune controller parameters for differential drive
- Test navigation in controlled environment

**Success Criteria:**
- SLAM builds accurate map while teleoperating
- Can set goal in Foxglove and robot navigates there
- Robot avoids static obstacles reliably
- Localization remains stable during navigation
- Robot navigates through doorways

**Duration:** 2-3 weeks

### Phase 3: Autonomous Exploration

**Goal:** Robot explores autonomously without manual goals.

**Tasks:**
- Integrate Autonomous-Explorer-and-Mapper-ros2-nav2 package
- Configure exploration parameters (frontier detection, goal selection)
- Implement mission manager state machine
- Implement return-to-home behavior
- Add exploration completion detection
- Test full exploration run in test environment

**Success Criteria:**
- Robot autonomously explores unknown space
- Frontiers identified and prioritized correctly
- Exploration terminates when complete
- Robot returns to starting position
- Map saved successfully after mission

**Duration:** 2 weeks

### Phase 4: Object Detection & Cataloging

**Goal:** Detect and catalog objects during exploration.

**Tasks:**
- Set up MobileNet-SSD on Oak-d-lite
- Implement object detection ROS2 node
- Create object catalog data structure
- Integrate detection with mission manager
- Add object position calculation (map coordinates)
- Store detected objects with timestamps and images

**Success Criteria:**
- Objects detected in real-time during exploration
- Object positions accurately recorded on map
- Object catalog saved with map data
- False positive rate acceptable

**Duration:** 1-2 weeks

### Phase 5: Mission Control & Monitoring

**Goal:** Complete user interface for mission control.

**Tasks:**
- Implement TUI with real-time telemetry display
- Add mission control commands (start/stop/emergency)
- Implement diagnostics monitoring
- Add event logging system
- Create data export functionality
- Test full mission workflow end-to-end

**Success Criteria:**
- TUI provides clear mission status
- All control commands work reliably
- Mission data easily accessible post-mission
- User can run full mission without technical intervention

**Duration:** 1 week

### Testing Strategy

**Unit Testing** covers individual node functionality (motors, sensors, detection), state machine transitions, and data structure operations.

**Integration Testing** validates SLAM + Navigation together, Exploration + Object Detection, and the full system with all components.

**Field Testing** includes scenarios with multiple rooms with varied layouts, different lighting conditions, wood/carpet transitions, doorway navigation scenarios, and recovery behavior validation.

### Phase Dependencies

Phase 1 must complete before Phase 2 (need working sensors for SLAM). Phase 2 must complete before Phase 3 (need navigation for exploration). Phase 3 and Phase 4 can partially overlap (exploration works without object detection). Phase 5 can be developed incrementally alongside other phases.

## Configuration Management

### Configuration Philosophy

A single unified `config.yaml` file contains all tunable parameters for the Explore stack. This file uses ROS2's native node-namespaced parameter format (`ros__parameters`), which allows it to serve dual purposes:

1. **Override file for external packages**: Nav2 and SLAM Toolbox ship with comprehensive default parameter files. Our `config.yaml` is passed as a second parameter file that patches/overrides only the values we need to customize.

2. **Configuration for Explore nodes**: Custom Explore nodes (mission manager, explorer, object detector, etc.) read their parameters from the same file.

This approach provides a single source of truth for all customizations while leveraging well-tested defaults from Nav2 and SLAM Toolbox.

### How Parameter Override Works

In ROS2, when multiple parameter files are passed to a node, later files override earlier ones. Launch files use this pattern:

```python
# In launch file
Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    parameters=[
        default_slam_params_path,  # Package defaults (comprehensive)
        config_yaml_path           # Our overrides (sparse)
    ]
)

# Nav2 bringup accepts params_file argument
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([nav2_bringup_dir, '/bringup_launch.py']),
    launch_arguments={
        'params_file': config_yaml_path,  # Overrides Nav2 defaults
        'use_sim_time': 'false'
    }.items()
)
```

### Proposed config.yaml Structure

The file uses ROS2 node namespacing. Each top-level key is a node name, and parameters are nested under `ros__parameters`. Only values that differ from defaults need to be specified.

```yaml
# Explore Robot Configuration
# Unified parameter overrides for Nav2, SLAM Toolbox, and Explore nodes
# Uses ROS2 node namespacing format for direct parameter loading

# =============================================================================
# SLAM Toolbox Overrides
# Default params from: slam_toolbox/config/mapper_params_online_async.yaml
# =============================================================================
slam_toolbox:
  ros__parameters:
    # Map resolution (default is 0.05)
    resolution: 0.05

    # Scan matching - tune for reliability
    minimum_travel_distance: 0.2      # meters before updating
    minimum_travel_heading: 0.2       # radians before updating

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 5.0

    # Frame configuration
    base_frame: base_link
    odom_frame: odom
    map_frame: map

# =============================================================================
# Nav2 Controller Server Overrides
# Default params from: nav2_bringup/params/nav2_params.yaml
# =============================================================================
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.4                  # m/s (conservative for reliability)
      min_vel_x: -0.2                 # m/s (backing up)
      max_vel_theta: 0.8              # rad/s
      acc_lim_x: 0.3                  # m/s^2
      acc_lim_theta: 1.0              # rad/s^2

# =============================================================================
# Nav2 Planner Server Overrides
# =============================================================================
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.2                  # meters to goal
      use_astar: false                # Dijkstra for reliability

# =============================================================================
# Nav2 Global Costmap Overrides
# =============================================================================
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.05
      robot_radius: 0.20              # meters (conservative)
      inflation_layer:
        inflation_radius: 0.4         # meters - wider safety margin

# =============================================================================
# Nav2 Local Costmap Overrides
# =============================================================================
local_costmap:
  local_costmap:
    ros__parameters:
      resolution: 0.05
      robot_radius: 0.20
      width: 5
      height: 5
      inflation_layer:
        inflation_radius: 0.3

# =============================================================================
# Nav2 Recovery Server Overrides
# =============================================================================
recoveries_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]

# =============================================================================
# Nav2 BT Navigator Overrides
# =============================================================================
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom

# =============================================================================
# Explore Mission Manager Node
# =============================================================================
mission_manager:
  ros__parameters:
    # Timing
    max_duration: 1800                # seconds (30 minutes)
    safety_margin_duration: 300       # seconds (5 min buffer)

    # Starting position
    home_position:
      x: 0.0
      y: 0.0
      theta: 0.0

    # Return to home
    return_home_enabled: true
    return_home_distance_tolerance: 0.3   # meters
    return_home_angular_tolerance: 0.2    # radians

    # Failure handling
    localization_loss_timeout: 5.0        # seconds
    stuck_timeout: 30.0                   # seconds
    max_recovery_attempts: 3

# =============================================================================
# Explore Explorer Node
# =============================================================================
explorer_node:
  ros__parameters:
    update_frequency: 5.0             # seconds (timer interval)

    # Frontier detection
    frontier_min_size: 5              # minimum frontier cells
    frontier_min_distance: 0.5        # meters (filter nearby)
    frontier_cluster_tolerance: 0.3   # meters (group nearby cells)

    # Frontier selection
    selection_strategy: "closest"     # closest, largest, or weighted
    distance_weight: 1.0
    size_weight: 0.5

    # Safety
    max_goal_distance: 10.0           # meters
    blacklist_timeout: 60.0           # seconds
    max_blacklist_size: 20

# =============================================================================
# Explore Object Detector Node
# =============================================================================
object_detector:
  ros__parameters:
    enabled: true
    model: "mobilenet_ssd"
    confidence_threshold: 0.7
    detection_rate: 2.0               # Hz

    # Classes to detect (COCO dataset)
    target_classes: ["bottle", "cup", "bowl", "chair", "couch"]

    # Catalog settings
    min_distance_between: 0.5         # meters (deduplicate)
    max_catalog_size: 100
    save_images: true

# =============================================================================
# Explore Diagnostics Node
# =============================================================================
diagnostics_node:
  ros__parameters:
    status_rate: 1.0                  # Hz
    health_check_interval: 5.0        # seconds

    # Thresholds
    min_battery_voltage: 11.0         # volts
    max_cpu_usage: 85.0               # percent
    max_memory_usage: 80.0            # percent
    min_lidar_points: 100             # per scan

    # Logging
    log_level: "INFO"
    log_to_file: true
    log_directory: "/var/log/explore"
    max_log_size_mb: 100

# =============================================================================
# Explore TUI Node
# =============================================================================
tui_node:
  ros__parameters:
    refresh_rate: 2.0                 # Hz
    show_map_preview: false
    color_enabled: true
    position_precision: 2             # decimal places
    percentage_precision: 1

# =============================================================================
# Development & Testing
# =============================================================================
# These can be set via environment or separate config_test.yaml
development:
  ros__parameters:
    simulation_mode: false
    debug_visualization: false
    record_rosbag: false
    rosbag_topics: ["/scan", "/map", "/odom", "/camera/rgb"]
```

### Configuration Usage

**Launch File Integration:**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('explore_bringup')
    config_yaml = os.path.join(pkg_dir, 'config', 'config.yaml')

    # SLAM Toolbox with our overrides
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[config_yaml]  # Our overrides applied to defaults
    )

    # Explore nodes read from same file
    mission_manager = Node(
        package='explore_mission',
        executable='mission_manager',
        name='mission_manager',
        parameters=[config_yaml]
    )
```

**Environment-Specific Overrides:** For testing or simulation, create additional override files:
- `config.yaml` — Production defaults
- `config_test.yaml` — Testing overrides (faster rates, smaller areas)
- `config_sim.yaml` — Simulation parameters

Pass multiple files to stack overrides:
```python
parameters=[config_yaml, config_test_yaml]  # test overrides production
```

**Runtime Parameter Updates:** ROS2 parameters enable runtime value changes:
```bash
ros2 param set /explorer_node update_frequency 2.0
ros2 param set /mission_manager stuck_timeout 45.0
```

### Configuration Validation

**Pre-flight Checks** validate all required parameters are present, check value ranges (e.g., velocities > 0), verify frame names are consistent across nodes, and ensure file paths exist and are writable.

**Configuration Schema:** JSON Schema or similar tools can validate config.yaml structure before loading (optional).

## Open Issues & Decisions

### Software
- Path planner choice: NavFn (simple, fast) vs Smac Planner (more sophisticated)

### Configuration

Tuning is required for SLAM parameters (resolution, update rates, loop closure settings), Nav2 (costmap inflation radius, max velocity, acceleration limits), exploration parameters (frontier size threshold, goal timeout, blacklist distance), and object detection (confidence threshold, detection rate, classes to detect).

### Development Environment

Decisions are needed regarding simulation (use Gazebo for testing before hardware ready?), development workflow (develop on robot vs remote computer), and testing environment (build test area with furniture/obstacles?).

### Future Enhancements (Post-MVP)

Potential future enhancements include AprilTag dock for improved return-to-home, battery monitoring and low-power return behavior, multi-floor exploration capability, more sophisticated object detection (custom trained models), web dashboard as alternative to TUI, and mission replay and analysis tools.
