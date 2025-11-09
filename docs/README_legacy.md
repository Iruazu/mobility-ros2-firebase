# mobility-ros2-firebase (Legacy Docker Environment)

> ⚠️ **Warning**: This environment is deprecated. Legacy implementation using Docker + Nav2.  
> See `README_current.md` for the current Ubuntu-native environment.

## Overview

Initial implementation of an autonomous mobile robot system using ROS2 Humble + Firebase + Nav2, developed by Yugo Obana, Department of Mechanical Systems Engineering, Utsunomiya University.

**Key Features:**
- Self-contained Docker Compose environment
- Nav2 autonomous navigation
- Bidirectional Firebase Firestore communication
- TurtleBot3 Gazebo simulation
- GPU acceleration (WSLg support)

## Architecture
```
Docker Container (ROS2 Humble)
├── Gazebo (TurtleBot3 World)
├── Nav2 (Autonomous Navigation)
├── Firebase Bridge (Python)
│   ├── Firestore Listener
│   ├── Coordinate Conversion (GPS ↔ Map)
│   └── Goal Publisher
└── RViz (Visualization)
```

## Structure

### Core Directories
- `archive/docker/` - Docker environment definition
  - `Dockerfile` - ROS2 Humble + Nav2 + dependencies
  - `devcontainer.json` - VSCode Dev Container config
- `archive/legacy/launch/` - Launch scripts
  - `phase2_minimal.launch.py` - Minimal configuration
  - `multi_robot.launch.py` - Multi-robot support
- `archive/legacy/scripts/` - Debug tools
  - `coordinate_debug.py` - Coordinate conversion verification
  - `test_infinite_loop.py` - Infinite loop prevention test

### Configuration
- `archive/docker/rviz/firebase_config.yaml` - Firebase connection settings
```yaml
  firebase:
    service_account_key: "/path/to/serviceAccountKey.json"
  coordinate_system:
    origin_latitude: 36.5513
    origin_longitude: 139.9286
    scale_factor: 0.01
```

## Dependencies

**ROS2 Packages:**
- `ros-humble-nav2-bringup`
- `ros-humble-turtlebot3-gazebo`
- `ros-humble-gazebo-ros-pkgs`

**Python Libraries:**
- `firebase-admin>=6.0.0`
- `google-cloud-firestore>=2.11.0`

**System Requirements:**
- Docker Desktop (WSL2 backend recommended)
- GPU: Intel/NVIDIA (via WSLg)
- RAM: 8GB minimum

## Setup

### 1. Firebase Authentication
```bash
# Place serviceAccountKey.json
cp /path/to/serviceAccountKey.json config/
```

### 2. Start Docker Environment
```bash
# Open in VSCode Dev Container
# Or
docker-compose up -d
docker exec -it ros2_firebase_container bash
```

### 3. Build Packages
```bash
cd /workspace
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash
```

### 4. Run
```bash
# Minimal (single robot)
ros2 launch ros2_firebase_bridge phase2_minimal.launch.py

# Multi-robot
ros2 launch ros2_firebase_bridge multi_robot.launch.py
```

## Key Features

### Firebase Integration
- **Destination Monitoring**: Real-time detection of Firestore `robots/{robot_id}/destination` changes
- **Position Sync**: Convert `/odom` topic to GPS coordinates and update Firestore
- **Infinite Loop Prevention**: Hash-based duplicate detection

### Coordinate Conversion
- GPS (WGS84) ↔ Gazebo Map coordinates
- Scale factor: `0.01` (GPS 100m = Map 1m)
- Origin: Utsunomiya University Yoto Campus Building 7

### Nav2 Integration
- AMCL localization
- DWB Local Planner
- Global/Local Costmap
- Goal reached detection (tolerance: 0.5m)

## Troubleshooting

### Nav2 Not Starting
```bash
# Check Nav2 packages
ros2 pkg list | grep nav2

# Check action server
ros2 action list
```

### Firebase Connection Error
```bash
# Verify auth file
ls -l /workspace/config/serviceAccountKey.json

# Test connection
python3 scripts/test_firebase.py
```

### Coordinate Mismatch
```bash
# Debug coordinate conversion
python3 archive/legacy/scripts/coordinate_debug.py
```

## Limitations

- Docker environment dependent (not compatible with host Ubuntu)
- GPU required (WSLg/Mesa)
- Nav2 initialization requires ~15 seconds
- Concurrent robots: Tested up to 3

## Migration Guide

**Migrating to Current Ubuntu Environment:**
1. See `README_current.md`
2. Remove Nav2, switch to lightweight navigation
3. Run natively without Docker

## Development History

- **Phase 1**: Basic Firebase integration (Oct 2024)
- **Phase 2**: Infinite loop prevention, position sync optimization (Nov 2024)
- **Phase 3**: Multi-robot support (Dec 2024)
- **Deprecated**: Legacy status due to Ubuntu migration (Jan 2025)

## License

MIT License

## Contact

Developer: Yugo Obana  
Affiliation: Utsunomiya University, Mechanical Systems Engineering  
Project: Cloud-Robotics Integration Platform