# ROS2 Firebase Bridge for Personal Mobility Platform

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![Firebase](https://img.shields.io/badge/Firebase-Admin_SDK-orange.svg)](https://firebase.google.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A real-time bidirectional bridge connecting ROS2 robotics systems with Firebase Firestore, enabling cloud-based fleet management, remote monitoring, and web-based control interfaces for autonomous mobile robots.

## 🎯 Project Overview

This project implements a production-ready integration layer between ROS2 (Robot Operating System 2) and Google Firebase, specifically designed for personal mobility platforms and autonomous delivery robots. The system enables:

- **Real-time Position Tracking**: GPS-synchronized robot position updates with coordinate transformation
- **Remote Navigation**: Cloud-based goal dispatch with Nav2 integration
- **Sensor Telemetry**: Aggregated sensor data streaming (LiDAR, IMU, Battery, Odometry)
- **Multi-Robot Support**: Scalable architecture supporting multiple robots simultaneously
- **Memory-Leak Prevention**: Advanced duplicate detection with automatic cleanup mechanisms

### Key Features

✅ **Bidirectional Synchronization**: ROS2 ↔ Firestore real-time data flow
✅ **Infinite Loop Prevention**: Idempotent destination handling with hash-based deduplication
✅ **Optimized Position Updates**: Distance-threshold based filtering (0.5m threshold)
✅ **Coordinate System Conversion**: GPS ↔ ROS2 Map coordinate transformation with configurable scaling
✅ **Nav2 Integration**: Seamless integration with ROS2 Navigation2 stack
✅ **Docker Development Environment**: Pre-configured `.devcontainer` with WSLg GPU support

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Web Interface                        │
│            (Firebase Realtime Database)                 │
└───────────────────┬─────────────────────────────────────┘
                    │ Firestore API
                    │
┌───────────────────▼─────────────────────────────────────┐
│             Firebase Bridge Node                        │
│  ┌──────────────────────────────────────────────────┐  │
│  │  • Destination Listener (Firestore → ROS2)       │  │
│  │  • State Publisher (ROS2 → Firestore)            │  │
│  │  • Coordinate Converter (GPS ↔ Map)              │  │
│  │  • Sensor Aggregator (Telemetry)                 │  │
│  └──────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────────────────────┘
                    │ ROS2 Topics & Actions
                    │
┌───────────────────▼─────────────────────────────────────┐
│              ROS2 Navigation Stack                      │
│  ┌──────────────────────────────────────────────────┐  │
│  │  • Nav2 (navigate_to_pose action)                │  │
│  │  • AMCL (Localization)                           │  │
│  │  • Costmap (Obstacle avoidance)                  │  │
│  └──────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────────────────────┘
                    │ Sensor Data
                    │
┌───────────────────▼─────────────────────────────────────┐
│           TurtleBot3 / Physical Robot                   │
│  • Odometry (/odom)                                     │
│  • LiDAR Scan (/scan)                                   │
│  • IMU (/imu)                                           │
│  • Battery State (/battery_state)                       │
└─────────────────────────────────────────────────────────┘
```

---

## 📋 Prerequisites

### Software Requirements

- **ROS2 Humble** (Desktop Full installation recommended)
- **Ubuntu 22.04 LTS** (or compatible Linux distribution)
- **Python 3.10+**
- **Docker** (for containerized development)
- **VS Code** with Remote-Containers extension (recommended)

### Hardware Support

- **TurtleBot3 Waffle** (simulation and real robot)
- **LiDAR Sensor** (e.g., LDS-01/02)
- **IMU Sensor** (optional, for enhanced localization)
- **GPS Module** (for outdoor operation)

### Firebase Setup

1. Create a Firebase project at [Firebase Console](https://console.firebase.google.com/)
2. Enable **Firestore Database** (Native mode)
3. Download the **Service Account Key** (JSON file)
4. Place the key at `/workspace/config/serviceAccountKey.json`

---

## 🚀 Quick Start

### Option 1: Docker Development Container (Recommended)

```bash
# Clone the repository
git clone https://github.com/yourusername/ros2-firebase-bridge.git
cd ros2-firebase-bridge

# Open in VS Code with Dev Container
code .
# Press: Ctrl+Shift+P → "Dev Containers: Reopen in Container"

# Inside the container, build the workspace
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash

# Run the bridge
ros2 run ros2_firebase_bridge firebase_bridge --ros-args -p robot_id:=robot_001
```

### Option 2: Native Installation

```bash
# Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install Python dependencies
pip3 install firebase-admin google-cloud-firestore python-dotenv --break-system-packages

# Clone and build
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2-firebase-bridge.git
cd ~/ros2_ws
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash

# Configure Firebase credentials
cp serviceAccountKey.json ~/ros2_ws/src/ros2-firebase-bridge/config/

# Launch simulation + bridge
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py
ros2 run ros2_firebase_bridge firebase_bridge
```

---

## 📦 Package Structure

```
ros2_firebase_bridge/
├── ros2_firebase_bridge/
│   ├── __init__.py
│   ├── firebase_bridge_node.py       # Main bridge node
│   ├── firebase_client.py            # Firestore connection manager
│   ├── coordinate_converter.py       # GPS ↔ Map coordinate conversion
│   ├── state_publisher.py            # Smart state publishing with filtering
│   └── sensor_aggregator.py          # Sensor data aggregation
├── config/
│   └── rviz/
│       └── firebase_config.yaml      # Configuration file
├── launch/
│   ├── multi_robot_firebase_bridge.launch.py
│   └── phase2_minimal.launch.py
├── scripts/
│   ├── firebase_bridge               # Executable entry point
│   ├── test_firebase.py              # Connection test script
│   ├── test_infinite_loop.py         # Loop prevention test
│   └── phase2_validation_tests.py    # Comprehensive test suite
├── worlds/
│   └── phase2_minimal.world          # Lightweight Gazebo world
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## ⚙️ Configuration

### Firebase Config (`config/rviz/firebase_config.yaml`)

```yaml
firebase:
  service_account_key: "/workspace/config/serviceAccountKey.json"
  database_url: "https://your-project.firebaseio.com"

ros2:
  robot_namespace: "/turtlebot3"
  goal_topic: "/goal_pose"
  odom_topic: "/odom"

coordinate_system:
  origin_latitude: 36.55077    # Map origin GPS coordinates
  origin_longitude: 139.92957
  scale_factor: 0.01           # GPS distance → Map distance (0.01 = 100m GPS → 1m Map)

  map_bounds:
    x_min: -4.0
    x_max: 4.0
    y_min: -4.0
    y_max: 4.0

telemetry:
  update_interval: 2.0          # Telemetry publish rate (seconds)
  position_update_interval: 2.0 # Position update rate
```

### Launch Parameters

```bash
# Single robot with custom ID
ros2 run ros2_firebase_bridge firebase_bridge \
  --ros-args \
  -p robot_id:=robot_001 \
  -p robot_namespace:=/turtlebot3

# Multiple robots (using launch file)
ros2 launch ros2_firebase_bridge multi_robot_firebase_bridge.launch.py
```

---

## 🔧 Core Components

### 1. Firebase Bridge Node (`firebase_bridge_node.py`)

Main orchestrator that manages all subsystems.

**Key Responsibilities:**
- Firestore real-time listener for destination updates
- Nav2 action client for navigation goal dispatch
- Duplicate destination detection (hash-based with timestamp tracking)
- Memory leak prevention (automatic cleanup of old hashes)
- Nav2 initialization sequence management

**Anti-Infinite-Loop Mechanism:**
```python
# Hash calculation with rounding to handle floating-point errors
lat_rounded = round(destination.latitude, 5)  # ~1.1m precision
lng_rounded = round(destination.longitude, 5)
hash_input = f"{lat_rounded:.5f}_{lng_rounded:.5f}"
destination_hash = hashlib.md5(hash_input.encode()).hexdigest()

# Thread-safe duplicate detection
with self.destination_lock:
    if destination_hash == self.last_processed_destination_hash:
        return  # Skip duplicate
```

### 2. Coordinate Converter (`coordinate_converter.py`)

Handles GPS ↔ ROS2 Map coordinate transformations.

**Features:**
- Haversine formula for accurate distance calculation
- Configurable scale factor for simulation environments
- Automatic map boundary enforcement
- Safe goal correction for out-of-bounds coordinates

**Example Usage:**
```python
converter = CoordinateConverter(
    origin_lat=36.55077,
    origin_lng=139.92957,
    scale_factor=0.01  # 100m GPS = 1m Map
)

# GPS → Map
map_coords = converter.gps_to_map_coordinates(36.55080, 139.92960)
# Returns: {'x': 0.33, 'y': 0.03}

# Map → GPS
gps_coords = converter.map_to_gps_coordinates(1.0, 2.0)
# Returns: {'lat': 36.55257, 'lng': 139.93757}
```

### 3. State Publisher (`state_publisher.py`)

Intelligent state publishing with rate limiting and threshold-based filtering.

**Optimization Features:**
- **Position Threshold**: 0.5m minimum movement before update
- **Time-Based Rate Limiting**: Max 1 update per second
- **Heading Threshold**: 0.1 radian minimum rotation

**Benefits:**
- Reduced Firestore write operations (cost savings)
- Prevented Firebase-to-ROS2 echo loops
- Smoother web UI marker updates

### 4. Sensor Aggregator (`sensor_aggregator.py`)

Collects and aggregates sensor data from multiple ROS2 topics.

**Supported Sensors:**
- **LiDAR** (`/scan`): Obstacle detection, minimum distance
- **IMU** (`/imu`): Acceleration magnitude, orientation
- **Battery** (`/battery_state`): Voltage, current, charge percentage
- **Odometry** (`/odom`): Speed, position

**Telemetry Data Format (Firestore):**
```json
{
  "telemetry": {
    "speed": 0.22,
    "battery_percent": 85.5,
    "obstacle_detected": false,
    "min_obstacle_distance": 2.35,
    "distance_to_goal": 3.42
  }
}
```

---

## 🧪 Testing

### Connection Test

```bash
# Test Firebase connection
python3 scripts/test_firebase.py

# Expected output:
# ✅ Firebase Admin SDK initialization successful
# ✅ Firestore connection successful
# ✅ Test data written successfully
```

### Infinite Loop Prevention Test

```bash
# Test duplicate destination handling
python3 scripts/test_infinite_loop.py

# Tests:
# 1. Duplicate destination filtering
# 2. High-frequency update handling
# 3. ROS2 echo-back prevention
```

### Phase 2 Validation Suite

```bash
# Comprehensive integration tests
python3 scripts/phase2_validation_tests.py

# Tests:
# 1. Infinite loop prevention (100 dispatches)
# 2. Position sync performance (30s test)
# 3. Multi-robot operation (3 robots)
```

---

## 📊 Performance Characteristics

### Position Update Optimization

| Metric | Before Optimization | After Optimization |
|--------|--------------------|--------------------|
| Firestore Writes/min | ~60 | ~6 |
| Average Update Interval | 1s | 5-10s |
| Movement Threshold | N/A | 0.5m |
| Cost Reduction | Baseline | **90%** |

### Memory Management

- **Automatic Hash Cleanup**: Old destination hashes (>1 hour) are removed every 10 minutes
- **Thread-Safe Operations**: All destination processing uses mutex locks
- **Bounded Memory Growth**: Hash dictionaries maintain constant size during long-term operation

---

## 🔍 Troubleshooting

### Firebase Bridge Not Starting

**Symptom:** `firebase_bridge` node fails to start

**Solutions:**
1. Check service account key path:
   ```bash
   ls -la /workspace/config/serviceAccountKey.json
   ```
2. Verify Firebase credentials:
   ```bash
   python3 scripts/test_firebase.py
   ```
3. Check log output:
   ```bash
   ros2 run ros2_firebase_bridge firebase_bridge --ros-args --log-level DEBUG
   ```

### Navigation Goals Not Accepted

**Symptom:** Nav2 rejects goals with "REJECTED" status

**Solutions:**
1. Set initial pose in RViz:
   - Click "2D Pose Estimate"
   - Click on map where robot is located
2. Verify AMCL localization:
   ```bash
   ros2 topic echo /amcl_pose
   ```
3. Check costmaps:
   ```bash
   ros2 topic list | grep costmap
   ```

### Position Not Updating in Firebase

**Symptom:** Robot position frozen in web interface

**Solutions:**
1. Check odometry topic:
   ```bash
   ros2 topic hz /odom
   ```
2. Verify bridge is receiving odom:
   ```bash
   ros2 node info /phase2_firebase_bridge
   ```
3. Check position threshold (may need to move >0.5m):
   ```python
   # In firebase_config.yaml
   telemetry:
     position_update_interval: 1.0  # Increase frequency
   ```

---

## 🛣️ Roadmap

### Completed (Phase 2)
- ✅ Infinite loop prevention with hash-based deduplication
- ✅ Position sync optimization (threshold-based filtering)
- ✅ Multi-robot support (namespace-based architecture)
- ✅ Memory leak prevention (automatic cleanup)
- ✅ Nav2 initialization sequence handling

### Planned (Future Phases)
- 🔲 AWS IoT Core integration (hybrid cloud)
- 🔲 Real-time video streaming
- 🔲 Fleet-wide task scheduling
- 🔲 Predictive maintenance alerts
- 🔲 GraphQL API for web clients
- 🔲 ROS2 Jazzy support

---

## 📚 Documentation

### Related Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Firebase Admin Python SDK](https://firebase.google.com/docs/admin/setup)
- [Nav2 Documentation](https://navigation.ros.org/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

### Academic Context

This project was developed as part of personal mobility research at Utsunomiya University, focusing on cloud-robotics integration for autonomous delivery systems.

---

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style

- Follow [PEP 8](https://pep8.org/) for Python code
- Use descriptive variable names
- Add docstrings to all functions
- Include type hints where applicable

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Yugo Obana**
Mechanical Engineering, Utsunomiya University
Specialization: Cloud-Robotics Integration, Autonomous Mobility Systems

### Connect
- LinkedIn: [your-profile](www.linkedin.com/in/yugo-dev)
- GitHub: [your-github](https://github.com/Iruazu)

---

## 🙏 Acknowledgments

- **ROS2 Community** for the robust robotics middleware
- **Google Firebase** for scalable cloud infrastructure
- **ROBOTIS** for TurtleBot3 platform and documentation
- **Utsunomiya University** for research support

---

## 📧 Support

For questions or issues:
- Open an [Issue](https://github.com/Iruazu/mobility-ros2-firebase/issues)
- Email: ygnk0805@outlook.jp

---

**Built with ❤️ for autonomous mobility and cloud robotics**