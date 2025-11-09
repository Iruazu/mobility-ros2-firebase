# mobility-ros2-firebase (Current Ubuntu Environment)

> 🚀 **Current Implementation**: Ubuntu 22.04 + ROS2 Humble + Firebase Dispatch Integration  
> Lightweight real-time dispatch system without Nav2 dependency

## Overview

Real-time robot dispatch system using ROS2 and Firebase, developed by Yugo Obana, Department of Mechanical Systems Engineering, Utsunomiya University.

**Key Features:**
- **Native Ubuntu Execution** (No Docker required)
- **Firebase Dispatch Integration** (GPS coordinates → Gazebo control)
- **Custom Campus Model** (Utsunomiya University Yoto Campus)
- **Lightweight Navigation** (No Nav2, direct cmd_vel control)
- **Real-time Position Sync** (Firestore ↔ ROS2 Odometry)

## Architecture
```
Ubuntu 22.04 (Native)
├── Gazebo
│   └── Utsunomiya Campus Model (.dae)
├── ROS2 Nodes
│   ├── simple_goal_navigator (Main Node)
│   │   ├── Firebase Realtime DB Polling
│   │   ├── GPS → Gazebo Coordinate Conversion
│   │   ├── cmd_vel Control
│   │   └── Firestore Position Sync
│   └── robot_state_publisher
└── Firebase
    ├── Realtime DB (goal distribution)
    └── Firestore (position/status sync)
```

## Structure

### Main Code
- `src/ros2_firebase_bridge/ros2_firebase_bridge/simple_goal_navigator.py`
  - Fetch goals from Firebase Realtime DB (2Hz polling)
  - GPS → Gazebo coordinate conversion
  - PID-style control for robot movement
  - Position sync to Firestore (5Hz)

### Launch Files
- `src/ros2_firebase_bridge/launch/utsunomiya_campus.launch.py`
  - Start Gazebo
  - Load campus model
  - Spawn TurtleBot3

### 3D Model
- `src/ros2_firebase_bridge/models/utsunomiya_campus/`
  - `utunomiya-yoto7.dae` - Generated with Blender GIS
  - `model.sdf` - Gazebo definition

### Configuration
- `src/ros2_firebase_bridge/config/firebase_config.yaml`
```yaml
  firebase:
    service_account_key: "/home/obana/mobility-ros2-firebase/src/ros2_firebase_bridge/config/serviceAccountKey.json"
    database_url: "https://mobility-map-ae58e-default-rtdb.asia-southeast1.firebasedatabase.app/"
  map:
    origin:
      latitude: 36.5546169518534
      longitude: 139.87190842628479
    scale: 111320.0
```

## Setup

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble (Desktop Full)
- Gazebo 11
- Python 3.10+

### 1. Install Dependencies
```bash
# ROS2 Humble (if not installed)
sudo apt update
sudo apt install ros-humble-desktop-full

# Gazebo & TurtleBot3
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3*

# Python dependencies
pip install firebase-admin google-cloud-firestore PyYAML --break-system-packages
```

### 2. Firebase Setup
```bash
# Place serviceAccountKey.json
mkdir -p ~/mobility-ros2-firebase/src/ros2_firebase_bridge/config
cp /path/to/serviceAccountKey.json ~/mobility-ros2-firebase/src/ros2_firebase_bridge/config/
```

### 3. Build
```bash
cd ~/mobility-ros2-firebase
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash
```

### 4. Environment Variables
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/mobility-ros2-firebase/src/ros2_firebase_bridge/models
```

## Running

### Basic Launch
```bash
# Terminal 1: Gazebo + Campus Model
ros2 launch ros2_firebase_bridge utsunomiya_campus.launch.py

# Terminal 2: Simple Goal Navigator
ros2 run ros2_firebase_bridge simple_goal_navigator
```

### Verification
1. **Set goal in Firebase Realtime DB**
```json
   // Path: robot/goal
   {
     "x": 36.551291,  // Latitude
     "y": 139.928716  // Longitude
   }
```

2. **Verify movement in Gazebo**
   - TurtleBot3 moves toward goal
   - Goal deleted upon reaching (tolerance 0.3m)

3. **Check position in Firestore**
```javascript
   // Collection: robots/robot_001
   {
     position: GeoPoint(36.551291, 139.928716),
     status: "moving" | "idle"
   }
```

## Coordinate Conversion Logic

### GPS → Gazebo
```python
# Reference: Gazebo (0,0) = GPS (36.551291, 139.928716)
base_lat = 36.551291
base_lng = 139.928716

# Meters per degree
meters_per_lat = 111320.0  # Latitude
meters_per_lng = 91290.0   # Longitude (Japan area)

# Conversion
gazebo_y = (gps_lat - base_lat) * meters_per_lat
gazebo_x = (gps_lng - base_lng) * meters_per_lng
```

### Gazebo → GPS
```python
gps_lat = base_lat + (gazebo_y / meters_per_lat)
gps_lng = base_lng + (gazebo_x / meters_per_lng)
```

## Navigation Control

### Control Loop (10Hz)
```python
# Calculate distance and angle
dx = goal_x - current_x
dy = goal_y - current_y
distance = sqrt(dx² + dy²)
goal_angle = atan2(dy, dx)
angle_diff = normalize(goal_angle - current_yaw)

# Control
if distance < 0.3:  # Goal reached
    stop()
elif abs(angle_diff) > 0.2:  # Rotate
    rotate(angular_speed)
else:  # Move forward
    move_forward(linear_speed)
```

## Troubleshooting

### Robot Not Moving
```bash
# Check odometry
ros2 topic echo /odom --once

# Check cmd_vel
ros2 topic echo /cmd_vel
```

### Firebase Connection Error
```bash
# Verify auth file
ls -l ~/mobility-ros2-firebase/src/ros2_firebase_bridge/config/serviceAccountKey.json

# Test connection
python3 scripts/test_firebase.py
```

### Gazebo Model Not Displaying
```bash
# Check model path
echo $GAZEBO_MODEL_PATH

# Verify model files
ls -l src/ros2_firebase_bridge/models/utsunomiya_campus/
```

### Firestore Quota Exceeded
```
⚠️ Free tier: 20,000 writes/day
Solution: Reduce position update rate from 5Hz to 1Hz
```

## Limitations

- **No Nav2**: No obstacle avoidance
- **Single Robot**: Currently robot_001 only
- **GPS Accuracy**: ±1m error margin
- **Firestore Limits**: Mind write frequency

## Work in Progress

The following features are under development:

- [ ] Nav2 integration (autonomous obstacle avoidance)
- [ ] Multi-robot simultaneous control
- [ ] Web UI dashboard
- [ ] Real robot testing (actual TurtleBot3)

## Development Roadmap

1. **Phase 4** (Current): Firebase Dispatch completion
2. **Phase 5** (March 2025): Nav2 integration
3. **Phase 6** (June 2025): Real robot deployment

## License

MIT License

## Contact

Developer: Yugo Obana  
Affiliation: Utsunomiya University, Mechanical Systems Engineering  
Goal: Mobility × Cloud × AI Architect  