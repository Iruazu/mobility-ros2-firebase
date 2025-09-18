#!/bin/bash
set -e

echo "=== ROS2 Firebase Bridge ビルド・実行スクリプト ==="

# ROS2環境設定
source /opt/ros/humble/setup.bash

# ワークスペースに移動
cd /workspace

echo "1. 依存関係の更新..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "2. パッケージビルド..."
colcon build --packages-select ros2_firebase_bridge

echo "3. 環境設定..."
source install/setup.bash

echo "✅ ビルド完了！"
echo ""
echo "🚀 実行方法:"
echo "Terminal 1: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "Terminal 2: ros2 launch turtlebot3_navigation2 navigation2.launch.py"
echo "Terminal 3: ros2 run ros2_firebase_bridge firebase_bridge"
echo ""
echo "🔧 デバッグ用コマンド:"
echo "ros2 topic list"
echo "ros2 node list"
echo "ros2 topic echo /goal_pose"