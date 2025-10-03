#!/usr/bin/env python3
"""
Sensor Aggregator - Collects sensor data and publishes to Firebase
"""

import time
import math
from typing import Dict, Optional, Any
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from geometry_msgs.msg import PoseStamped


class SensorAggregator:
    """
    Aggregates sensor data from multiple ROS2 topics and publishes
    to Firebase at a controlled rate.
    """

    def __init__(self, node: Node, robot_id: str, firebase_client, logger):
        # 🚨 修正 1: robot_id を初期化時に受け取る
        self.node = node
        self.robot_id = robot_id
        self.firebase = firebase_client
        self.logger = logger
        self.last_odom_speed = 0.0 # オドメトリからの最新速度

        # Sensor data cache - 単一ロボットIDを使用
        self.sensor_data: Dict[str, Any] = {}
        self.last_publish_time = time.time()

        self.publish_interval = 2.0 # 2秒ごと (メインノードのタイマー設定と一致させる)

        # Initialize subscribers
        self.setup_subscriptions()

    def setup_subscriptions(self):
        """Setup ROS2 subscriptions for sensor topics."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # LiDAR / Laser scan
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # IMU (for advanced use - orientation/acceleration)
        self.imu_sub = self.node.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile
        )

        # Battery state (if available)
        self.battery_sub = self.node.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10 # デフォルトのQoSでOK
        )

        self.logger.info("✅ Sensor subscriptions initialized")

    # --- ROS2 Callbacks ---

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan data."""
        try:
            if 'lidar' not in self.sensor_data:
                 self.sensor_data['lidar'] = {}

            # Calculate minimum distance (obstacle detection)
            valid_ranges = [r for r in msg.ranges
                          if msg.range_min < r < msg.range_max]

            min_distance = min(valid_ranges) if valid_ranges else float('inf')

            # Detect obstacles (< 0.5m in front)
            obstacle_detected = min_distance < 0.5

            # 障害物角度の計算は、Web側では不要なため、一旦簡略化

            self.sensor_data['lidar'] = {
                'min_distance': round(min_distance, 2),
                'obstacle_detected': obstacle_detected,
                'timestamp': time.time()
            }
        except Exception as e:
            self.logger.error(f"Scan callback error: {e}")

    def imu_callback(self, msg: Imu):
        """Process IMU data."""
        try:
            if 'imu' not in self.sensor_data:
                self.sensor_data['imu'] = {}

            # Linear acceleration magnitude
            accel = msg.linear_acceleration
            accel_mag = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)

            self.sensor_data['imu'] = {
                'accel_magnitude': round(accel_mag, 2),
                'orientation_w': round(msg.orientation.w, 3),
                'timestamp': time.time()
            }

        except Exception as e:
            self.logger.error(f"IMU callback error: {e}")

    def battery_callback(self, msg: BatteryState):
        """Process battery state (real sensor)."""
        try:
            if 'battery' not in self.sensor_data:
                 self.sensor_data['battery'] = {}

            # ROS2 BatteryStateメッセージの形式に基づきpercentageを使用
            battery_percent = msg.percentage * 100

            self.sensor_data['battery'] = {
                'percent': round(battery_percent, 1),
                'voltage': round(msg.voltage, 2),
                'current': round(msg.current, 2),
                'charging': msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                'simulated': False, # 実機データ
                'timestamp': time.time()
            }
        except Exception as e:
            self.logger.error(f"Battery callback error: {e}")

    # --- Data Updaters (Main Nodeから呼び出し) ---

    def update_motion_state(self, speed: float):
        """メインノードのodomコールバックから速度を受け取る"""
        self.last_odom_speed = speed
        if 'motion' not in self.sensor_data:
            self.sensor_data['motion'] = {}
        self.sensor_data['motion']['speed'] = round(speed, 2)

    def update_navigation_state(self, goal_pose: Optional[PoseStamped], current_pose: PoseStamped):
        """
        メインノードのodomコールバックから、ナビゲーション関連の状態を受け取る。
        """
        try:
            if 'navigation' not in self.sensor_data:
                 self.sensor_data['navigation'] = {}

            if goal_pose:
                # Map座標での距離を計算
                dx = goal_pose.pose.position.x - current_pose.pose.position.x
                dy = goal_pose.pose.position.y - current_pose.pose.position.y
                distance = math.sqrt(dx**2 + dy**2)

                self.sensor_data['navigation'] = {
                    'distance_to_goal': round(distance, 2),
                    'has_goal': True,
                }
            else:
                self.sensor_data['navigation'] = {
                    'distance_to_goal': None,
                    'has_goal': False,
                }
        except Exception as e:
            self.logger.error(f"Navigation state update error: {e}")

    def simulate_battery(self):
        """
        Simulate battery drain for testing (when real battery topic unavailable).
        (メインノードのタイマーから呼び出されるべき)
        """
        # 🚨 修正 2: NoneType エラーを回避し、シミュレーションロジックを簡素化
        if 'battery' not in self.sensor_data or self.sensor_data['battery'].get('simulated', True):
            if 'battery' not in self.sensor_data:
                 # 初回初期化
                 self.sensor_data['battery'] = {
                    'percent': 100.0,
                    'voltage': 12.6,
                    'current': 0.0,
                    'charging': False,
                    'simulated': True,
                    'timestamp': time.time()
                 }

            battery_data = self.sensor_data['battery']
            time_delta = time.time() - battery_data['timestamp']

            # Drain rate: 0.05% per second when moving
            drain_rate = 0.05 * (self.last_odom_speed / 0.5) * time_delta # 0.5 m/s を max speed と仮定

            battery_data['percent'] = max(0, battery_data['percent'] - drain_rate)
            battery_data['current'] = self.last_odom_speed * 1.5 # 簡易的な電流シミュレーション
            battery_data['timestamp'] = time.time()

            self.logger.debug(f"Simulated battery: {battery_data['percent']:.1f}%")

    # --- Publishing Logic (Main Nodeからタイマーで呼び出し) ---

    def publish_telemetry_check(self):
        """メインノードのタイマーから呼び出され、集約データをFirebaseに送信する"""

        # 🚨 バッテリーデータがない場合、シミュレーションを実行
        if 'battery' not in self.sensor_data or self.sensor_data['battery'].get('simulated', True):
             self.simulate_battery()

        self._publish_telemetry()
        self.last_publish_time = time.time()


    def _publish_telemetry(self):
        """Publish aggregated sensor data to Firebase."""
        try:
            if not self.sensor_data:
                 return

            # Build telemetry document (Web UIが期待する形式にフラット化)
            telemetry: Dict[str, Any] = {
                'speed': self.sensor_data.get('motion', {}).get('speed', 0.0),
            }

            # Battery
            battery = self.sensor_data.get('battery', {})
            telemetry['battery_percent'] = battery.get('percent', 0.0)
            telemetry['battery_voltage'] = battery.get('voltage', 0.0)
            telemetry['battery_charging'] = battery.get('charging', False)

            # Obstacle detection
            lidar = self.sensor_data.get('lidar', {})
            telemetry['obstacle_detected'] = lidar.get('obstacle_detected', False)
            telemetry['min_obstacle_distance'] = lidar.get('min_distance', float('inf'))

            # Navigation
            navigation = self.sensor_data.get('navigation', {})
            telemetry['distance_to_goal'] = navigation.get('distance_to_goal', None)

            # Publish to Firebase (FirebaseClient.update_robot_telemetry を呼び出す)
            self.firebase.update_robot_telemetry(self.robot_id, telemetry)

            self.logger.debug(f"Published telemetry for {self.robot_id}")

        except Exception as e:
            self.logger.error(f"Telemetry publish error: {e}")