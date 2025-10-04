#!/usr/bin/env python3
"""
Firebase-ROS2 Bridge - 無限ループ完全防止版
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from firebase_admin import firestore
from ros2_firebase_bridge.firebase_client import FirebaseClient
from ros2_firebase_bridge.coordinate_converter import CoordinateConverter
from ros2_firebase_bridge.state_publisher import StatePublisher, RobotStateTracker
from ros2_firebase_bridge.sensor_aggregator import SensorAggregator

import yaml
import os
import math
from typing import Dict, Any
import hashlib


class EnhancedFirebaseBridge(Node):
    """無限ループ防止機能を強化したFirebase-ROS2 Bridge"""

    def __init__(self):
        super().__init__('enhanced_firebase_bridge')

        self.config = self.load_config()
        self.callback_group = ReentrantCallbackGroup()

        # Core components
        self.firebase_client = None
        self.coordinate_converter = None
        self.state_publisher = None
        self.state_tracker = None
        self.sensor_aggregator = None

        # Robot state
        self.robot_id = "robot_001"
        self.current_goal = None
        self.navigation_active = False
        self.goal_handle = None

        # 🚨 無限ループ防止の強化
        self.last_processed_destination = None
        self.destination_hash = None  # destination のハッシュ値を保存
        self.processing_navigation = False
        self.last_position_update_time = 0.0
        self.position_update_cooldown = 2.0  # 位置更新の最小間隔を2秒に変更

        self.setup_ros2_interfaces()
        self.firebase_init_timer = self.create_timer(
            1.0, self.initialize_firebase, callback_group=self.callback_group
        )

        self.get_logger().info("🚀 Enhanced Firebase Bridge (Loop Prevention) started")

    def load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            possible_paths = [
                os.path.join(os.path.dirname(__file__), '..', 'config', 'firebase_config.yaml'),
                '/workspace/config/firebase_config.yaml',
            ]

            for config_path in possible_paths:
                if os.path.exists(config_path):
                    with open(config_path, 'r') as f:
                        return yaml.safe_load(f)

            return self.get_default_config()
        except Exception as e:
            self.get_logger().error(f"Config load error: {e}")
            return self.get_default_config()

    def get_default_config(self) -> Dict[str, Any]:
        """Return default configuration."""
        return {
            'firebase': {
                'service_account_key': '/workspace/config/serviceAccountKey.json'
            },
            'ros2': {
                'robot_namespace': '/turtlebot3',
                'odom_topic': '/odom',
            },
            'coordinate_system': {
                'origin_latitude': 36.55077,
                'origin_longitude': 139.92957,
                'map_frame': 'map'
            }
        }

    def setup_ros2_interfaces(self):
        """Setup all ROS2 publishers and subscribers."""
        try:
            self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

            self.odom_subscriber = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10
            )
            self.scan_subscriber = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10
            )

            self.get_logger().info("✅ ROS2 interfaces configured")
        except Exception as e:
            self.get_logger().error(f"ROS2 interface setup error: {e}")

    def initialize_firebase(self):
        """Initialize Firebase connection."""
        try:
            if self.firebase_client is not None:
                return

            service_account_path = self.config['firebase']['service_account_key']
            if not os.path.exists(service_account_path):
                self.get_logger().error(f"❌ Service account not found: {service_account_path}")
                return

            self.firebase_client = FirebaseClient(service_account_path, self.get_logger())

            coord_config = self.config['coordinate_system']
            self.coordinate_converter = CoordinateConverter(
                origin_lat=coord_config['origin_latitude'],
                origin_lng=coord_config['origin_longitude'],
                logger=self.get_logger()
            )

            self.state_publisher = StatePublisher(
                self.firebase_client, self.coordinate_converter, self.get_logger()
            )
            self.state_tracker = RobotStateTracker(self, self.state_publisher)
            self.sensor_aggregator = SensorAggregator(
                self, self.robot_id, self.firebase_client, self.get_logger()
            )

            # Firestore リスナー設定
            self.firebase_client.setup_realtime_listener(
                'robots', self.on_firestore_update
            )

            self.initialize_robot_in_firebase()

            self.telemetry_timer = self.create_timer(
                2.0, self.telemetry_publish_callback, callback_group=self.callback_group
            )

            self.firebase_init_timer.cancel()
            self.get_logger().info("🔥 Firebase connection established")

        except Exception as e:
            self.get_logger().error(f"Firebase initialization error: {e}")

    def initialize_robot_in_firebase(self):
        """Ensure robot document exists in Firebase."""
        try:
            robot_data = self.firebase_client.get_robot_data(self.robot_id)
            if robot_data is None:
                initial_data = {
                    'id': self.robot_id,
                    'name': 'TurtleBot3 Alpha',
                    'status': 'idle',
                    'position': firestore.GeoPoint(
                        self.config['coordinate_system']['origin_latitude'],
                        self.config['coordinate_system']['origin_longitude']
                    ),
                    'heading': 0.0,
                    'telemetry': {
                        'battery_percent': 100.0,
                        'speed': 0.0,
                        'distance_to_goal': None,
                        'obstacle_detected': False
                    }
                }
                self.firebase_client.db.collection('robots').document(
                    self.robot_id
                ).set(initial_data)
                self.get_logger().info(f"✅ Initialized robot {self.robot_id}")
        except Exception as e:
            self.get_logger().error(f"Robot initialization error: {e}")

    def calculate_destination_hash(self, destination) -> str:
        """destination の一意なハッシュ値を計算"""
        if destination is None:
            return None

        lat_str = f"{destination.latitude:.8f}"
        lng_str = f"{destination.longitude:.8f}"
        hash_input = f"{lat_str}_{lng_str}"

        return hashlib.md5(hash_input.encode()).hexdigest()

    def on_firestore_update(self, robot_id: str, robot_data: Dict[str, Any], change_type: str):
        """🚨 無限ループ防止を強化した Firestore 更新ハンドラ"""
        try:
            if robot_id != self.robot_id:
                return

            self.get_logger().debug(f"📨 Firestore update: {robot_id} - {change_type}")

            # 1️⃣ ナビゲーション処理中は無視
            if self.processing_navigation:
                self.get_logger().debug("⏸️ Navigation processing, skipping")
                return

            # 2️⃣ destination の確認
            if 'destination' not in robot_data or not robot_data['destination']:
                return

            destination = robot_data['destination']

            # 3️⃣ destination のハッシュ値をチェック
            new_hash = self.calculate_destination_hash(destination)

            if new_hash == self.destination_hash:
                self.get_logger().debug("⏸️ Same destination hash, skipping")
                return

            # 4️⃣ 新しい destination として処理
            self.destination_hash = new_hash
            self.last_processed_destination = destination

            self.get_logger().info(
                f"🎯 New destination detected: ({destination.latitude:.6f}, {destination.longitude:.6f})"
            )

            self.send_navigation_goal(destination)

        except Exception as e:
            self.get_logger().error(f"Firestore update error: {e}")

    def send_navigation_goal(self, destination):
        """Send navigation goal to Nav2 with validation."""
        try:
            self.processing_navigation = True

            # ✅ ゴール検証を追加
            is_valid, error_msg = self.coordinate_converter.validate_goal(
                destination.latitude, destination.longitude
            )

            if not is_valid:
                self.get_logger().error(f"❌ 無効なゴール: {error_msg}")

                # 最も近い安全な座標を取得
                safe_goal = self.coordinate_converter.get_safe_goal_near(
                    destination.latitude, destination.longitude
                )
                self.get_logger().warning(
                    f"🔧 安全な座標に補正: ({safe_goal['lat']:.6f}, {safe_goal['lng']:.6f})"
                )

                # 補正後の座標でゴールを作成
                goal_pose = self.coordinate_converter.create_pose_stamped(
                    safe_goal['lat'], safe_goal['lng'], frame_id='map'
                )
            else:
                # 通常のゴール作成
                self.get_logger().info(
                    f"🎯 Navigation goal: ({destination.latitude:.6f}, {destination.longitude:.6f})"
                )

                goal_pose = self.coordinate_converter.create_pose_stamped(
                    destination.latitude, destination.longitude, frame_id='map'
                )

            self.goal_publisher.publish(goal_pose)

            if self.nav_action_client.wait_for_server(timeout_sec=2.0):
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = goal_pose

                future = self.nav_action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.nav_feedback_callback
                )
                future.add_done_callback(self.nav_goal_response_callback)

                self.current_goal = destination
                self.navigation_active = True
            else:
                self.get_logger().warning("Nav2 action server not available")
                self.processing_navigation = False

        except Exception as e:
            self.get_logger().error(f"Navigation goal error: {e}")
            self.processing_navigation = False

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal response."""
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error("❌ Navigation goal rejected")
                self.navigation_active = False
                self.processing_navigation = False
                return

            self.get_logger().info("✅ Navigation goal accepted")
            self.goal_handle = goal_handle
            self.processing_navigation = False

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)

        except Exception as e:
            self.get_logger().error(f"Nav response error: {e}")
            self.processing_navigation = False

    def nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback."""
        pass

    def nav_result_callback(self, future):
        """Handle Nav2 result."""
        try:
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("🎉 Navigation completed")
            else:
                self.get_logger().warning(f"⚠️ Navigation failed: {status}")

            # 🚨 destination を削除してループを防止
            self.firebase_client.update_robot_state(
                self.robot_id,
                {
                    'destination': firestore.DELETE_FIELD,
                    'status': 'idle'
                }
            )

            # ハッシュ値とフラグをリセット
            self.navigation_active = False
            self.current_goal = None
            self.goal_handle = None
            self.destination_hash = None
            self.last_processed_destination = None

        except Exception as e:
            self.get_logger().error(f"Nav result error: {e}")

    def cancel_navigation(self):
        """Cancel current navigation."""
        try:
            if self.goal_handle:
                self.get_logger().info("🛑 Cancelling navigation")
                self.goal_handle.cancel_goal_async()

            self.navigation_active = False
            self.current_goal = None
            self.goal_handle = None
            self.processing_navigation = False
            self.destination_hash = None

            self.firebase_client.update_robot_state(
                self.robot_id,
                {
                    'status': 'idle',
                    'destination': firestore.DELETE_FIELD
                }
            )

        except Exception as e:
            self.get_logger().error(f"Cancel navigation error: {e}")

    def odom_callback(self, msg: Odometry):
        """🚨 位置同期対応 Odometry コールバック"""
        if not self.state_tracker or not self.sensor_aggregator:
            return

        try:
            current_time = self.get_clock().now().nanoseconds / 1e9

            # レート制限チェック（1秒に1回）
            if (current_time - self.last_position_update_time) < self.position_update_cooldown:
                return

            self.last_position_update_time = current_time

            # 🚨 位置更新を常に実行（Web側マーカー同期のため）
            # state_tracker内のフィルタリングで無駄な更新は抑制される
            self.state_tracker.update_from_odom(self.robot_id, msg)

            # 速度更新
            speed = math.sqrt(
                msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
            )
            self.sensor_aggregator.update_motion_state(speed)

            # ナビゲーション状態更新
            current_pose = PoseStamped()
            current_pose.pose = msg.pose.pose

            if self.current_goal and self.navigation_active:
                goal_pose = self.coordinate_converter.create_pose_stamped(
                    self.current_goal.latitude, self.current_goal.longitude
                )
                self.sensor_aggregator.update_navigation_state(goal_pose, current_pose)
            else:
                self.sensor_aggregator.update_navigation_state(None, current_pose)

        except Exception as e:
            self.get_logger().error(f"Odom callback error: {e}")

    def scan_callback(self, msg: LaserScan):
        """Handle LiDAR scan."""
        if self.sensor_aggregator:
            self.sensor_aggregator.scan_callback(msg)

    def battery_callback(self, msg: BatteryState):
        """Handle battery state."""
        if self.sensor_aggregator:
            self.sensor_aggregator.battery_callback(msg)

    def telemetry_publish_callback(self):
        """テレメトリ定期送信"""
        if self.sensor_aggregator:
            self.sensor_aggregator.publish_telemetry_check()

    def shutdown(self):
        """Clean shutdown."""
        try:
            self.get_logger().info("🛑 Shutting down")

            if self.firebase_client:
                self.firebase_client.close_listeners()

            if self.navigation_active:
                self.cancel_navigation()

        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        node = EnhancedFirebaseBridge()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt received")
        finally:
            node.shutdown()
            node.destroy_node()

    except Exception as e:
        print(f"Main error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()