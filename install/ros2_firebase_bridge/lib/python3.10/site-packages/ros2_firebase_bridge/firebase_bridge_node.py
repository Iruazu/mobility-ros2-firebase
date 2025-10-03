#!/usr/bin/env python3
"""
Enhanced Firebase-ROS2 Bridge - Complete end-to-end integration
Replaces the existing firebase_bridge_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion # Quaternion to Yaw 変換用

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# Custom modules
# 🚨 インポートパスは ROS2 パッケージ構造に合わせて修正
from ros2_firebase_bridge.firebase_client import FirebaseClient
from ros2_firebase_bridge.coordinate_converter import CoordinateConverter
from ros2_firebase_bridge.state_publisher import StatePublisher, RobotStateTracker
from ros2_firebase_bridge.sensor_aggregator import SensorAggregator

import yaml
import os
import time
import math
from typing import Dict, Any, Optional


class EnhancedFirebaseBridge(Node):
    """
    Complete Firebase-ROS2 Bridge with sensor integration and
    smart state management to prevent infinite loops.
    """

    def __init__(self):
        super().__init__('enhanced_firebase_bridge')

        # Load configuration
        self.config = self.load_config()

        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # Core components (will be initialized after Firebase connection)
        self.firebase_client = None
        self.coordinate_converter = None
        self.state_publisher = None
        self.state_tracker = None
        self.sensor_aggregator = None

        # Robot state
        self.robot_id = "robot_001"
        self.current_goal = None # GPS Coordinates (from Firebase)
        self.navigation_active = False
        self.goal_handle = None # Nav2 Action Goal Handle

        # Setup ROS2 interfaces (Subscribers are set up here, callbacks are run later)
        self.setup_ros2_interfaces()

        # Initialize Firebase (async retry timer)
        self.firebase_init_timer = self.create_timer(
            1.0,
            self.initialize_firebase,
            callback_group=self.callback_group
        )

        self.get_logger().info("🚀 Enhanced Firebase Bridge started")

    def load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        # ... (設定ファイル読み込みロジックはそのまま利用)
        try:
            config_path = os.path.join(
                os.path.dirname(__file__),
                '../config/firebase_config.yaml'
            )

            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    return yaml.safe_load(f)
            else:
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
                'goal_topic': '/goal_pose',
                'odom_topic': '/odom',
                'scan_topic': '/scan',
                'battery_topic': '/battery_state'
            },
            'coordinate_system': {
                'origin_latitude': 36.5598,
                'origin_longitude': 139.9088,
                'map_frame': 'map'
            }
        }

    def setup_ros2_interfaces(self):
        """Setup all ROS2 publishers and subscribers."""
        try:
            # Publishers (Topics)
            self.goal_publisher = self.create_publisher(
                PoseStamped, '/goal_pose', 10, callback_group=self.callback_group
            )

            # Navigation2 Action Client (Action)
            self.nav_action_client = ActionClient(
                self, NavigateToPose, '/navigate_to_pose', callback_group=self.callback_group
            )

            # Subscribers (Odom is the main trigger for state updates)
            self.odom_subscriber = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10, callback_group=self.callback_group
            )

            # 🚨 Note: Scan and Battery subscribers are now handled by SensorAggregator

            self.get_logger().info("✅ ROS2 interfaces configured")

        except Exception as e:
            self.get_logger().error(f"ROS2 interface setup error: {e}")

    def initialize_firebase(self):
        """Initialize Firebase connection and core components."""
        try:
            if self.firebase_client is not None:
                return  # Already initialized

            service_account_path = self.config['firebase']['service_account_key']

            # Initialize Firebase client
            self.firebase_client = FirebaseClient(service_account_path, self.get_logger())

            # Initialize coordinate converter
            coord_config = self.config['coordinate_system']
            self.coordinate_converter = CoordinateConverter(
                origin_lat=coord_config['origin_latitude'],
                origin_lng=coord_config['origin_longitude'],
                logger=self.get_logger()
            )

            # Initialize state publisher (with smart filtering)
            self.state_publisher = StatePublisher(
                self.firebase_client, self.coordinate_converter, self.get_logger()
            )

            # Initialize state tracker
            self.state_tracker = RobotStateTracker(self, self.state_publisher)

            # 🚀 Initialize sensor aggregator (FIX: Passing robot_id)
            self.sensor_aggregator = SensorAggregator(
                self, self.robot_id, self.firebase_client, self.get_logger()
            )

            # Setup Firestore listener (FIX: Using correct arguments)
            self.firebase_client.setup_realtime_listener(
                'robots', self.on_firestore_update
            )

            # Initial setup in Firebase
            self.initialize_robot_in_firebase()

            self.get_logger().info("🔥 Firebase connection established")

            # Start telemetry timer only after successful init
            self.telemetry_timer = self.create_timer(
                self.sensor_aggregator.publish_interval,
                self.sensor_aggregator.publish_telemetry_check,
                callback_group=self.callback_group
            )

            # Stop initialization timer
            self.firebase_init_timer.cancel()

        except Exception as e:
            self.get_logger().error(f"Firebase initialization error: {e}")

    def initialize_robot_in_firebase(self):
        """Ensure robot document exists in Firebase with initial data."""
        # ... (初期データ設定ロジックはそのまま利用)
        try:
            robot_data = self.firebase_client.get_robot_data(self.robot_id)

            if robot_data is None:
                from firebase_admin import firestore
                initial_data = {
                    'id': self.robot_id,
                    'name': 'TurtleBot3 Alpha',
                    'status': 'idle',
                    'position': firestore.GeoPoint( # 🚨 GeoPoint を使用
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

                self.get_logger().info(f"✅ Initialized robot {self.robot_id} in Firebase")

        except Exception as e:
            self.get_logger().error(f"Robot initialization error: {e}")

    # ============================================================
    # FIRESTORE CALLBACKS (Firebase → ROS2)
    # ============================================================

    def on_firestore_update(self, robot_id: str, robot_data: Dict[str, Any], change_type: str):
        """Handle Firestore updates - send commands to ROS2."""
        try:
            if robot_id != self.robot_id:
                return  # Only handle our robot

            self.get_logger().info(f"📨 Firestore update: {robot_id} - {change_type}")

            # Check if destination changed (Firebase → ROS2 Command)
            if 'destination' in robot_data and robot_data['destination']:
                destination = robot_data['destination']

                # Check if this is a new destination
                if self.should_navigate_to(destination):
                    self.send_navigation_goal(destination)

            # Handle status changes (e.g., Web App sends 'stop')
            status = robot_data.get('status', 'unknown')
            if status == 'stop':
                self.cancel_navigation()

        except Exception as e:
            self.get_logger().error(f"Firestore update handler error: {e}")

    def should_navigate_to(self, destination) -> bool:
        """Check if we should send a new navigation goal."""
        # ... (ロジックはそのまま利用)
        if not self.navigation_active:
            return True

        if self.current_goal:
            # Assume destination is a GeoPoint object from Firestore.
            current_lat = self.current_goal.latitude
            current_lng = self.current_goal.longitude
            new_lat = destination.latitude
            new_lng = destination.longitude

            # If moved more than 1 meter, it's a new goal
            distance = self.coordinate_converter.calculate_distance(
                current_lat, current_lng, new_lat, new_lng
            )

            return distance > 1.0

        return True

    def send_navigation_goal(self, destination):
        """Send navigation goal to ROS2 Navigation2."""
        try:
            self.get_logger().info(
                f"🎯 Sending navigation goal: "
                f"({destination.latitude:.6f}, {destination.longitude:.6f})"
            )

            # Create PoseStamped message
            goal_pose = self.coordinate_converter.create_pose_stamped(
                destination.latitude, destination.longitude, frame_id='map'
            )

            # Publish to /goal_pose (for RViz visualization)
            self.goal_publisher.publish(goal_pose)

            # Send to Navigation2 action server
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
                self.get_logger().warning("Navigation2 action server not available")

        except Exception as e:
            self.get_logger().error(f"Navigation goal send error: {e}")

    def cancel_navigation(self):
        """Cancel current navigation."""
        try:
            if self.goal_handle: # Check if a goal handle exists
                self.get_logger().info("🛑 Cancelling navigation")
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda f: self.get_logger().info("Navigation goal cancelled.")
                )
            self.navigation_active = False
            self.current_goal = None
            self.goal_handle = None
            self.firebase_client.update_robot_state(self.robot_id, {'status': 'idle'}) # Update status

        except Exception as e:
            self.get_logger().error(f"Navigation cancel error: {e}")

    # --- Navigation2 Action Callbacks ---

    def nav_goal_response_callback(self, future):
        """Handle Navigation2 goal response."""
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error("Navigation goal rejected")
                self.navigation_active = False
                self.current_goal = None
                return

            self.get_logger().info("✅ Navigation goal accepted")
            self.goal_handle = goal_handle # Store goal handle for cancellation

            # Wait for result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)

        except Exception as e:
            self.get_logger().error(f"Nav goal response error: {e}")

    def nav_feedback_callback(self, feedback_msg):
        """Handle Navigation2 feedback (distance to goal)."""
        try:
            # Feedback is primarily handled by SensorAggregator via Odom callback
            pass

        except Exception as e:
            self.get_logger().error(f"Nav feedback error: {e}")

    def nav_result_callback(self, future):
        """Handle Navigation2 result (Success/Failure)."""
        try:
            result = future.result().result
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("🎉 Navigation completed successfully")

                # 🚀 FIX: Update Firebase status and clear destination
                self.firebase_client.update_robot_state(
                    self.robot_id,
                    {'status': 'idle', 'destination': firestore.DELETE_FIELD}
                )

            else:
                self.get_logger().warning(f"Navigation failed with status: {status}")
                self.firebase_client.update_robot_state(
                    self.robot_id,
                    {'status': 'idle'} # Reset to idle on failure
                )

            self.navigation_active = False
            self.current_goal = None
            self.goal_handle = None

        except Exception as e:
            self.get_logger().error(f"Nav result error: {e}")

    # ============================================================
    # ROS2 SENSOR CALLBACKS (ROS2 → Firebase)
    # ============================================================

    def odom_callback(self, msg: Odometry):
        """
        Handle odometry updates - trigger smart filtering and aggregation.
        """
        if self.state_tracker is None: return

        try:
            # 1. Update StateTracker (triggers smart Firebase position update)
            self.state_tracker.update_from_odom(self.robot_id, msg)

            # 2. Update Sensor Aggregator (speed and navigation state)
            speed = math.sqrt(
                msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
            )

            self.sensor_aggregator.update_motion_state(speed)

            # 🚨 Note: Battery simulation logic is now handled in SensorAggregator.publish_telemetry_check()
            # self.sensor_aggregator.simulate_battery(self.robot_id, speed) # 削除・移動

        except Exception as e:
            self.get_logger().error(f"Odom callback error: {e}")

    def scan_callback(self, msg: LaserScan):
        """Handle LiDAR scan updates."""
        if self.sensor_aggregator:
            self.sensor_aggregator.scan_callback(msg)

    def battery_callback(self, msg: BatteryState):
        """Handle battery state updates."""
        if self.sensor_aggregator:
            self.sensor_aggregator.battery_callback(msg)

    # ============================================================
    # UTILITIES
    # ============================================================

    def shutdown(self):
        """Clean shutdown."""
        try:
            self.get_logger().info("🛑 Shutting down Enhanced Firebase Bridge")

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
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()