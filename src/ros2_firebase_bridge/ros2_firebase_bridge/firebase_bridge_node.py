#!/usr/bin/env python3
"""
Enhanced Firebase-ROS2 Bridge - Fixed Version for ROS2 Humble
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 Messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# Firebase Admin
from firebase_admin import firestore

# Custom modules
from ros2_firebase_bridge.firebase_client import FirebaseClient
from ros2_firebase_bridge.coordinate_converter import CoordinateConverter
from ros2_firebase_bridge.state_publisher import StatePublisher, RobotStateTracker
from ros2_firebase_bridge.sensor_aggregator import SensorAggregator

import yaml
import os
import math
from typing import Dict, Any


class EnhancedFirebaseBridge(Node):
    """
    Firebase-ROS2 Bridge with sensor integration
    """

    def __init__(self):
        super().__init__('enhanced_firebase_bridge')

        # Load configuration
        self.config = self.load_config()

        # Callback group for concurrent operations
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

        # Setup ROS2 interfaces
        self.setup_ros2_interfaces()

        # Initialize Firebase (with retry)
        self.firebase_init_timer = self.create_timer(
            1.0,
            self.initialize_firebase,
            callback_group=self.callback_group
        )

        self.get_logger().info("🚀 Enhanced Firebase Bridge started")

    def load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            # Try multiple config locations
            possible_paths = [
                os.path.join(os.path.dirname(__file__), '..', 'config', 'firebase_config.yaml'),
                '/workspace/config/firebase_config.yaml',
                os.path.expanduser('~/config/firebase_config.yaml'),
            ]

            for config_path in possible_paths:
                if os.path.exists(config_path):
                    self.get_logger().info(f"Config found: {config_path}")
                    with open(config_path, 'r') as f:
                        return yaml.safe_load(f)

            self.get_logger().warning("No config file found, using defaults")
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
            # Publishers
            self.goal_publisher = self.create_publisher(
                PoseStamped, '/goal_pose', 10
            )

            # Navigation2 Action Client
            self.nav_action_client = ActionClient(
                self, NavigateToPose, '/navigate_to_pose'
            )

            # QoS for sensor topics
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

            # Subscribers
            self.odom_subscriber = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10
            )

            self.scan_subscriber = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, sensor_qos
            )

            # Battery subscriber (may not exist on all robots)
            try:
                self.battery_subscriber = self.create_subscription(
                    BatteryState, '/battery_state', self.battery_callback, 10
                )
            except Exception as e:
                self.get_logger().warning(f"Battery topic not available: {e}")

            self.get_logger().info("✅ ROS2 interfaces configured")

        except Exception as e:
            self.get_logger().error(f"ROS2 interface setup error: {e}")

    def initialize_firebase(self):
        """Initialize Firebase connection and core components."""
        try:
            if self.firebase_client is not None:
                return  # Already initialized

            service_account_path = self.config['firebase']['service_account_key']

            # Check if service account file exists
            if not os.path.exists(service_account_path):
                self.get_logger().error(f"❌ Service account not found: {service_account_path}")
                return

            # Initialize Firebase client
            self.firebase_client = FirebaseClient(service_account_path, self.get_logger())

            # Initialize coordinate converter
            coord_config = self.config['coordinate_system']
            self.coordinate_converter = CoordinateConverter(
                origin_lat=coord_config['origin_latitude'],
                origin_lng=coord_config['origin_longitude'],
                logger=self.get_logger()
            )

            # Initialize state publisher
            self.state_publisher = StatePublisher(
                self.firebase_client, self.coordinate_converter, self.get_logger()
            )

            # Initialize state tracker
            self.state_tracker = RobotStateTracker(self, self.state_publisher)

            # Initialize sensor aggregator
            self.sensor_aggregator = SensorAggregator(
                self, self.robot_id, self.firebase_client, self.get_logger()
            )

            # Setup Firestore listener
            self.firebase_client.setup_realtime_listener(
                'robots', self.on_firestore_update
            )

            # Initial setup in Firebase
            self.initialize_robot_in_firebase()

            self.get_logger().info("🔥 Firebase connection established")

            # Start telemetry timer
            self.telemetry_timer = self.create_timer(
                2.0,  # 2秒ごと
                self.telemetry_publish_callback,
                callback_group=self.callback_group
            )

            # Stop initialization timer
            self.firebase_init_timer.cancel()

        except Exception as e:
            self.get_logger().error(f"Firebase initialization error: {e}")

    def telemetry_publish_callback(self):
        """テレメトリ定期送信コールバック"""
        if self.sensor_aggregator:
            self.sensor_aggregator.publish_telemetry_check()

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

                self.get_logger().info(f"✅ Initialized robot {self.robot_id} in Firebase")

        except Exception as e:
            self.get_logger().error(f"Robot initialization error: {e}")

    # ============================================================
    # FIRESTORE CALLBACKS
    # ============================================================

    def on_firestore_update(self, robot_id: str, robot_data: Dict[str, Any], change_type: str):
        """Handle Firestore updates."""
        try:
            if robot_id != self.robot_id:
                return

            self.get_logger().info(f"📨 Firestore update: {robot_id} - {change_type}")

            # Check destination
            if 'destination' in robot_data and robot_data['destination']:
                destination = robot_data['destination']

                if self.should_navigate_to(destination):
                    self.send_navigation_goal(destination)

            # Handle status changes
            status = robot_data.get('status', 'unknown')
            if status == 'stop':
                self.cancel_navigation()

        except Exception as e:
            self.get_logger().error(f"Firestore update error: {e}")

    def should_navigate_to(self, destination) -> bool:
        """Check if navigation goal is new."""
        if not self.navigation_active:
            return True

        if self.current_goal:
            distance = self.coordinate_converter.calculate_distance(
                self.current_goal.latitude, self.current_goal.longitude,
                destination.latitude, destination.longitude
            )
            return distance > 1.0

        return True

    def send_navigation_goal(self, destination):
        """Send navigation goal to Nav2."""
        try:
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

        except Exception as e:
            self.get_logger().error(f"Navigation goal error: {e}")

    def cancel_navigation(self):
        """Cancel current navigation."""
        try:
            if self.goal_handle:
                self.get_logger().info("🛑 Cancelling navigation")
                self.goal_handle.cancel_goal_async()

            self.navigation_active = False
            self.current_goal = None
            self.goal_handle = None

            self.firebase_client.update_robot_state(
                self.robot_id, {'status': 'idle'}
            )

        except Exception as e:
            self.get_logger().error(f"Cancel navigation error: {e}")

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal response."""
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error("Navigation goal rejected")
                self.navigation_active = False
                return

            self.get_logger().info("✅ Navigation goal accepted")
            self.goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)

        except Exception as e:
            self.get_logger().error(f"Nav response error: {e}")

    def nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback."""
        pass

    def nav_result_callback(self, future):
        """Handle Nav2 result."""
        try:
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("🎉 Navigation completed")
                self.firebase_client.update_robot_state(
                    self.robot_id,
                    {'status': 'idle', 'destination': firestore.DELETE_FIELD}
                )
            else:
                self.get_logger().warning(f"Navigation failed: {status}")
                self.firebase_client.update_robot_state(
                    self.robot_id, {'status': 'idle'}
                )

            self.navigation_active = False
            self.current_goal = None
            self.goal_handle = None

        except Exception as e:
            self.get_logger().error(f"Nav result error: {e}")

    # ============================================================
    # ROS2 SENSOR CALLBACKS
    # ============================================================

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates."""
        if not self.state_tracker or not self.sensor_aggregator:
            return

        try:
            # Update position in Firebase
            self.state_tracker.update_from_odom(self.robot_id, msg)

            # Update speed
            speed = math.sqrt(
                msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
            )
            self.sensor_aggregator.update_motion_state(speed)

            # Update navigation state
            current_pose = PoseStamped()
            current_pose.pose = msg.pose.pose

            if self.current_goal and self.navigation_active:
                goal_pose = self.coordinate_converter.create_pose_stamped(
                    self.current_goal.latitude,
                    self.current_goal.longitude
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