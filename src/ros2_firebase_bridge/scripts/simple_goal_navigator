#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import firebase_admin
from firebase_admin import credentials, db as firebase_db, firestore
import math
import os
import time

class SimpleGoalNavigator(Node):
    def __init__(self):
        super().__init__('simple_goal_navigator')

        # Firebase initialization
        cred_path = os.path.expanduser('~/mobility-ros2-firebase/src/ros2_firebase_bridge/config/serviceAccountKey.json')
        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://mobility-map-ae58e-default-rtdb.asia-southeast1.firebasedatabase.app/'
            })
            self.get_logger().info('✅ Firebase initialized successfully.')

        # Firestore client
        self.firestore_db = firestore.client()

        # Robot ID (Firestoreのドキュメント名)
        self.robot_id = 'robot_001'

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Goal
        self.goal_x = None
        self.goal_y = None
        self.has_goal = False

        # Control parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.goal_tolerance = 0.3  # meters (30cm)

        # Timer for control loop (10Hz)
        self.create_timer(0.1, self.control_loop)

        # Timer for Firebase Realtime DB polling (2Hz)
        self.create_timer(0.5, self.check_goal_updates)

        # Timer for Firestore updates (5Hz)
        self.create_timer(0.2, self.update_firestore_position)

        # Firebase references
        self.goal_ref = firebase_db.reference('robot/goal')
        self.status_ref = firebase_db.reference('robot/status')
        self.position_ref = firebase_db.reference('robot/current_position')

        # Initialize status
        self.status_ref.set('idle')

        # Last goal hash
        self.last_goal_hash = None

        self.get_logger().info('🚀 Simple Goal Navigator initialized')
        self.get_logger().info(f'📍 Polling Firebase: robot/goal every 0.5s')
        self.get_logger().info(f'📤 Updating Firestore: robots/{self.robot_id} every 0.2s')
        self.get_logger().info(f'🎯 Goal tolerance: {self.goal_tolerance}m')

        # Initialize robot position in Firestore with retry
        self.initialize_robot_position()

    def initialize_robot_position(self):
        """Initialize robot position in Firestore with retry"""
        max_retries = 3
        retry_delay = 5  # seconds

        for attempt in range(max_retries):
            try:
                robot_ref = self.firestore_db.collection('robots').document(self.robot_id)

                # 初期位置: Gazeboのモデル初期位置に合わせる
                initial_lat = 36.551291
                initial_lng = 139.928716

                robot_ref.update({
                    'position': firestore.GeoPoint(initial_lat, initial_lng),
                    'status': 'idle',
                    'last_updated': firestore.SERVER_TIMESTAMP
                })

                self.get_logger().info(f'📍 Initial position set: ({initial_lat}, {initial_lng})')
                return  # 成功したら終了

            except Exception as e:
                error_msg = str(e)
                if '429' in error_msg or 'Quota exceeded' in error_msg or 'quota' in error_msg.lower():
                    if attempt < max_retries - 1:
                        self.get_logger().warn(f'⚠️ Quota exceeded, retrying in {retry_delay}s... (attempt {attempt + 1}/{max_retries})')
                        time.sleep(retry_delay)
                    else:
                        self.get_logger().error(f'❌ Failed to set initial position after {max_retries} attempts: Quota exceeded')
                        self.get_logger().warn('⚠️ Continuing without initial position update. Please set position manually in Firebase Console.')
                else:
                    self.get_logger().error(f'❌ Failed to set initial position: {e}')
                    return

    def odom_callback(self, msg):
        """Odometry callback - update current position"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def gps_to_gazebo(self, lat, lng):
        """Convert GPS coordinates to Gazebo coordinates"""
        # 基準点: Gazebo (0, 0) = GPS (36.551291, 139.928716)
        base_lat = 36.551291
        base_lng = 139.928716

        # 1度の距離（メートル）
        # 緯度: 約111,320m/度
        # 経度: 約91,290m/度（日本の緯度付近）
        meters_per_lat = 111320.0
        meters_per_lng = 91290.0

        # GPS差分をメートルに変換
        gazebo_y = (lat - base_lat) * meters_per_lat
        gazebo_x = (lng - base_lng) * meters_per_lng

        return gazebo_x, gazebo_y

    def check_goal_updates(self):
        """Firebase Realtime DB polling - check for goal updates"""
        try:
            goal_data = self.goal_ref.get()

            if goal_data and isinstance(goal_data, dict):
                new_goal_x = goal_data.get('x')
                new_goal_y = goal_data.get('y')

                if new_goal_x is not None and new_goal_y is not None:
                    # GPS座標をGazebo座標に変換
                    gazebo_x, gazebo_y = self.gps_to_gazebo(float(new_goal_x), float(new_goal_y))

                    # ハッシュ計算（同じ目標の重複検知）
                    goal_hash = f"{gazebo_x:.2f}_{gazebo_y:.2f}"

                    if goal_hash != self.last_goal_hash:
                        self.goal_x = gazebo_x
                        self.goal_y = gazebo_y
                        self.has_goal = True
                        self.last_goal_hash = goal_hash

                        self.get_logger().info(f'🎯 New goal: GPS({new_goal_x:.6f}, {new_goal_y:.6f}) → Gazebo({gazebo_x:.2f}, {gazebo_y:.2f})')

                        # Update Realtime DB status to moving
                        self.status_ref.set('moving')

                        # 🚀 重要: Firestore も moving に更新
                        try:
                            robot_ref = self.firestore_db.collection('robots').document(self.robot_id)
                            robot_ref.update({
                                'status': 'moving',
                                'last_updated': firestore.SERVER_TIMESTAMP
                            })
                            self.get_logger().info('✅ Firestore status updated to moving')
                        except Exception as e:
                            self.get_logger().warn(f'⚠️ Failed to update Firestore status: {e}')

        except Exception as e:
            self.get_logger().error(f'❌ Firebase Realtime DB polling error: {e}')

    def update_firestore_position(self):
        """Update current position to Firestore"""
        try:
            # Gazebo座標系をGPS座標系に変換
            # 基準点: Gazebo (0, 0) = GPS (36.551291, 139.928716)
            base_lat = 36.551291
            base_lng = 139.928716

            # 1度の距離（メートル）
            meters_per_lat = 111320.0
            meters_per_lng = 91290.0

            # Gazebo座標をGPS座標に変換
            gps_lat = base_lat + (self.current_y / meters_per_lat)
            gps_lng = base_lng + (self.current_x / meters_per_lng)

            # Firestoreに書き込み
            robot_ref = self.firestore_db.collection('robots').document(self.robot_id)
            robot_ref.update({
                'position': firestore.GeoPoint(gps_lat, gps_lng),
                'last_updated': firestore.SERVER_TIMESTAMP
            })

        except Exception as e:
            error_msg = str(e)
            # Quota exceeded エラーは警告レベルに（頻繁に出る可能性があるため）
            if '429' in error_msg or 'Quota exceeded' in error_msg or 'quota' in error_msg.lower():
                # 最初の1回だけ警告を出す
                if not hasattr(self, '_quota_warning_shown'):
                    self.get_logger().warn(f'⚠️ Firestore quota exceeded. Position updates may be delayed.')
                    self._quota_warning_shown = True
            else:
                self.get_logger().error(f'❌ Firestore update error: {e}')

    def control_loop(self):
        """Main control loop - runs at 10Hz"""
        if not self.has_goal:
            return

        # Calculate distance and angle to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.current_yaw

        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        twist = Twist()

        # デバッグ: 距離をログ出力（5秒ごと）
        if not hasattr(self, '_last_distance_log_time'):
            self._last_distance_log_time = 0
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self._last_distance_log_time > 5.0:
            self.get_logger().info(f'📏 Distance to goal: {distance:.2f}m (tolerance: {self.goal_tolerance}m)')
            self._last_distance_log_time = current_time

        if distance < self.goal_tolerance:
            # Goal reached
            self.get_logger().info(f'✅ Goal reached! Final distance: {distance:.2f}m')
            self.has_goal = False
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            # Update Firebase Realtime DB
            self.status_ref.set('idle')
            self.goal_ref.delete()

            # Update Firestore status with retry
            self.update_firestore_status_with_retry('idle')

        elif abs(angle_diff) > 0.2:
            # Rotate towards goal
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed

        else:
            # Move forward
            twist.linear.x = min(self.linear_speed, distance * 0.5)  # Slow down near goal
            twist.angular.z = angle_diff * 0.5  # Proportional turning

        self.cmd_vel_pub.publish(twist)

    def update_firestore_status_with_retry(self, status):
        """Update Firestore status with retry on quota exceeded"""
        max_retries = 2
        retry_delay = 3  # seconds

        for attempt in range(max_retries):
            try:
                robot_ref = self.firestore_db.collection('robots').document(self.robot_id)
                robot_ref.update({
                    'status': status,
                    'last_updated': firestore.SERVER_TIMESTAMP
                })
                self.get_logger().info(f'✅ Status updated to {status} in Firestore')
                return

            except Exception as e:
                error_msg = str(e)
                if '429' in error_msg or 'Quota exceeded' in error_msg or 'quota' in error_msg.lower():
                    if attempt < max_retries - 1:
                        self.get_logger().warn(f'⚠️ Quota exceeded when updating status, retrying in {retry_delay}s...')
                        time.sleep(retry_delay)
                    else:
                        self.get_logger().warn(f'⚠️ Failed to update Firestore status after {max_retries} attempts')
                else:
                    self.get_logger().error(f'❌ Failed to update Firestore status: {e}')
                    return

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()