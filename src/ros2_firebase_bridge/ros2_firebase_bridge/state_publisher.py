#!/usr/bin/env python3
"""
Smart State Publisher - Prevents infinite loops with rate limiting and delta checks
"""

import time
import math
from typing import Dict, Optional
from geometry_msgs.msg import Pose
from firebase_admin import firestore # GeoPoint を使用するためにインポート (FirebaseClient側で処理される場合もあるが、ここで準備)
import rclpy
from rclpy.node import Node


class StatePublisher:
    """
    Manages publishing robot state to Firebase with intelligent filtering
    to prevent infinite update loops.
    """

    def __init__(self, firebase_client, coordinate_converter, logger):
        self.firebase = firebase_client
        self.converter = coordinate_converter
        self.logger = logger

        # Rate limiting: Don't update Firebase more than once per second
        self.min_update_interval = 1.0  # seconds
        self.last_update_time = {}

        # Delta threshold: Only update if robot moved significantly
        self.position_threshold = 0.5  # meters
        self.heading_threshold = 0.1   # radians (~5.7 degrees)
        self.last_published_state = {}

    def should_publish_update(self, robot_id: str, new_position: Dict,
                            new_heading: float) -> bool:
        """
        Determine if state update should be published to Firebase.
        """
        current_time = time.time()

        # レイヤー 1: 時間ベースのレート制限
        if robot_id in self.last_update_time:
            time_since_update = current_time - self.last_update_time[robot_id]
            if time_since_update < self.min_update_interval:
                return False

        # レイヤー 2: デルタチェック
        if robot_id in self.last_published_state:
            last_state = self.last_published_state[robot_id]

            # 距離と方位角の差分を計算
            distance = self._calculate_distance(last_state['position'], new_position)
            heading_delta = abs(new_heading - last_state['heading'])

            # 閾値未満なら無視
            if distance < self.position_threshold and heading_delta < self.heading_threshold:
                return False

        return True

    def publish_state(self, robot_id: str, map_x: float, map_y: float,
                      heading: float, additional_data: Optional[Dict] = None):
        """
        Publish robot state to Firebase if update criteria met.
        """
        try:
            # 1. 座標変換
            # converter.map_to_gps_coordinates は {'lat': float, 'lng': float} を返すと仮定
            gps_coords = self.converter.map_to_gps_coordinates(map_x, map_y)

            # 2. フィルタリングチェック
            if not self.should_publish_update(robot_id, gps_coords, heading):
                return

            # 3. データを準備 (GeoPoint オブジェクトの作成)
            # 🚨 Note: GeoPointオブジェクトは update_data に含め、FirebaseClient で処理されるようにする
            update_data = {
                'position': firestore.GeoPoint(gps_coords['lat'], gps_coords['lng']),
                'heading': heading,
                'last_updated': firestore.SERVER_TIMESTAMP
            }

            if additional_data:
                update_data.update(additional_data)

            # 4. 🚀 Firebaseへの更新 (メソッド名修正)
            # 🚨 エラー解消: update_robot_status -> update_robot_state
            self.firebase.update_robot_state(robot_id, update_data)

            # 5. トラッキング情報を更新
            self.last_update_time[robot_id] = time.time()
            self.last_published_state[robot_id] = {
                'position': gps_coords,
                'heading': heading
            }

            self.logger.debug(
                f"Published state for {robot_id}: "
                f"GPS({gps_coords['lat']:.6f}, {gps_coords['lng']:.6f}), "
                f"heading={heading:.2f}rad"
            )

        except Exception as e:
            self.logger.error(f"State publish error for {robot_id}: {e}")

    def _calculate_distance(self, pos1: Dict, pos2: Dict) -> float:
        """Calculate distance between two GPS positions in meters (Haversine formula)."""
        # (Haversine formula はそのまま利用)
        R = 6371000
        dlat = math.radians(pos2['lat'] - pos1['lat'])
        dlng = math.radians(pos2['lng'] - pos1['lng'])
        a = (math.sin(dlat/2)**2 +
             math.cos(math.radians(pos1['lat'])) *
             math.cos(math.radians(pos2['lat'])) *
             math.sin(dlng/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c


class RobotStateTracker:
    """
    Tracks state of multiple robots and publishes updates.
    (Main bridge node が Odom コールバック内で使用)
    """

    def __init__(self, node: Node, state_publisher: StatePublisher):
        self.node = node
        self.publisher = state_publisher
        self.robots = {}  # robot_id -> latest state

        # 🚨 RobotStateTracker は単なるデータ集約ロジックとして機能させるため、
        # Odomメッセージの処理は update_from_odom メソッドで実行する

    def update_from_odom(self, robot_id: str, odom_msg):
        """Update robot state from odometry message."""
        try:
            # Extract position
            map_x = odom_msg.pose.pose.position.x
            map_y = odom_msg.pose.pose.position.y

            # Extract heading from quaternion
            q = odom_msg.pose.pose.orientation
            # 🚨 Note: _quaternion_to_yaw はこのクラスの外部に定義されているか、
            # tf_transformations からインポートして使用することを推奨します。
            # ここでは便宜上、クラスメソッドとして実装。
            heading = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)

            # Calculate speed
            linear_vel = odom_msg.twist.twist.linear
            speed = math.sqrt(linear_vel.x**2 + linear_vel.y**2)

            # Store state
            self.robots[robot_id] = {
                'map_x': map_x,
                'map_y': map_y,
                'heading': heading,
                'speed': speed
            }

            # Publish to Firebase (with smart filtering)
            # 🚨 Note: 速度は sensor_aggregator 側で処理するため、ここでは status update のみに集中
            self.publisher.publish_state(
                robot_id, map_x, map_y, heading
                # additional_data={'telemetry': {'speed': speed}} は sensor_aggregator に任せる
            )

        except Exception as e:
            self.node.get_logger().error(f"Odom update error (RobotStateTracker): {e}")

    def _quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_robot_state(self, robot_id: str) -> Optional[Dict]:
        """Get current tracked state for a robot."""
        return self.robots.get(robot_id)