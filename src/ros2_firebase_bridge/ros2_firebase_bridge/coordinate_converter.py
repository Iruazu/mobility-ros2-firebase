"""GPS座標とROS2マップ座標の変換ユーティリティ"""

import math
import logging
from typing import Dict, Tuple
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import rclpy

class CoordinateConverter:
    def __init__(self, origin_lat: float = 36.5598, origin_lng: float = 139.9088,
                 logger=None):
        """
        座標変換ユーティリティを初期化

        Args:
            origin_lat: マップ座標系の原点緯度
            origin_lng: マップ座標系の原点経度
            logger: ロガーインスタンス
        """
        self.origin_lat = origin_lat
        self.origin_lng = origin_lng
        self.logger = logger or logging.getLogger(__name__)

        self.logger.info(f"座標系原点設定: ({origin_lat:.6f}, {origin_lng:.6f})")

    def gps_to_map_coordinates(self, lat: float, lng: float) -> Dict[str, float]:
        """
        GPS座標をROS2マップ座標系に変換

        Args:
            lat: 緯度
            lng: 経度

        Returns:
            マップ座標 {'x': float, 'y': float} (メートル単位)
        """
        try:
            R = 6371000  # 地球半径(m)

            # 緯度・経度差をラジアンに変換
            dlat = math.radians(lat - self.origin_lat)
            dlng = math.radians(lng - self.origin_lng)

            # メートル単位に変換
            # x: 東西方向（経度方向）
            # y: 南北方向（緯度方向）
            x = dlng * R * math.cos(math.radians(self.origin_lat))
            y = dlat * R

            self.logger.debug(f"GPS→MAP: ({lat:.6f}, {lng:.6f}) → ({x:.2f}, {y:.2f})")

            return {'x': x, 'y': y}

        except Exception as e:
            self.logger.error(f"GPS→MAP変換エラー: {e}")
            return {'x': 0.0, 'y': 0.0}

    def map_to_gps_coordinates(self, x: float, y: float) -> Dict[str, float]:
        """
        ROS2マップ座標をGPS座標に変換

        Args:
            x: マップ座標X (メートル)
            y: マップ座標Y (メートル)

        Returns:
            GPS座標 {'lat': float, 'lng': float}
        """
        try:
            R = 6371000  # 地球半径(m)

            # メートルから緯度・経度に変換
            lat = self.origin_lat + math.degrees(y / R)
            lng = self.origin_lng + math.degrees(x / (R * math.cos(math.radians(self.origin_lat))))

            self.logger.debug(f"MAP→GPS: ({x:.2f}, {y:.2f}) → ({lat:.6f}, {lng:.6f})")

            return {'lat': lat, 'lng': lng}

        except Exception as e:
            self.logger.error(f"MAP→GPS変換エラー: {e}")
            return {'lat': self.origin_lat, 'lng': self.origin_lng}

    def create_pose_stamped(self, lat: float, lng: float,
                          frame_id: str = "map", yaw: float = 0.0) -> PoseStamped:
        """
        GPS座標からPoseStampedメッセージを作成

        Args:
            lat: 緯度
            lng: 経度
            frame_id: フレームID
            yaw: 回転角度（ラジアン）

        Returns:
            PoseStampedメッセージ
        """
        try:
            map_coords = self.gps_to_map_coordinates(lat, lng)

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = rclpy.clock.Clock().now().to_msg()

            # 位置設定
            pose.pose.position.x = map_coords['x']
            pose.pose.position.y = map_coords['y']
            pose.pose.position.z = 0.0

            # 向き設定（クォータニオン）
            quat = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            self.logger.debug(f"PoseStamped作成: ({lat:.6f}, {lng:.6f}) → MAP({map_coords['x']:.2f}, {map_coords['y']:.2f})")

            return pose

        except Exception as e:
            self.logger.error(f"PoseStamped作成エラー: {e}")
            # エラー時は原点を返す
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = rclpy.clock.Clock().now().to_msg()
            return pose

    def calculate_distance(self, lat1: float, lng1: float,
                         lat2: float, lng2: float) -> float:
        """
        2点間のGPS座標の距離を計算（ハバーサイン公式）

        Args:
            lat1, lng1: 地点1の緯度・経度
            lat2, lng2: 地点2の緯度・経度

        Returns:
            距離（メートル）
        """
        try:
            R = 6371000  # 地球半径(m)

            dlat = math.radians(lat2 - lat1)
            dlng = math.radians(lng2 - lng1)

            a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
                 math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
                 math.sin(dlng / 2) * math.sin(dlng / 2))

            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance = R * c

            return distance

        except Exception as e:
            self.logger.error(f"距離計算エラー: {e}")
            return 0.0