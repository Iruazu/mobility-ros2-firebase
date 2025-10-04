"""GPS座標とROS2マップ座標の変換ユーティリティ - 実機対応版"""

import math
import logging
from typing import Dict, Tuple, Optional
from geometry_msgs.msg import PoseStamped, Quaternion
import rclpy
from rclpy.clock import Clock


class CoordinateConverter:
    """
    実機・シミュレーション両対応の座標変換システム
    - TurtleBot3のマップ範囲に自動適応
    - スケールファクターによる柔軟な調整
    - 範囲外座標の自動補正
    """

    def __init__(self, origin_lat: float = 36.5598, origin_lng: float = 139.9088,
                 scale_factor: float = 0.01, logger=None):
        """
        座標変換ユーティリティを初期化

        Args:
            origin_lat: マップ座標系の原点緯度
            origin_lng: マップ座標系の原点経度
            scale_factor: GPS距離をマップ距離に変換する係数（デフォルト: 0.01 = 1/100）
            logger: ロガーインスタンス
        """
        self.origin_lat = origin_lat
        self.origin_lng = origin_lng
        self.scale_factor = scale_factor
        self.logger = logger or logging.getLogger(__name__)

        # TurtleBot3マップの安全範囲（メートル）
        # turtlebot3_world.world のマップサイズに基づく
        self.map_bounds = {
            'x_min': -4.0,
            'x_max': 4.0,
            'y_min': -4.0,
            'y_max': 4.0
        }

        self.logger.info(
            f"🗺️ 座標系初期化:\n"
            f"   原点: ({origin_lat:.6f}, {origin_lng:.6f})\n"
            f"   スケール係数: {scale_factor}\n"
            f"   マップ範囲: X[{self.map_bounds['x_min']}, {self.map_bounds['x_max']}], "
            f"Y[{self.map_bounds['y_min']}, {self.map_bounds['y_max']}]"
        )

    def gps_to_map_coordinates(self, lat: float, lng: float) -> Dict[str, float]:
        """
        GPS座標をROS2マップ座標系に変換（スケール・範囲補正付き）

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

            # メートル単位に変換（生の距離）
            x_raw = dlng * R * math.cos(math.radians(self.origin_lat))
            y_raw = dlat * R

            # スケールファクター適用
            x = x_raw * self.scale_factor
            y = y_raw * self.scale_factor

            self.logger.debug(
                f"📍 GPS→MAP: ({lat:.6f}, {lng:.6f}) → "
                f"生座標({x_raw:.2f}, {y_raw:.2f}) → "
                f"スケール後({x:.2f}, {y:.2f})"
            )

            # 範囲チェックと補正
            if not self.is_within_bounds(x, y):
                self.logger.warning(
                    f"⚠️ 座標が範囲外: ({x:.2f}, {y:.2f})"
                )
                x_clamped, y_clamped = self.clamp_to_bounds(x, y)
                self.logger.warning(
                    f"🔧 自動補正: ({x:.2f}, {y:.2f}) → ({x_clamped:.2f}, {y_clamped:.2f})"
                )
                return {'x': x_clamped, 'y': y_clamped}

            return {'x': x, 'y': y}

        except Exception as e:
            self.logger.error(f"❌ GPS→MAP変換エラー: {e}")
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

            # スケール逆適用
            x_real = x / self.scale_factor
            y_real = y / self.scale_factor

            # GPS座標に変換
            lat = self.origin_lat + math.degrees(y_real / R)
            lng = self.origin_lng + math.degrees(
                x_real / (R * math.cos(math.radians(self.origin_lat)))
            )

            self.logger.debug(f"MAP→GPS: ({x:.2f}, {y:.2f}) → ({lat:.6f}, {lng:.6f})")

            return {'lat': lat, 'lng': lng}

        except Exception as e:
            self.logger.error(f"❌ MAP→GPS変換エラー: {e}")
            return {'lat': self.origin_lat, 'lng': self.origin_lng}

    def is_within_bounds(self, x: float, y: float) -> bool:
        """座標がマップ範囲内かチェック"""
        return (
            self.map_bounds['x_min'] <= x <= self.map_bounds['x_max'] and
            self.map_bounds['y_min'] <= y <= self.map_bounds['y_max']
        )

    def clamp_to_bounds(self, x: float, y: float) -> Tuple[float, float]:
        """座標をマップ範囲内に収める"""
        # 安全マージンを考慮（境界から0.5m内側に制限）
        margin = 0.5
        x_clamped = max(
            self.map_bounds['x_min'] + margin,
            min(x, self.map_bounds['x_max'] - margin)
        )
        y_clamped = max(
            self.map_bounds['y_min'] + margin,
            min(y, self.map_bounds['y_max'] - margin)
        )
        return (x_clamped, y_clamped)

    def create_pose_stamped(self, lat: float, lng: float,
                          frame_id: str = "map", yaw: float = 0.0) -> PoseStamped:
        """
        GPS座標からPoseStampedメッセージを作成（範囲チェック付き）

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
            pose.header.stamp = Clock().now().to_msg()

            # 位置設定
            pose.pose.position.x = map_coords['x']
            pose.pose.position.y = map_coords['y']
            pose.pose.position.z = 0.0

            # 向き設定（クォータニオン）
            quat = self._euler_to_quaternion(0, 0, yaw)
            pose.pose.orientation = quat

            self.logger.info(
                f"✅ PoseStamped作成:\n"
                f"   GPS: ({lat:.6f}, {lng:.6f})\n"
                f"   MAP: ({map_coords['x']:.2f}, {map_coords['y']:.2f}, yaw={yaw:.2f})"
            )

            return pose

        except Exception as e:
            self.logger.error(f"❌ PoseStamped作成エラー: {e}")
            # エラー時は原点を返す
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = Clock().now().to_msg()
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            return pose

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """オイラー角をクォータニオンに変換"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

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
            self.logger.error(f"❌ 距離計算エラー: {e}")
            return 0.0

    def validate_goal(self, lat: float, lng: float) -> Tuple[bool, str]:
        """
        ゴール座標が有効かどうかチェック

        Args:
            lat: 緯度
            lng: 経度

        Returns:
            (有効かどうか, エラーメッセージ)
        """
        map_coords = self.gps_to_map_coordinates(lat, lng)

        # 原点からの距離チェック
        distance_from_origin = math.sqrt(
            map_coords['x']**2 + map_coords['y']**2
        )

        max_distance = math.sqrt(
            self.map_bounds['x_max']**2 + self.map_bounds['y_max']**2
        )

        if distance_from_origin > max_distance:
            msg = f"ゴールが原点から遠すぎます: {distance_from_origin:.2f}m (上限: {max_distance:.2f}m)"
            self.logger.warning(f"⚠️ {msg}")
            return (False, msg)

        # マップ範囲内チェック
        if not self.is_within_bounds(map_coords['x'], map_coords['y']):
            msg = f"ゴールがマップ範囲外: ({map_coords['x']:.2f}, {map_coords['y']:.2f})"
            self.logger.warning(f"⚠️ {msg}")
            return (False, msg)

        self.logger.info(f"✅ ゴール座標は有効: MAP({map_coords['x']:.2f}, {map_coords['y']:.2f})")
        return (True, "OK")

    def get_safe_goal_near(self, lat: float, lng: float) -> Dict[str, float]:
        """
        範囲外の座標に対して、最も近い安全な座標を返す

        Args:
            lat: 緯度
            lng: 経度

        Returns:
            安全な座標 {'lat': float, 'lng': float}
        """
        map_coords = self.gps_to_map_coordinates(lat, lng)
        x_safe, y_safe = self.clamp_to_bounds(map_coords['x'], map_coords['y'])
        return self.map_to_gps_coordinates(x_safe, y_safe)