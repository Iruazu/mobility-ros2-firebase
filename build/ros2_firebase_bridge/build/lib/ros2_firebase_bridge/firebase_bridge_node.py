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
import hashlib
import threading
from typing import Dict, Any, Optional


class Phase2FirebaseBridge(Node):
    """
    Phase 2完全版 Firebase-ROS2 Bridge

    主要改善点:
    1. 無限ループ完全防止 (丸め誤差対応 + 処理ロック)
    2. 位置同期最適化 (移動距離ベース + 動的間隔調整)
    3. 複数ロボット対応 (launch引数でrobot_id設定可能)
    """

    def __init__(self):
        super().__init__('phase2_firebase_bridge')

        # ===== Phase 2: 複数ロボット対応 =====
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('robot_namespace', '/turtlebot3')

        self.robot_id = self.get_parameter('robot_id').value
        self.robot_namespace = self.get_parameter('robot_namespace').value

        self.get_logger().info(f"🤖 Robot ID: {self.robot_id}")
        self.get_logger().info(f"📡 Namespace: {self.robot_namespace}")

        self.config = self.load_config()
        self.callback_group = ReentrantCallbackGroup()

        # Core components
        self.firebase_client = None
        self.coordinate_converter = None
        self.state_publisher = None
        self.state_tracker = None
        self.sensor_aggregator = None

        # Robot state
        self.current_goal = None
        self.navigation_active = False
        self.goal_handle = None

        # ===== Phase 2改善: 無限ループ防止の強化 =====
        self.destination_lock = threading.Lock()  # 🔒 スレッドセーフな処理
        self.last_processed_destination_hash = None
        self.processing_navigation = False
        self.destination_processing_count = {}  # 重複検知カウンター

        # Phase 2改善: 位置同期最適化
        self.last_position_update_time = 0.0
        self.last_published_position = None
        self.position_threshold = 0.5  # 0.5m以上移動で更新
        self.update_interval_moving = 1.0  # 移動中: 1秒
        self.update_interval_idle = 5.0     # 停止中: 5秒

        self.setup_ros2_interfaces()
        self.firebase_init_timer = self.create_timer(
            1.0, self.initialize_firebase, callback_group=self.callback_group
        )

        self.get_logger().info("🚀 Phase 2 Firebase Bridge started")

    def load_config(self) -> Dict[str, Any]:
        """設定ファイル読み込み"""
        try:
            config_paths = [
                '/workspace/config/rviz/firebase_config.yaml',
                os.path.join(os.path.dirname(__file__), '..', 'config', 'firebase_config.yaml'),
            ]

            for path in config_paths:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        return yaml.safe_load(f)

            return self.get_default_config()
        except Exception as e:
            self.get_logger().error(f"Config load error: {e}")
            return self.get_default_config()

    def get_default_config(self) -> Dict[str, Any]:
        """デフォルト設定"""
        return {
            'firebase': {
                'service_account_key': '/workspace/config/serviceAccountKey.json'
            },
            'ros2': {
                'robot_namespace': self.robot_namespace,
                'odom_topic': f'{self.robot_namespace}/odom',
            },
            'coordinate_system': {
                'origin_latitude': 36.55077,
                'origin_longitude': 139.92957,
                'map_frame': 'map',
                'scale_factor': 0.01
            }
        }

    def setup_ros2_interfaces(self):
        """ROS2インターフェース設定"""
        try:
            # トピック名をnamespace対応に変更
            odom_topic = f'{self.robot_namespace}/odom'
            scan_topic = f'{self.robot_namespace}/scan'
            goal_topic = f'{self.robot_namespace}/goal_pose'
            nav_action = f'{self.robot_namespace}/navigate_to_pose'

            self.goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
            self.nav_action_client = ActionClient(self, NavigateToPose, nav_action)

            self.odom_subscriber = self.create_subscription(
                Odometry, odom_topic, self.odom_callback, 10
            )
            self.scan_subscriber = self.create_subscription(
                LaserScan, scan_topic, self.scan_callback, 10
            )

            self.get_logger().info("✅ ROS2 interfaces configured")
        except Exception as e:
            self.get_logger().error(f"ROS2 interface setup error: {e}")

    def initialize_firebase(self):
        """Firebase接続初期化"""
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
                scale_factor=coord_config.get('scale_factor', 0.01),
                logger=self.get_logger()
            )

            self.state_publisher = StatePublisher(
                self.firebase_client, self.coordinate_converter, self.get_logger()
            )
            self.state_tracker = RobotStateTracker(self, self.state_publisher)
            self.sensor_aggregator = SensorAggregator(
                self, self.robot_id, self.firebase_client, self.get_logger()
            )

            # Firestoreリスナー設定
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
        """Firebaseにロボット情報を初期化"""
        try:
            robot_data = self.firebase_client.get_robot_data(self.robot_id)
            if robot_data is None:
                initial_data = {
                    'id': self.robot_id,
                    'name': f'TurtleBot3-{self.robot_id[-3:]}',
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
                self.get_logger().info(f"✅ Robot {self.robot_id} initialized in Firebase")
        except Exception as e:
            self.get_logger().error(f"Robot initialization error: {e}")

    def calculate_destination_hash(self, destination) -> str:
        """
        Phase 2改善: 丸め誤差に強いハッシュ計算
        小数点5桁(約1.1m精度)に丸めることで、Firebase往復での微小誤差を吸収
        """
        if destination is None:
            return None

        lat_rounded = round(destination.latitude, 5)
        lng_rounded = round(destination.longitude, 5)
        hash_input = f"{lat_rounded:.5f}_{lng_rounded:.5f}"

        return hashlib.md5(hash_input.encode()).hexdigest()

    def on_firestore_update(self, robot_id: str, robot_data: Dict[str, Any], change_type: str):
        """
        Phase 2改善: 無限ループ完全防止版 Firestore更新ハンドラ

        防止機構:
        1. スレッドロックによる排他制御
        2. 丸め誤差に強いハッシュ比較
        3. 処理カウンターによる異常検知
        """
        try:
            if robot_id != self.robot_id:
                return

            # 1️⃣ ナビゲーション処理中は無視
            if self.processing_navigation:
                self.get_logger().debug("⏸️ Navigation processing, skipping")
                return

            # 2️⃣ destinationの存在確認
            if 'destination' not in robot_data or not robot_data['destination']:
                return

            destination = robot_data['destination']

            # 3️⃣ 🔒 スレッドロックで排他制御
            with self.destination_lock:
                new_hash = self.calculate_destination_hash(destination)

                # 4️⃣ ハッシュ比較による重複検知
                if new_hash == self.last_processed_destination_hash:
                    self.get_logger().debug(f"⏸️ Duplicate destination hash: {new_hash[:8]}")
                    return

                # 5️⃣ 異常な重複回数チェック
                if new_hash not in self.destination_processing_count:
                    self.destination_processing_count[new_hash] = 0

                self.destination_processing_count[new_hash] += 1

                if self.destination_processing_count[new_hash] > 3:
                    self.get_logger().error(
                        f"🚨 無限ループ検知! Hash {new_hash[:8]} が "
                        f"{self.destination_processing_count[new_hash]}回処理されました"
                    )
                    return

                # 6️⃣ 新しいdestinationとして処理
                self.last_processed_destination_hash = new_hash

                self.get_logger().info(
                    f"🎯 New destination: ({destination.latitude:.6f}, {destination.longitude:.6f}) "
                    f"[Hash: {new_hash[:8]}]"
                )

            # 7️⃣ ナビゲーションゴール送信
            self.send_navigation_goal(destination)

        except Exception as e:
            self.get_logger().error(f"Firestore update error: {e}")

    def send_navigation_goal(self, destination):
        """Phase 2: ゴール検証強化版"""
        try:
            self.processing_navigation = True

            # ゴール有効性検証
            is_valid, error_msg = self.coordinate_converter.validate_goal(
                destination.latitude, destination.longitude
            )

            if not is_valid:
                self.get_logger().error(f"❌ Invalid goal: {error_msg}")

                # 最も近い安全な座標に補正
                safe_goal = self.coordinate_converter.get_safe_goal_near(
                    destination.latitude, destination.longitude
                )
                self.get_logger().warning(
                    f"🔧 Corrected to safe goal: ({safe_goal['lat']:.6f}, {safe_goal['lng']:.6f})"
                )

                goal_pose = self.coordinate_converter.create_pose_stamped(
                    safe_goal['lat'], safe_goal['lng'], frame_id='map'
                )
            else:
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
        """Nav2ゴールレスポンス処理"""
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
        """Nav2フィードバック処理(現在は何もしない)"""
        pass

    def nav_result_callback(self, future):
        """
        Phase 2改善: ナビゲーション完了処理
        destinationを確実に削除して無限ループを防止
        """
        try:
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("🎉 Navigation completed successfully")
            else:
                self.get_logger().warning(f"⚠️ Navigation failed: {status}")

            # 🚨 destinationを削除してループ防止
            with self.destination_lock:
                self.firebase_client.update_robot_state(
                    self.robot_id,
                    {
                        'destination': firestore.DELETE_FIELD,
                        'status': 'idle'
                    }
                )

                # 処理状態をリセット
                self.navigation_active = False
                self.current_goal = None
                self.goal_handle = None
                self.last_processed_destination_hash = None

                self.get_logger().info("✅ Destination cleared, ready for next task")

        except Exception as e:
            self.get_logger().error(f"Nav result error: {e}")

    def cancel_navigation(self):
        """ナビゲーションキャンセル"""
        try:
            if self.goal_handle:
                self.get_logger().info("🛑 Cancelling navigation")
                self.goal_handle.cancel_goal_async()

            with self.destination_lock:
                self.navigation_active = False
                self.current_goal = None
                self.goal_handle = None
                self.processing_navigation = False
                self.last_processed_destination_hash = None

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
        """
        Phase 2改善: 位置同期最適化版 Odometry コールバック

        改善点:
        1. 移動距離ベースの更新判定
        2. 動的な更新間隔調整(移動中1秒/停止中5秒)
        """
        if not self.state_tracker or not self.sensor_aggregator:
            return

        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            current_position = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            }

            # 速度計算
            speed = math.sqrt(
                msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
            )

            # 動的な更新間隔決定
            is_moving = speed > 0.05  # 0.05 m/s以上で移動中と判定
            update_interval = self.update_interval_moving if is_moving else self.update_interval_idle

            # 時間ベースのレート制限
            time_since_update = current_time - self.last_position_update_time
            if time_since_update < update_interval:
                # 速度情報だけ更新
                self.sensor_aggregator.update_motion_state(speed)
                return

            # 移動距離ベースのチェック
            should_update = False

            if self.last_published_position is None:
                should_update = True
            else:
                dx = current_position['x'] - self.last_published_position['x']
                dy = current_position['y'] - self.last_published_position['y']
                distance = math.sqrt(dx**2 + dy**2)

                if distance >= self.position_threshold:
                    should_update = True
                    self.get_logger().debug(
                        f"📍 Moved {distance:.2f}m (threshold: {self.position_threshold}m)"
                    )

            # 位置更新実行
            if should_update:
                self.state_tracker.update_from_odom(self.robot_id, msg)
                self.last_position_update_time = current_time
                self.last_published_position = current_position

            # センサー情報は常に更新
            self.sensor_aggregator.update_motion_state(speed)

            # ナビゲーション状態更新
            if self.current_goal and self.navigation_active:
                current_pose = PoseStamped()
                current_pose.pose = msg.pose.pose
                goal_pose = self.coordinate_converter.create_pose_stamped(
                    self.current_goal.latitude, self.current_goal.longitude
                )
                self.sensor_aggregator.update_navigation_state(goal_pose, current_pose)
            else:
                current_pose = PoseStamped()
                current_pose.pose = msg.pose.pose
                self.sensor_aggregator.update_navigation_state(None, current_pose)

        except Exception as e:
            self.get_logger().error(f"Odom callback error: {e}")

    def scan_callback(self, msg: LaserScan):
        """LiDARスキャン処理"""
        if self.sensor_aggregator:
            self.sensor_aggregator.scan_callback(msg)

    def battery_callback(self, msg: BatteryState):
        """バッテリー状態処理"""
        if self.sensor_aggregator:
            self.sensor_aggregator.battery_callback(msg)

    def telemetry_publish_callback(self):
        """テレメトリ定期送信"""
        if self.sensor_aggregator:
            self.sensor_aggregator.publish_telemetry_check()

    def shutdown(self):
        """クリーンシャットダウン"""
        try:
            self.get_logger().info("🛑 Shutting down Phase 2 Bridge")

            if self.firebase_client:
                self.firebase_client.close_listeners()

            if self.navigation_active:
                self.cancel_navigation()

        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")


def main(args=None):
    """メインエントリーポイント"""
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        node = Phase2FirebaseBridge()
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