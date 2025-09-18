#!/usr/bin/env python3
"""Firebase-ROS2 Bridge メインノード"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2メッセージ型
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# アクション関連
from rclpy.action import ActionClient

from ros2_firebase_bridge.firebase_client import FirebaseClient
from ros2_firebase_bridge.coordinate_converter import CoordinateConverter

import yaml
import os
import logging
import threading
import time
from typing import Dict, Any, Optional

class FirebaseBridgeNode(Node):
    """Firebase-ROS2ブリッジのメインノード"""

    def __init__(self):
        super().__init__('firebase_bridge_node')

        # ロギング設定
        self.setup_logging()

        # 設定読み込み
        self.config = self.load_config()

        # コールバックグループ設定（並行処理のため）
        self.callback_group = ReentrantCallbackGroup()

        # Firebase接続
        self.firebase_client = None
        self.coordinate_converter = None

        # ROS2通信設定
        self.setup_ros2_interfaces()

        # 状態管理
        self.robot_states = {}  # ロボット状態を追跡
        self.active_navigation = {}  # アクティブなナビゲーション

        # Firebase初期化（非同期）
        self.firebase_init_timer = self.create_timer(
            1.0, self.initialize_firebase, callback_group=self.callback_group
        )

        self.get_logger().info("🚀 Firebase Bridge Node が開始されました")

    def setup_logging(self):
        """ロギングを設定"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(self.__class__.__name__)

    def load_config(self) -> Dict[str, Any]:
        """設定ファイルを読み込み"""
        try:
            config_path = '/workspace/src/ros2_firebase_bridge/config/firebase_config.yaml'

            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                self.get_logger().info(f"設定ファイル読み込み完了: {config_path}")
                return config
            else:
                self.get_logger().warning(f"設定ファイルが見つかりません: {config_path}")
                return self.get_default_config()

        except Exception as e:
            self.get_logger().error(f"設定ファイル読み込みエラー: {e}")
            return self.get_default_config()

    def get_default_config(self) -> Dict[str, Any]:
        """デフォルト設定を返す"""
        return {
            'firebase': {
                'service_account_key': '/workspace/config/serviceAccountKey.json'
            },
            'ros2': {
                'robot_namespace': '/turtlebot3',
                'goal_topic': '/goal_pose',
                'odom_topic': '/odom',
                'status_topic': '/robot_status'
            },
            'coordinate_system': {
                'origin_latitude': 36.5598,
                'origin_longitude': 139.9088,
                'map_frame': 'map',
                'base_frame': 'base_link'
            }
        }

    def setup_ros2_interfaces(self):
        """ROS2のパブリッシャー・サブスクライバーを設定"""
        try:
            # ゴール位置のパブリッシャー
            self.goal_publisher = self.create_publisher(
                PoseStamped,
                '/goal_pose',
                10,
                callback_group=self.callback_group
            )

            # Navigation2アクションクライアント
            self.nav_action_client = ActionClient(
                self,
                NavigateToPose,
                '/navigate_to_pose',
                callback_group=self.callback_group
            )

            # オドメトリサブスクライバー（ロボット位置取得）
            self.odom_subscriber = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10,
                callback_group=self.callback_group
            )

            # 初期位置サブスクライバー（ロボット初期位置取得）
            self.initialpose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped,
                '/initialpose',
                self.initialpose_callback,
                10,
                callback_group=self.callback_group
            )

            # ステータスパブリッシャー
            self.status_publisher = self.create_publisher(
                String,
                '/robot_status',
                10,
                callback_group=self.callback_group
            )

            self.get_logger().info("✅ ROS2インターフェース設定完了")

        except Exception as e:
            self.get_logger().error(f"ROS2インターフェース設定エラー: {e}")

    def initialize_firebase(self):
        """Firebase接続を初期化"""
        try:
            if self.firebase_client is None:
                service_account_path = self.config['firebase']['service_account_key']

                # Firebase接続
                self.firebase_client = FirebaseClient(service_account_path, self.logger)

                # 座標変換器初期化
                coord_config = self.config['coordinate_system']
                self.coordinate_converter = CoordinateConverter(
                    origin_lat=coord_config['origin_latitude'],
                    origin_lng=coord_config['origin_longitude'],
                    logger=self.logger
                )

                # Firestoreリアルタイム監視開始
                self.firebase_client.setup_realtime_listener(
                    'robots',
                    self.on_firestore_robot_update
                )

                self.get_logger().info("🔥 Firebase接続完了 - リアルタイム監視開始")

                # 初期化タイマーを停止
                self.firebase_init_timer.cancel()

        except Exception as e:
            self.get_logger().error(f"Firebase初期化エラー: {e}")
            # 5秒後に再試行
            time.sleep(5.0)

    def on_firestore_robot_update(self, robot_id: str, robot_data: Dict[str, Any], change_type: str):
        """Firestoreのロボットデータ更新時のコールバック"""
        try:
            self.get_logger().info(f"🔄 Firestore更新検知: {robot_id} ({change_type})")

            # 前回の状態と比較
            prev_state = self.robot_states.get(robot_id, {})
            self.robot_states[robot_id] = robot_data

            # 目的地が設定された場合の処理
            if self.should_send_navigation_goal(robot_data, prev_state):
                self.send_navigation_goal(robot_id, robot_data)

            # ステータス変更の通知
            if robot_data.get('status') != prev_state.get('status'):
                self.publish_robot_status(robot_id, robot_data.get('status', 'unknown'))

        except Exception as e:
            self.get_logger().error(f"Firestore更新処理エラー: {e}")

    def should_send_navigation_goal(self, current_data: Dict[str, Any],
                                  prev_data: Dict[str, Any]) -> bool:
        """ナビゲーションゴールを送信すべきかどうか判定"""
        try:
            # 目的地が設定されているか
            if 'destination' not in current_data:
                return False

            # 適切なステータスかどうか
            valid_statuses = ['配車中', '走行中']
            if current_data.get('status') not in valid_statuses:
                return False

            # 前回と目的地が変わったかどうか
            current_dest = current_data['destination']
            prev_dest = prev_data.get('destination')

            if prev_dest is None:
                return True  # 新しく目的地が設定された

            # 座標が変わったかチェック
            if (abs(current_dest.latitude - prev_dest.latitude) > 0.00001 or
                abs(current_dest.longitude - prev_dest.longitude) > 0.00001):
                return True

            return False

        except Exception as e:
            self.get_logger().error(f"ナビゲーション判定エラー: {e}")
            return False

    def send_navigation_goal(self, robot_id: str, robot_data: Dict[str, Any]):
        """ROS2にナビゲーションゴールを送信"""
        try:
            destination = robot_data['destination']
            self.get_logger().info(
                f"🎯 ナビゲーションゴール送信: {robot_id} → "
                f"({destination.latitude:.6f}, {destination.longitude:.6f})"
            )

            # PoseStampedメッセージ作成
            goal_pose = self.coordinate_converter.create_pose_stamped(
                destination.latitude,
                destination.longitude,
                frame_id='map'
            )

            # 2つの方法でゴールを送信

            # 1. 簡単なgoal_poseトピックに発行（RViz用）
            self.goal_publisher.publish(goal_pose)

            # 2. Navigation2アクションに送信
            if self.nav_action_client.wait_for_server(timeout_sec=1.0):
                self.send_nav2_action_goal(robot_id, goal_pose)
            else:
                self.get_logger().warning("Navigation2アクションサーバーが利用できません")

        except Exception as e:
            self.get_logger().error(f"ナビゲーションゴール送信エラー: {e}")

    def send_nav2_action_goal(self, robot_id: str, goal_pose: PoseStamped):
        """Navigation2アクションゴールを送信"""
        try:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose

            # アクション送信
            future = self.nav_action_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self.navigation_feedback_callback(robot_id, feedback)
            )

            future.add_done_callback(
                lambda f: self.navigation_goal_response_callback(robot_id, f)
            )

            self.active_navigation[robot_id] = future

        except Exception as e:
            self.get_logger().error(f"Nav2アクションゴール送信エラー: {e}")

    def navigation_goal_response_callback(self, robot_id: str, future):
        """Navigation2ゴール応答コールバック"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"ナビゲーションゴールが拒否されました: {robot_id}")
                return

            self.get_logger().info(f"ナビゲーションゴール受付: {robot_id}")

            # 結果を待機
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.navigation_result_callback(robot_id, f)
            )

        except Exception as e:
            self.get_logger().error(f"ナビゲーションゴール応答エラー: {e}")

    def navigation_feedback_callback(self, robot_id: str, feedback):
        """Navigation2フィードバックコールバック"""
        try:
            # フィードバックから現在位置を取得（必要に応じて）
            current_pose = feedback.feedback.current_pose
            self.get_logger().debug(f"ナビゲーション進行中: {robot_id}")

        except Exception as e:
            self.get_logger().error(f"ナビゲーションフィードバックエラー: {e}")

    def navigation_result_callback(self, robot_id: str, future):
        """Navigation2結果コールバック"""
        try:
            result = future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"🎉 ナビゲーション完了: {robot_id}")
                # Firestoreのステータスを更新（必要に応じて）

            else:
                self.get_logger().warning(f"ナビゲーション失敗: {robot_id}, ステータス: {result.status}")

            # アクティブナビゲーションから削除
            if robot_id in self.active_navigation:
                del self.active_navigation[robot_id]

        except Exception as e:
            self.get_logger().error(f"ナビゲーション結果処理エラー: {e}")

    def odom_callback(self, msg: Odometry):
        """オドメトリコールバック（ロボット位置更新）"""
        try:
            # マップ座標をGPS座標に変換
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            gps_coords = self.coordinate_converter.map_to_gps_coordinates(x, y)

            # 現在のロボットIDを取得（実際の実装では適切に設定）
            robot_id = self.get_current_robot_id()

            if robot_id and self.firebase_client:
                # 位置更新の頻度を制御（1秒に1回程度）
                if not hasattr(self, '_last_position_update'):
                    self._last_position_update = {}

                current_time = time.time()
                if (robot_id not in self._last_position_update or
                    current_time - self._last_position_update[robot_id] > 1.0):

                    # Firestoreに位置を更新
                    current_status = self.robot_states.get(robot_id, {}).get('status', '走行中')
                    self.firebase_client.update_robot_status(
                        robot_id,
                        gps_coords,
                        current_status
                    )

                    self._last_position_update[robot_id] = current_time

        except Exception as e:
            self.get_logger().error(f"オドメトリ処理エラー: {e}")

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """初期位置コールバック"""
        try:
            self.get_logger().info("ロボット初期位置が設定されました")

            # 初期位置もFirestoreに反映する場合
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            gps_coords = self.coordinate_converter.map_to_gps_coordinates(x, y)
            robot_id = self.get_current_robot_id()

            if robot_id and self.firebase_client:
                self.firebase_client.update_robot_status(
                    robot_id,
                    gps_coords,
                    'アイドリング中'
                )

        except Exception as e:
            self.get_logger().error(f"初期位置処理エラー: {e}")

    def publish_robot_status(self, robot_id: str, status: str):
        """ロボットステータスをROS2トピックに発行"""
        try:
            status_msg = String()
            status_msg.data = f"{robot_id}:{status}"
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"ステータス発行エラー: {e}")

    def get_current_robot_id(self) -> Optional[str]:
        """現在のロボットIDを取得"""
        # 実際の実装では、以下の方法でロボットIDを特定する必要があります：
        # 1. 起動パラメータから取得
        # 2. robot_descriptionから取得
        # 3. 固定値として設定

        # 今回はテスト用に固定値を返す
        return "robot_001"  # 実装時は適切に変更してください

    def shutdown(self):
        """シャットダウン処理"""
        try:
            self.get_logger().info("Firebase Bridge Node をシャットダウンしています...")

            if self.firebase_client:
                self.firebase_client.close_listeners()

            # アクティブなナビゲーションをキャンセル
            for robot_id, future in self.active_navigation.items():
                try:
                    if not future.done():
                        future.cancel()
                except Exception as e:
                    self.get_logger().error(f"ナビゲーションキャンセルエラー {robot_id}: {e}")

            self.get_logger().info("シャットダウン完了")

        except Exception as e:
            self.get_logger().error(f"シャットダウンエラー: {e}")

def main(args=None):
    """メイン関数"""
    rclpy.init(args=args)

    try:
        # マルチスレッドエグゼキュータを使用
        executor = MultiThreadedExecutor()

        # ノード作成
        node = FirebaseBridgeNode()

        # エグゼキュータにノードを追加
        executor.add_node(node)

        try:
            # ノード実行
            executor.spin()

        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt を受信しました")

        finally:
            # クリーンアップ
            node.shutdown()
            node.destroy_node()

    except Exception as e:
        print(f"メイン関数エラー: {e}")

    finally:
        # ROS2終了処理
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"ROS2シャットダウンエラー: {e}")

if __name__ == '__main__':
    main()