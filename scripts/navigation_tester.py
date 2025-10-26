#!/usr/bin/env python3
"""
実践的な配車テストツール
Web配車の実際の動作を確認
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import sys
import time


class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')

        # 購読
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # 発行
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )

        self.current_position = None
        self.get_logger().info('Navigation Tester Ready!')

    def odom_callback(self, msg):
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }

        # 方位角を計算
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_yaw = yaw

    def send_test_goal(self, x, y, description=""):
        """テスト用の目標を送信"""
        if self.current_position is None:
            self.get_logger().warn('現在位置が不明です。少し待ってから再試行してください。')
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        goal.pose.orientation.w = 1.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0

        self.goal_pub.publish(goal)

        # 計算
        dx = x - self.current_position['x']
        dy = y - self.current_position['y']
        distance = math.sqrt(dx**2 + dy**2)
        direction = math.degrees(math.atan2(dy, dx))

        print("\n" + "="*60)
        print(f"📍 テスト配車: {description}")
        print("="*60)
        print(f"現在位置: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f})")
        print(f"目標位置: ({x:.2f}, {y:.2f})")
        print(f"移動距離: {distance:.2f}m")
        print(f"移動方向: {direction:.1f}° (0°=東, 90°=北)")
        print("="*60)
        print("\n💡 RVizで以下を確認:")
        print("  1. Global Path (緑の線) が正しい方向に引かれているか")
        print("  2. ロボット (TurtleBot3) が動き始めたか")
        print("  3. 目標地点にマーカーが表示されているか")
        print("\n")

    def run_test_sequence(self):
        """4方向テストシーケンス"""
        print("""
╔══════════════════════════════════════════════════════════╗
║        配車動作テスト（4方向）                            ║
║  各方向にロボットが正しく進むか確認します                ║
╚══════════════════════════════════════════════════════════╝
        """)

        time.sleep(2)  # 初期化待機

        # 現在位置確認
        if self.current_position is None:
            print("❌ エラー: 現在位置が取得できません")
            print("   /odom トピックが配信されているか確認してください")
            return

        print(f"\n✅ 現在位置: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f})")
        print(f"✅ 現在の向き: {math.degrees(self.current_yaw):.1f}°\n")

        tests = [
            (1.0, 0.0, "東へ1m"),
            (0.0, 1.0, "北へ1m"),
            (-1.0, 0.0, "西へ1m"),
            (0.0, -1.0, "南へ1m"),
        ]

        print("以下のテストを実行します:")
        for i, (x, y, desc) in enumerate(tests, 1):
            print(f"  {i}. {desc} → Map座標 ({x:.1f}, {y:.1f})")

        print("\n各テストの間に30秒の間隔があります。")
        print("ロボットが目標に到達したら、次のテストに進みます。")

        input("\nEnterキーを押して開始...")

        for i, (x, y, desc) in enumerate(tests, 1):
            print(f"\n\n{'='*60}")
            print(f"テスト {i}/4: {desc}")
            print('='*60)

            self.send_test_goal(x, y, desc)

            if i < len(tests):
                print(f"\n⏰ 30秒後に次のテスト ({tests[i][2]}) を実行します...")
                print("   (Ctrl+Cでキャンセル)")
                time.sleep(30)

        print("\n\n" + "="*60)
        print("✅ すべてのテストが完了しました")
        print("="*60)
        print("\n📊 結果確認:")
        print("  各テストで、ロボットが目標方向に進んだかチェックしてください。")
        print("  もし違う方向に進んだ場合、以下を確認:")
        print("    1. RVizのGlobal Pathが正しい方向を指しているか")
        print("    2. AMCLのパーティクル（緑の矢印）が収束しているか")
        print("    3. Gazeboとの座標系が一致しているか")


def main(args=None):
    rclpy.init(args=args)

    tester = NavigationTester()

    print("""
╔══════════════════════════════════════════════════════════╗
║           配車動作テストツール                            ║
║  使い方:                                                  ║
║    1. Gazebo, Nav2, Firebase Bridgeを起動しておく        ║
║    2. RVizを開いて視覚確認できる状態にする                ║
║    3. このスクリプトを実行                                ║
╚══════════════════════════════════════════════════════════╝
    """)

    # 少し待機してトピック接続
    print("⏳ ROSトピック接続待機中...")
    time.sleep(3)

    try:
        # テストモード選択
        print("\nテストモード選択:")
        print("  1. 自動4方向テスト（推奨）")
        print("  2. 手動で座標を指定")

        choice = input("\n選択 (1 or 2): ").strip()

        if choice == "1":
            tester.run_test_sequence()
        elif choice == "2":
            while True:
                print("\n目標座標を入力（Map座標系）:")
                try:
                    x = float(input("  X座標: "))
                    y = float(input("  Y座標: "))
                    desc = input("  説明（任意): ")

                    tester.send_test_goal(x, y, desc)

                    cont = input("\n別の座標をテストしますか？ (y/n): ")
                    if cont.lower() != 'y':
                        break

                except ValueError:
                    print("❌ 数値を入力してください")
                except KeyboardInterrupt:
                    break

        print("\n👋 テスト終了")

    except KeyboardInterrupt:
        print("\n\n⚠️ テスト中断")

    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()