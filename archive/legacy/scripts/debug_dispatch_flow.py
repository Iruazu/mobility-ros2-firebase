#!/usr/bin/env python3
"""
配車フロー デバッグツール
Web配車 → Gazebo移動 → ゴール到達 → Webフィードバックの流れを検証
"""

import firebase_admin
from firebase_admin import credentials, firestore
import time
import os
from datetime import datetime

class DispatchFlowDebugger:
    def __init__(self, service_account_path='/workspace/config/serviceAccountKey.json'):
        if not os.path.exists(service_account_path):
            print(f"❌ Service account not found: {service_account_path}")
            exit(1)

        # Firebase初期化
        cred = credentials.Certificate(service_account_path)
        try:
            firebase_admin.get_app()
            print("✅ Using existing Firebase app")
        except ValueError:
            firebase_admin.initialize_app(cred)
            print("✅ Firebase app initialized")

        self.db = firestore.client()
        self.robot_id = 'robot_001'

    def check_robot_state(self):
        """現在のロボット状態を確認"""
        print("\n" + "="*60)
        print("🔍 ロボット状態確認")
        print("="*60)

        try:
            robot_ref = self.db.collection('robots').document(self.robot_id)
            robot_doc = robot_ref.get()

            if not robot_doc.exists:
                print(f"❌ ロボット {self.robot_id} が見つかりません")
                return None

            robot_data = robot_doc.to_dict()

            print(f"ロボットID: {robot_data.get('id', 'N/A')}")
            print(f"名前: {robot_data.get('name', 'N/A')}")
            print(f"ステータス: {robot_data.get('status', 'N/A')}")

            position = robot_data.get('position')
            if position:
                print(f"現在位置: ({position.latitude:.6f}, {position.longitude:.6f})")
            else:
                print("現在位置: データなし")

            destination = robot_data.get('destination')
            if destination:
                print(f"目的地: ({destination.latitude:.6f}, {destination.longitude:.6f})")
            else:
                print("目的地: 設定なし")

            heading = robot_data.get('heading')
            if heading is not None:
                print(f"方位角: {heading:.2f} rad")

            telemetry = robot_data.get('telemetry', {})
            if telemetry:
                print("\nテレメトリデータ:")
                print(f"  速度: {telemetry.get('speed', 0):.2f} m/s")
                print(f"  バッテリー: {telemetry.get('battery_percent', 0):.1f}%")

                distance_to_goal = telemetry.get('distance_to_goal')
                if distance_to_goal is not None:
                    print(f"  目的地までの距離: {distance_to_goal:.2f} m")
                else:
                    print(f"  目的地までの距離: 測定なし")

                obstacle = telemetry.get('obstacle_detected', False)
                print(f"  障害物検知: {'⚠️ あり' if obstacle else '✅ なし'}")

            last_updated = robot_data.get('last_updated')
            if last_updated:
                print(f"\n最終更新: {last_updated}")

            return robot_data

        except Exception as e:
            print(f"❌ エラー: {e}")
            return None

    def monitor_dispatch_flow(self, duration=60):
        """配車フローをリアルタイムで監視"""
        print("\n" + "="*60)
        print(f"📡 配車フロー監視開始 (継続時間: {duration}秒)")
        print("="*60)
        print("\n監視項目:")
        print("  1. destination設定の検知")
        print("  2. ステータス変化の追跡")
        print("  3. 位置の更新頻度")
        print("  4. ゴール到達の検知")
        print("  5. destination削除の確認")
        print("\n" + "-"*60)

        robot_ref = self.db.collection('robots').document(self.robot_id)
        last_status = None
        last_position = None
        last_destination = None
        position_update_count = 0
        start_time = time.time()

        def on_snapshot(doc_snapshot, changes, read_time):
            nonlocal last_status, last_position, last_destination, position_update_count

            for change in changes:
                if change.document.id == self.robot_id:
                    robot_data = change.document.to_dict()
                    timestamp = datetime.now().strftime('%H:%M:%S')

                    # ステータス変化検知
                    current_status = robot_data.get('status')
                    if current_status != last_status:
                        print(f"\n[{timestamp}] 🔄 ステータス変化: {last_status} → {current_status}")
                        last_status = current_status

                    # destination設定検知
                    current_destination = robot_data.get('destination')
                    if current_destination and not last_destination:
                        print(f"[{timestamp}] 🎯 destination設定検知: "
                              f"({current_destination.latitude:.6f}, {current_destination.longitude:.6f})")
                    elif not current_destination and last_destination:
                        print(f"[{timestamp}] ✅ destination削除検知（ゴール到達）")
                    last_destination = current_destination

                    # 位置更新検知
                    current_position = robot_data.get('position')
                    if current_position:
                        if last_position:
                            # 移動距離計算（簡易版）
                            dlat = abs(current_position.latitude - last_position.latitude)
                            dlng = abs(current_position.longitude - last_position.longitude)

                            if dlat > 0.00001 or dlng > 0.00001:  # 約1m以上の移動
                                position_update_count += 1
                                print(f"[{timestamp}] 📍 位置更新 #{position_update_count}: "
                                      f"({current_position.latitude:.6f}, {current_position.longitude:.6f})")

                        last_position = current_position

                    # テレメトリ確認
                    telemetry = robot_data.get('telemetry', {})
                    distance_to_goal = telemetry.get('distance_to_goal')
                    if distance_to_goal is not None and distance_to_goal < 0.5:
                        print(f"[{timestamp}] 🏁 ゴール間近: 残り {distance_to_goal:.2f}m")

        # Firestoreリスナー開始
        listener = self.db.collection('robots').on_snapshot(on_snapshot)

        try:
            print(f"\n⏰ {duration}秒間監視します...")
            print("   (Ctrl+Cで中断)\n")

            while time.time() - start_time < duration:
                time.sleep(1)

            print("\n" + "="*60)
            print("✅ 監視完了")
            print("="*60)
            print(f"検出した位置更新回数: {position_update_count}")

        except KeyboardInterrupt:
            print("\n\n⚠️ 監視を中断しました")
        finally:
            listener.unsubscribe()

    def test_dispatch_scenario(self):
        """配車シナリオのテスト"""
        print("\n" + "="*60)
        print("🧪 配車シナリオテスト")
        print("="*60)

        # Step 1: 初期状態確認
        print("\n--- Step 1: 初期状態確認 ---")
        initial_state = self.check_robot_state()

        if not initial_state:
            print("❌ 初期状態の取得に失敗")
            return

        # Step 2: destination設定のシミュレーション
        print("\n--- Step 2: テスト用destination設定 ---")
        print("⚠️ この操作を実行すると、ロボットが実際に動き出します")
        response = input("続行しますか？ (y/n): ")

        if response.lower() != 'y':
            print("テストを中止しました")
            return

        # テスト用の目的地（TurtleBot3 Worldマップ内）
        test_lat = 36.55080
        test_lng = 139.92960

        robot_ref = self.db.collection('robots').document(self.robot_id)
        robot_ref.update({
            'destination': firestore.GeoPoint(test_lat, test_lng),
            'status': 'dispatching',
            'last_updated': firestore.SERVER_TIMESTAMP
        })

        print(f"✅ destination設定完了: ({test_lat:.6f}, {test_lng:.6f})")

        # Step 3: 60秒間監視
        print("\n--- Step 3: 配車フロー監視 ---")
        self.monitor_dispatch_flow(duration=60)

        # Step 4: 最終状態確認
        print("\n--- Step 4: 最終状態確認 ---")
        final_state = self.check_robot_state()

        # Step 5: 結果分析
        print("\n" + "="*60)
        print("📊 テスト結果分析")
        print("="*60)

        if not final_state:
            print("❌ 最終状態の取得に失敗")
            return

        initial_status = initial_state.get('status')
        final_status = final_state.get('status')

        print(f"初期ステータス: {initial_status}")
        print(f"最終ステータス: {final_status}")

        final_destination = final_state.get('destination')
        if final_destination:
            print("⚠️ destinationが残っています（ゴール未到達）")
        else:
            print("✅ destinationが削除されています（ゴール到達）")

        # テレメトリ確認
        telemetry = final_state.get('telemetry', {})
        distance_to_goal = telemetry.get('distance_to_goal')
        if distance_to_goal is not None:
            print(f"最終的な目的地までの距離: {distance_to_goal:.2f}m")

    def cleanup_test_data(self):
        """テストデータのクリーンアップ"""
        print("\n" + "="*60)
        print("🧹 テストデータクリーンアップ")
        print("="*60)

        try:
            robot_ref = self.db.collection('robots').document(self.robot_id)
            robot_ref.update({
                'destination': firestore.DELETE_FIELD,
                'status': 'idle',
                'last_updated': firestore.SERVER_TIMESTAMP
            })
            print("✅ クリーンアップ完了")
        except Exception as e:
            print(f"❌ クリーンアップエラー: {e}")


def main():
    print("""
╔══════════════════════════════════════════════════════════╗
║            配車フロー デバッグツール                      ║
║  Web配車 → Gazebo移動 → ゴール到達 → Webフィードバック  ║
╚══════════════════════════════════════════════════════════╝
    """)

    debugger = DispatchFlowDebugger()

    while True:
        print("\n" + "="*60)
        print("メニュー:")
        print("  1. ロボット状態確認")
        print("  2. 配車フロー監視（60秒）")
        print("  3. 配車シナリオテスト（完全版）")
        print("  4. テストデータクリーンアップ")
        print("  5. 終了")
        print("="*60)

        choice = input("\n選択してください (1-5): ").strip()

        if choice == '1':
            debugger.check_robot_state()

        elif choice == '2':
            debugger.monitor_dispatch_flow(duration=60)

        elif choice == '3':
            debugger.test_dispatch_scenario()

        elif choice == '4':
            debugger.cleanup_test_data()

        elif choice == '5':
            print("\n👋 終了します")
            break

        else:
            print("❌ 無効な選択です")

        input("\nEnterキーで続行...")


if __name__ == "__main__":
    main()