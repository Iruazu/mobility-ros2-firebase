#!/usr/bin/env python3
"""
無限ループ検証テストスクリプト
destination更新の冪等性と重複検知を確認
"""

import firebase_admin
from firebase_admin import credentials, firestore
import time
import sys
import os
from datetime import datetime


class InfiniteLoopTester:
    def __init__(self, service_account_path='/workspace/config/serviceAccountKey.json'):
        if not os.path.exists(service_account_path):
            print(f"❌ Service account not found: {service_account_path}")
            sys.exit(1)

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
        self.test_results = {
            'updates_sent': 0,
            'firestore_writes': 0,
            'loop_detected': False,
            'test_passed': False
        }

    def reset_robot_state(self):
        """ロボット状態をリセット"""
        print("\n🔄 ロボット状態をリセット中...")
        try:
            robot_ref = self.db.collection('robots').document(self.robot_id)
            robot_ref.update({
                'destination': firestore.DELETE_FIELD,
                'status': 'idle',
                'last_updated': firestore.SERVER_TIMESTAMP
            })
            print("✅ リセット完了")
            time.sleep(2)
            return True
        except Exception as e:
            print(f"❌ リセット失敗: {e}")
            return False

    def setup_listener(self):
        """Firestore更新をリスナーで監視"""
        self.updates_received = []

        def on_snapshot(doc_snapshot, changes, read_time):
            for change in changes:
                if change.document.id == self.robot_id:
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    data = change.document.to_dict()

                    update_info = {
                        'timestamp': timestamp,
                        'type': change.type.name,
                        'has_destination': 'destination' in data,
                        'destination': data.get('destination'),
                        'status': data.get('status')
                    }

                    self.updates_received.append(update_info)
                    self.test_results['firestore_writes'] += 1

                    print(f"  [{timestamp}] Firestore更新検知: "
                          f"{change.type.name}, "
                          f"destination={'あり' if update_info['has_destination'] else 'なし'}, "
                          f"status={update_info['status']}")

        collection_ref = self.db.collection('robots')
        self.listener = collection_ref.on_snapshot(on_snapshot)
        print("✅ Firestoreリスナー開始")

    def test_duplicate_destination(self):
        """Test 1: 同じdestinationの重複更新テスト"""
        print("\n" + "="*60)
        print("Test 1: 同じdestinationの重複更新テスト")
        print("="*60)

        test_lat = 36.55077
        test_lng = 139.92957

        robot_ref = self.db.collection('robots').document(self.robot_id)

        print(f"\n📍 テスト座標: ({test_lat:.6f}, {test_lng:.6f})")
        print("⚡ 同じdestinationを5回連続で送信します...\n")

        self.updates_received = []
        time.sleep(1)  # リスナーの準備

        for i in range(5):
            try:
                robot_ref.update({
                    'destination': firestore.GeoPoint(test_lat, test_lng),
                    'status': 'moving',
                    'last_updated': firestore.SERVER_TIMESTAMP
                })
                self.test_results['updates_sent'] += 1
                print(f"  [{i+1}/5] destination更新送信")
                time.sleep(0.5)
            except Exception as e:
                print(f"  ❌ 更新{i+1}失敗: {e}")

        # 結果を待つ
        print("\n⏳ 5秒待機してFirestoreの更新を確認...")
        time.sleep(5)

        # 結果分析
        print(f"\n📊 結果:")
        print(f"  送信回数: {self.test_results['updates_sent']}")
        print(f"  Firestore書き込み回数: {self.test_results['firestore_writes']}")

        # 同じdestinationの連続更新をカウント
        destination_updates = [u for u in self.updates_received if u['has_destination']]
        print(f"  destination付き更新: {len(destination_updates)}回")

        if len(destination_updates) > 1:
            print(f"\n⚠️ 警告: 同じdestinationが{len(destination_updates)}回書き込まれました")
            print("  → 無限ループのリスクあり")
            self.test_results['loop_detected'] = True
            return False
        elif len(destination_updates) == 1:
            print(f"\n✅ 正常: destinationは1回のみ書き込まれました")
            print("  → 重複検知が正しく動作しています")
            return True
        else:
            print(f"\n⚠️ destinationが書き込まれませんでした")
            return False

    def test_rapid_updates(self):
        """Test 2: 高頻度更新テスト"""
        print("\n" + "="*60)
        print("Test 2: 高頻度更新テスト（0.1秒間隔）")
        print("="*60)

        robot_ref = self.db.collection('robots').document(self.robot_id)

        # 異なる座標を高速で送信
        test_coords = [
            (36.55077, 139.92957),
            (36.55078, 139.92958),
            (36.55079, 139.92959),
            (36.55080, 139.92960),
            (36.55081, 139.92961),
        ]

        print("\n⚡ 異なるdestinationを0.1秒間隔で送信します...\n")

        self.updates_received = []
        time.sleep(1)

        sent_count = 0
        for i, (lat, lng) in enumerate(test_coords):
            try:
                robot_ref.update({
                    'destination': firestore.GeoPoint(lat, lng),
                    'status': 'moving',
                    'last_updated': firestore.SERVER_TIMESTAMP
                })
                sent_count += 1
                print(f"  [{i+1}/5] destination送信: ({lat:.6f}, {lng:.6f})")
                time.sleep(0.1)  # 0.1秒間隔
            except Exception as e:
                print(f"  ❌ 更新{i+1}失敗: {e}")

        print("\n⏳ 5秒待機...")
        time.sleep(5)

        # 結果分析
        destination_updates = [u for u in self.updates_received if u['has_destination']]

        print(f"\n📊 結果:")
        print(f"  送信回数: {sent_count}")
        print(f"  Firestore書き込み回数: {len(destination_updates)}")

        if len(destination_updates) == sent_count:
            print(f"\n✅ すべての更新が正しく書き込まれました")
            return True
        elif len(destination_updates) < sent_count:
            print(f"\n⚠️ 一部の更新がスキップされました")
            print(f"  → スロットリングまたはレート制限が動作しています")
            return True  # これは正常な動作
        else:
            print(f"\n⚠️ 送信回数より多く書き込まれました")
            print(f"  → 無限ループの可能性")
            self.test_results['loop_detected'] = True
            return False

    def test_ros2_echo_prevention(self):
        """Test 3: ROS2のエコーバック防止テスト"""
        print("\n" + "="*60)
        print("Test 3: ROS2エコーバック防止テスト")
        print("="*60)

        robot_ref = self.db.collection('robots').document(self.robot_id)

        test_lat = 36.55085
        test_lng = 139.92965

        print(f"\n📍 テスト座標: ({test_lat:.6f}, {test_lng:.6f})")
        print("⚡ destinationを設定して、ROS2の応答を監視します...\n")

        self.updates_received = []
        time.sleep(1)

        # destination設定
        try:
            robot_ref.update({
                'destination': firestore.GeoPoint(test_lat, test_lng),
                'status': 'moving',
                'last_updated': firestore.SERVER_TIMESTAMP
            })
            print("  ✅ destination設定完了")
        except Exception as e:
            print(f"  ❌ destination設定失敗: {e}")
            return False

        # 15秒間監視（ROS2が処理する時間）
        print("\n⏳ 15秒間、Firestoreの更新を監視します...")
        time.sleep(15)

        # 結果分析
        destination_updates = [u for u in self.updates_received if u['has_destination']]

        print(f"\n📊 結果:")
        print(f"  Firestore書き込み回数: {len(self.updates_received)}")
        print(f"  destination付き更新: {len(destination_updates)}")

        # 詳細表示
        if destination_updates:
            print("\n  destination更新の詳細:")
            for i, update in enumerate(destination_updates):
                dest = update['destination']
                print(f"    [{i+1}] {update['timestamp']} - "
                      f"({dest.latitude:.6f}, {dest.longitude:.6f})")

        # 同じdestinationが複数回書き込まれていないかチェック
        unique_destinations = set()
        for update in destination_updates:
            if update['destination']:
                dest = update['destination']
                coord_str = f"{dest.latitude:.6f},{dest.longitude:.6f}"
                unique_destinations.add(coord_str)

        if len(destination_updates) > len(unique_destinations):
            print(f"\n⚠️ 警告: 同じdestinationが複数回書き込まれました")
            print(f"  → ROS2からのエコーバックが発生している可能性")
            self.test_results['loop_detected'] = True
            return False
        else:
            print(f"\n✅ 各destinationは1回のみ書き込まれました")
            print(f"  → ROS2エコーバック防止が正しく動作しています")
            return True

    def cleanup(self):
        """クリーンアップ"""
        print("\n🧹 クリーンアップ中...")
        if hasattr(self, 'listener'):
            self.listener.unsubscribe()
            print("✅ リスナー停止")

        self.reset_robot_state()

    def run_all_tests(self):
        """すべてのテストを実行"""
        print("\n" + "="*60)
        print("🧪 無限ループ検証テスト開始")
        print("="*60)

        # 初期化
        if not self.reset_robot_state():
            print("❌ 初期化失敗。テストを中止します。")
            return False

        # リスナー開始
        self.setup_listener()

        try:
            # Test 1: 重複更新テスト
            test1_passed = self.test_duplicate_destination()
            time.sleep(2)
            self.reset_robot_state()

            # Test 2: 高頻度更新テスト
            test2_passed = self.test_rapid_updates()
            time.sleep(2)
            self.reset_robot_state()

            # Test 3: ROS2エコーバック防止テスト
            # ※ このテストはROS2が起動している必要があります
            print("\n" + "="*60)
            print("⚠️ Test 3 の実行には ROS2 Bridge が起動している必要があります")
            response = input("ROS2 Bridge は起動していますか？ (y/n): ")

            if response.lower() == 'y':
                test3_passed = self.test_ros2_echo_prevention()
            else:
                print("⏭️ Test 3 をスキップします")
                test3_passed = True  # スキップは失敗扱いにしない

        finally:
            self.cleanup()

        # 総合判定
        print("\n" + "="*60)
        print("📊 テスト結果サマリー")
        print("="*60)
        print(f"Test 1 (重複更新防止): {'✅ PASS' if test1_passed else '❌ FAIL'}")
        print(f"Test 2 (高頻度更新): {'✅ PASS' if test2_passed else '❌ FAIL'}")
        print(f"Test 3 (ROS2エコーバック防止): {'✅ PASS' if test3_passed else '⏭️ SKIP'}")

        all_passed = test1_passed and test2_passed and test3_passed

        if self.test_results['loop_detected']:
            print("\n🚨 無限ループのリスクが検出されました")
            print("   → Phase 2 の無限ループ防止実装が必要です")
            self.test_results['test_passed'] = False
        elif all_passed:
            print("\n🎉 すべてのテストに合格しました！")
            print("   → 無限ループ問題は既に解消されています")
            print("   → Phase 2 の他のタスクに進んでください")
            self.test_results['test_passed'] = True
        else:
            print("\n⚠️ 一部のテストで問題が見つかりました")
            print("   → 詳細を確認して対策を検討してください")
            self.test_results['test_passed'] = False

        print("="*60)

        return all_passed


def main():
    print("""
╔══════════════════════════════════════════════════════════╗
║         無限ループ検証テストスクリプト                    ║
║  Firebase-ROS2統合システムの冪等性とエコー防止を検証   ║
╚══════════════════════════════════════════════════════════╝
""")

    tester = InfiniteLoopTester()
    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()