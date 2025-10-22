#!/usr/bin/env python3
"""
Phase 2 総合検証テストスクリプト

テスト項目:
1. 無限ループ完全防止テスト(100回配車)
2. 位置同期パフォーマンステスト
3. 複数ロボット同時動作テスト
"""

import firebase_admin
from firebase_admin import credentials, firestore
import time
import sys
import os
from datetime import datetime
import math


class Phase2ValidationTests:
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

        self.test_results = {
            'infinite_loop_test': {'passed': False, 'details': {}},
            'position_sync_test': {'passed': False, 'details': {}},
            'multi_robot_test': {'passed': False, 'details': {}},
        }

    # ========================================
    # Test 1: 無限ループ完全防止テスト
    # ========================================
    def test_infinite_loop_prevention(self):
        """
        100回の配車リクエストで無限ループが発生しないか検証
        """
        print("\n" + "="*60)
        print("🧪 Test 1: 無限ループ完全防止テスト")
        print("="*60)

        robot_id = 'robot_001'
        test_coords = [
            (36.55077, 139.92957),
            (36.55080, 139.92960),
            (36.55075, 139.92950),
        ]

        # リスナーでFirestore書き込み回数をカウント
        write_counts = {'total': 0, 'destination_writes': 0}

        def on_snapshot(doc_snapshot, changes, read_time):
            for change in changes:
                if change.document.id == robot_id:
                    write_counts['total'] += 1
                    data = change.document.to_dict()
                    if 'destination' in data and data['destination']:
                        write_counts['destination_writes'] += 1

        # リスナー開始
        collection_ref = self.db.collection('robots')
        listener = collection_ref.on_snapshot(on_snapshot)

        try:
            print(f"\n📍 100回の配車リクエストを送信します...")

            # ロボット状態をリセット
            robot_ref = self.db.collection('robots').document(robot_id)
            robot_ref.update({
                'destination': firestore.DELETE_FIELD,
                'status': 'idle',
                'last_updated': firestore.SERVER_TIMESTAMP
            })
            time.sleep(2)

            # 100回配車リクエスト
            for i in range(100):
                coord = test_coords[i % len(test_coords)]

                robot_ref.update({
                    'destination': firestore.GeoPoint(coord[0], coord[1]),
                    'status': 'dispatching',
                    'last_updated': firestore.SERVER_TIMESTAMP
                })

                # destination到着シミュレーション(即座に削除)
                time.sleep(0.05)
                robot_ref.update({
                    'destination': firestore.DELETE_FIELD,
                    'status': 'idle',
                    'last_updated': firestore.SERVER_TIMESTAMP
                })

                if (i + 1) % 20 == 0:
                    print(f"  進捗: {i+1}/100")

                time.sleep(0.1)

            # 結果を待つ
            print("\n⏳ 5秒待機してFirestore書き込み回数を確認...")
            time.sleep(5)

            # 判定
            print(f"\n📊 結果:")
            print(f"  送信回数: 200回 (destination設定100回 + 削除100回)")
            print(f"  Firestore総書き込み: {write_counts['total']}回")
            print(f"  destination付き書き込み: {write_counts['destination_writes']}回")

            # 無限ループ判定: destination書き込みが150回を超えたら異常
            if write_counts['destination_writes'] <= 150:
                print("\n✅ 無限ループは発生しませんでした")
                self.test_results['infinite_loop_test']['passed'] = True
                self.test_results['infinite_loop_test']['details'] = {
                    'sent': 100,
                    'writes': write_counts['destination_writes'],
                    'loop_detected': False
                }
                return True
            else:
                print(f"\n❌ 無限ループ検知: {write_counts['destination_writes']}回のdestination書き込み")
                self.test_results['infinite_loop_test']['details'] = {
                    'sent': 100,
                    'writes': write_counts['destination_writes'],
                    'loop_detected': True
                }
                return False

        finally:
            listener.unsubscribe()

    # ========================================
    # Test 2: 位置同期パフォーマンステスト
    # ========================================
    def test_position_sync_performance(self):
        """
        位置更新頻度とレイテンシを測定
        """
        print("\n" + "="*60)
        print("🧪 Test 2: 位置同期パフォーマンステスト")
        print("="*60)

        robot_id = 'robot_001'
        position_updates = []

        def on_snapshot(doc_snapshot, changes, read_time):
            for change in changes:
                if change.document.id == robot_id and change.type.name == 'MODIFIED':
                    data = change.document.to_dict()
                    if 'position' in data:
                        position_updates.append({
                            'timestamp': time.time(),
                            'position': data['position']
                        })

        collection_ref = self.db.collection('robots')
        listener = collection_ref.on_snapshot(on_snapshot)

        try:
            print("\n📍 30秒間、位置更新をシミュレーションします...")

            robot_ref = self.db.collection('robots').document(robot_id)
            start_time = time.time()
            update_count = 0

            # 30秒間、0.5秒ごとに位置を更新
            while time.time() - start_time < 30:
                # 1m移動をシミュレーション
                lat = 36.55077 + (update_count * 0.00001)
                lng = 139.92957 + (update_count * 0.00001)

                robot_ref.update({
                    'position': firestore.GeoPoint(lat, lng),
                    'last_updated': firestore.SERVER_TIMESTAMP
                })

                update_count += 1
                time.sleep(0.5)

            # 結果待機
            time.sleep(3)

            # 判定
            print(f"\n📊 結果:")
            print(f"  送信した位置更新: {update_count}回")
            print(f"  実際のFirestore更新: {len(position_updates)}回")

            # 更新間隔計算
            if len(position_updates) > 1:
                intervals = []
                for i in range(1, len(position_updates)):
                    interval = position_updates[i]['timestamp'] - position_updates[i-1]['timestamp']
                    intervals.append(interval)

                avg_interval = sum(intervals) / len(intervals)
                print(f"  平均更新間隔: {avg_interval:.2f}秒")

                # 判定: 平均更新間隔が0.8秒以上なら合格(過剰更新を防いでいる)
                if avg_interval >= 0.8:
                    print("\n✅ 位置同期が最適化されています")
                    self.test_results['position_sync_test']['passed'] = True
                    self.test_results['position_sync_test']['details'] = {
                        'sent': update_count,
                        'received': len(position_updates),
                        'avg_interval': avg_interval
                    }
                    return True
                else:
                    print(f"\n⚠️ 更新頻度が高すぎます(平均{avg_interval:.2f}秒)")
                    self.test_results['position_sync_test']['details'] = {
                        'sent': update_count,
                        'received': len(position_updates),
                        'avg_interval': avg_interval
                    }
                    return False
            else:
                print("\n❌ 位置更新が検出されませんでした")
                return False

        finally:
            listener.unsubscribe()

    # ========================================
    # Test 3: 複数ロボット同時動作テスト
    # ========================================
    def test_multi_robot_operation(self):
        """
        3台のロボットが同時に動作できるか検証
        """
        print("\n" + "="*60)
        print("🧪 Test 3: 複数ロボット同時動作テスト")
        print("="*60)

        robot_ids = ['robot_001', 'robot_002', 'robot_003']

        # 各ロボットの初期化
        print("\n🤖 3台のロボットを初期化中...")
        for robot_id in robot_ids:
            robot_ref = self.db.collection('robots').document(robot_id)
            robot_ref.set({
                'id': robot_id,
                'name': f'TurtleBot3-{robot_id[-3:]}',
                'status': 'idle',
                'position': firestore.GeoPoint(36.55077, 139.92957),
                'heading': 0.0,
                'telemetry': {
                    'battery_percent': 100.0,
                    'speed': 0.0
                }
            })

        print("✅ 初期化完了")
        time.sleep(2)

        # 同時配車テスト
        print("\n📍 3台のロボットに同時に配車指示を送信...")

        test_destinations = [
            (36.55080, 139.92960),  # robot_001
            (36.55075, 139.92970),  # robot_002
            (36.55085, 139.92950),  # robot_003
        ]

        for i, robot_id in enumerate(robot_ids):
            robot_ref = self.db.collection('robots').document(robot_id)
            dest = test_destinations[i]
            robot_ref.update({
                'destination': firestore.GeoPoint(dest[0], dest[1]),
                'status': 'dispatching',
                'last_updated': firestore.SERVER_TIMESTAMP
            })

        print("✅ 配車指示送信完了")

        # 5秒待機してステータス確認
        time.sleep(5)

        # 各ロボットのステータス確認
        print("\n📊 各ロボットのステータス:")
        all_working = True

        for robot_id in robot_ids:
            robot_ref = self.db.collection('robots').document(robot_id)
            robot_doc = robot_ref.get()

            if robot_doc.exists:
                robot_data = robot_doc.to_dict()
                status = robot_data.get('status', 'unknown')
                has_dest = 'destination' in robot_data and robot_data['destination']

                print(f"  {robot_id}: status={status}, destination={'あり' if has_dest else 'なし'}")

                if status == 'idle' and not has_dest:
                    all_working = False
            else:
                print(f"  {robot_id}: ❌ データが見つかりません")
                all_working = False

        # destination削除(クリーンアップ)
        for robot_id in robot_ids:
            robot_ref = self.db.collection('robots').document(robot_id)
            robot_ref.update({
                'destination': firestore.DELETE_FIELD,
                'status': 'idle'
            })

        if all_working:
            print("\n✅ 3台のロボットが正常に動作しています")
            self.test_results['multi_robot_test']['passed'] = True
            self.test_results['multi_robot_test']['details'] = {
                'robots_tested': len(robot_ids),
                'all_operational': True
            }
            return True
        else:
            print("\n⚠️ 一部のロボットが正常に動作していません")
            self.test_results['multi_robot_test']['details'] = {
                'robots_tested': len(robot_ids),
                'all_operational': False
            }
            return False

    # ========================================
    # メイン実行
    # ========================================
    def run_all_tests(self):
        """すべてのテストを実行"""
        print("\n" + "="*60)
        print("🚀 Phase 2 総合検証テスト開始")
        print("="*60)

        test1_passed = self.test_infinite_loop_prevention()
        time.sleep(3)

        test2_passed = self.test_position_sync_performance()
        time.sleep(3)

        test3_passed = self.test_multi_robot_operation()

        # 総合判定
        print("\n" + "="*60)
        print("📊 Phase 2 テスト結果サマリー")
        print("="*60)
        print(f"Test 1 (無限ループ防止): {'✅ PASS' if test1_passed else '❌ FAIL'}")
        print(f"Test 2 (位置同期最適化): {'✅ PASS' if test2_passed else '❌ FAIL'}")
        print(f"Test 3 (複数ロボット対応): {'✅ PASS' if test3_passed else '❌ FAIL'}")

        all_passed = test1_passed and test2_passed and test3_passed

        if all_passed:
            print("\n🎉 すべてのPhase 2テストに合格しました!")
            print("   → Phase 2の3大課題は完全に解決されています")
        else:
            print("\n⚠️ 一部のテストで問題が見つかりました")
            print("   → 修正が必要な項目を確認してください")

        print("="*60)

        return all_passed


def main():
    print("""
╔══════════════════════════════════════════════════════════╗
║         Phase 2 総合検証テストスクリプト                 ║
║  無限ループ・位置同期・複数ロボット動作を検証           ║
╚══════════════════════════════════════════════════════════╝
""")

    tester = Phase2ValidationTests()
    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()