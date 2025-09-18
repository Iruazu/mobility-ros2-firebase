#!/usr/bin/env python3
"""Firebase接続テスト"""

import os
import sys
import firebase_admin
from firebase_admin import credentials, firestore
from datetime import datetime

def test_firebase_connection():
    print("🔥 Firebase接続テストを開始します...")

    try:
        # Firebase認証ファイルの確認
        service_account_path = '/workspace/config/serviceAccountKey.json'

        if not os.path.exists(service_account_path):
            print(f"❌ Firebase認証ファイルが見つかりません: {service_account_path}")
            print("💡 config/serviceAccountKey.json を配置してください")
            return False

        print(f"✅ Firebase認証ファイル確認: {service_account_path}")

        # Firebase初期化
        cred = credentials.Certificate(service_account_path)

        # 既存のアプリがある場合は削除
        try:
            firebase_admin.delete_app(firebase_admin.get_app())
        except ValueError:
            pass  # アプリが存在しない場合

        app = firebase_admin.initialize_app(cred)
        print("✅ Firebase Admin SDK 初期化成功")

        # Firestore接続テスト
        db = firestore.client()
        print("✅ Firestore接続成功")

        # テストデータ書き込み
        test_ref = db.collection('connection_test').document('ros2_test')
        test_data = {
            'message': 'ROS2からの接続テスト成功',
            'timestamp': firestore.SERVER_TIMESTAMP,
            'test_time': datetime.now().isoformat(),
            'status': 'connected'
        }

        test_ref.set(test_data)
        print("✅ Firestoreにテストデータを書き込み成功")

        # テストデータ読み込み
        doc = test_ref.get()
        if doc.exists:
            data = doc.to_dict()
            print(f"✅ データ読み込み成功:")
            print(f"   - メッセージ: {data.get('message', 'N/A')}")
            print(f"   - テスト時刻: {data.get('test_time', 'N/A')}")

        # robotsコレクションの確認
        robots_ref = db.collection('robots')
        robots = list(robots_ref.stream())
        print(f"✅ robotsコレクション確認: {len(robots)}個のロボットが登録されています")

        for robot in robots[:3]:  # 最初の3個だけ表示
            robot_data = robot.to_dict()
            print(f"   - ロボットID: {robot.id}")
            print(f"     状態: {robot_data.get('status', 'N/A')}")
            if 'position' in robot_data:
                pos = robot_data['position']
                print(f"     位置: ({pos.latitude:.4f}, {pos.longitude:.4f})")

        print("\n🎉 Firebase接続テスト完了！すべて正常に動作しています。")
        return True

    except Exception as e:
        print(f"❌ Firebase接続失敗:")
        print(f"   エラー詳細: {str(e)}")
        print(f"   エラータイプ: {type(e).__name__}")
        print("\n🔧 対処方法:")
        print("1. serviceAccountKey.json ファイルが正しく配置されているか確認")
        print("2. Firebaseプロジェクトの設定が正しいか確認")
        print("3. インターネット接続を確認")
        return False

if __name__ == "__main__":
    success = test_firebase_connection()
    sys.exit(0 if success else 1)