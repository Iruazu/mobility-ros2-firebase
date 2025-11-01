#!/usr/bin/env python3
"""Firebase接続テストスクリプト"""

import os
import sys
from datetime import datetime
import firebase_admin
from firebase_admin import credentials, firestore

def test_firebase_connection():
    """
    Firebase Admin SDKを初期化し、Firestoreへのデータの読み書きをテストします。
    """
    print("🔥 Firebase接続テストを開始します...")

    try:
        # --- 認証ファイルの設定 ---
        # ユーザーのホームディレクトリを取得
        home = os.path.expanduser('~')
        # serviceAccountKey.json への想定されるパス
        service_account_path = os.path.join(home, 'mobility-ros2-firebase/config/serviceAccountKey.json')

        # Firebase認証ファイルの確認
        if not os.path.exists(service_account_path):
            print(f"❌ Firebase認証ファイルが見つかりません: {service_account_path}")
            print("💡 config/serviceAccountKey.json をホームディレクトリからの相対パスで配置してください")
            return False

        print(f"✅ Firebase認証ファイル確認: {service_account_path}")

        # --- Firebase初期化 ---
        cred = credentials.Certificate(service_account_path)

        # 既存のアプリがある場合は削除してから初期化（テスト環境で重複初期化を防ぐため）
        try:
            # 既存のデフォルトアプリを取得しようと試みる
            firebase_admin.get_app()
            # 存在すれば削除
            firebase_admin.delete_app(firebase_admin.get_app())
        except ValueError:
            pass  # アプリが存在しない場合は何もしない

        app = firebase_admin.initialize_app(cred)
        print("✅ Firebase Admin SDK 初期化成功")

        # --- Firestore接続テスト ---
        db = firestore.client()
        print("✅ Firestore接続成功")

        # --- テストデータ書き込み ---
        test_ref = db.collection('connection_test').document('ros2_test')
        test_data = {
            'message': 'ROS2からの接続テスト成功',
            'timestamp': firestore.SERVER_TIMESTAMP, # サーバー時刻を記録
            'test_time': datetime.now().isoformat(), # ローカル時刻を記録
            'status': 'connected'
        }

        test_ref.set(test_data)
        print("✅ Firestoreにテストデータを書き込み成功 ('connection_test/ros2_test')")

        # --- テストデータ読み込み ---
        doc = test_ref.get()
        if doc.exists:
            data = doc.to_dict()
            print(f"✅ データ読み込み成功:")
            print(f"   - メッセージ: {data.get('message', 'N/A')}")
            print(f"   - ローカルテスト時刻: {data.get('test_time', 'N/A')}")
            # firestore.SERVER_TIMESTAMP は読み込み時にdatetimeオブジェクトとして取得される
            server_time = data.get('timestamp')
            if server_time:
                 # datetimeオブジェクトを整形して表示
                 print(f"   - サーバータイムスタンプ: {server_time.isoformat()}")

        # --- robotsコレクションの確認（オプション）---
        robots_ref = db.collection('robots')
        # コレクションからすべてのドキュメントを取得
        robots = list(robots_ref.stream())
        print(f"✅ robotsコレクション確認: {len(robots)}個のロボットが登録されています")

        for robot in robots[:3]:  # 最初の3個だけ表示
            robot_data = robot.to_dict()
            print(f"   - ロボットID: {robot.id}")
            print(f"     状態: {robot_data.get('status', 'N/A')}")
            # 'position'フィールドがGeoPoint型の場合の処理を想定
            if 'position' in robot_data and hasattr(robot_data['position'], 'latitude'):
                pos = robot_data['position']
                print(f"     位置 (GeoPoint): ({pos.latitude:.4f}, {pos.longitude:.4f})")
            elif 'position' in robot_data and isinstance(robot_data['position'], dict):
                 # positionがマップとして保存されている場合（例: {'lat': X, 'lon': Y}）
                pos = robot_data['position']
                print(f"     位置 (Map): ({pos.get('lat', 'N/A'):.4f}, {pos.get('lon', 'N/A'):.4f})")
            else:
                pass # 位置情報がない、または不明な形式

        print("\n🎉 Firebase接続テスト完了！すべて正常に動作しています。")
        return True

    except Exception as e:
        print(f"❌ Firebase接続失敗:")
        print(f"   エラー詳細: {str(e)}")
        print(f"   エラータイプ: {type(e).__name__}")
        print("\n🔧 対処方法:")
        print("1. serviceAccountKey.json ファイルが正しいパスに配置されているか確認")
        print("2. FirebaseプロジェクトID、権限などが正しいか確認")
        print("3. Python環境で 'firebase-admin' がインストールされているか確認 (pip install firebase-admin)")
        print("4. インターネット接続を確認")
        return False

if __name__ == "__main__":
    success = test_firebase_connection()
    # 成功なら終了コード0、失敗なら1で終了
    sys.exit(0 if success else 1)
