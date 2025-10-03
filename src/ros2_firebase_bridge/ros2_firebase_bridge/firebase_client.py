"""Firebase Firestore接続クライアント - 完全版"""

import firebase_admin
from firebase_admin import credentials, firestore
import threading
import time
import logging
from typing import Callable, Dict, Any, Optional

class FirebaseClient:
    def __init__(self, service_account_path: str, logger=None):
        """
        Firebase Firestoreクライアントを初期化

        Args:
            service_account_path: サービスアカウントキーのパス
            logger: ロガーインスタンス
        """
        self.logger = logger or logging.getLogger(__name__)
        self.db = None
        self.listeners = {}
        self._initialize_firebase(service_account_path)

    def _initialize_firebase(self, service_account_path: str):
        """Firebase Admin SDKを初期化"""
        try:
            # 既存のアプリがある場合は使用
            try:
                app = firebase_admin.get_app()
                self.logger.info("既存のFirebaseアプリを使用します")
            except ValueError:
                # アプリが存在しない場合は新規作成
                cred = credentials.Certificate(service_account_path)
                app = firebase_admin.initialize_app(cred)
                self.logger.info("Firebase Admin SDK初期化完了")

            self.db = firestore.client()
            self.logger.info("Firestore接続確立")

        except Exception as e:
            self.logger.error(f"Firebase初期化失敗: {e}")
            raise

    def setup_realtime_listener(self, collection_name: str, callback: Callable):
        """
        Firestoreコレクションのリアルタイム更新を監視

        Args:
            collection_name: 監視するコレクション名
            callback: 更新時に呼び出されるコールバック関数
        """
        def on_snapshot(doc_snapshot, changes, read_time):
            try:
                for change in changes:
                    if change.type.name in ['ADDED', 'MODIFIED']:
                        doc_id = change.document.id
                        doc_data = change.document.to_dict()

                        self.logger.debug(f"Firestore更新検知: {doc_id}, {change.type.name}")
                        callback(doc_id, doc_data, change.type.name)

            except Exception as e:
                self.logger.error(f"Firestoreスナップショット処理エラー: {e}")

        try:
            collection_ref = self.db.collection(collection_name)
            listener = collection_ref.on_snapshot(on_snapshot)
            self.listeners[collection_name] = listener

            self.logger.info(f"Firestoreリアルタイムリスナー開始: {collection_name}")
            return listener

        except Exception as e:
            self.logger.error(f"リアルタイムリスナー設定失敗: {e}")
            raise

    def update_robot_state(self, robot_id: str, update_data: Dict[str, Any]):
        """
        ロボットの状態を更新（汎用メソッド）

        Args:
            robot_id: ロボットのドキュメントID
            update_data: 更新するデータ（辞書形式）
                例: {'position': GeoPoint(...), 'status': 'idle'}
        """
        try:
            doc_ref = self.db.collection('robots').document(robot_id)
            doc_ref.update(update_data)
            self.logger.debug(f"ロボット状態更新完了: {robot_id}")
        except Exception as e:
            self.logger.error(f"ロボット状態更新失敗 {robot_id}: {e}")

    def update_robot_status(self, robot_id: str, position: Dict[str, float],
                          status: str, additional_data: Optional[Dict] = None):
        """
        ロボットの状態をFirestoreに更新（後方互換性のため残す）

        Args:
            robot_id: ロボットのドキュメントID
            position: 位置情報 {'lat': float, 'lng': float}
            status: ロボットの状態
            additional_data: 追加データ
        """
        try:
            doc_ref = self.db.collection('robots').document(robot_id)

            update_data = {
                'position': firestore.GeoPoint(position['lat'], position['lng']),
                'status': status,
                'last_updated': firestore.SERVER_TIMESTAMP
            }

            if additional_data:
                update_data.update(additional_data)

            doc_ref.update(update_data)
            self.logger.debug(f"ロボット状態更新完了: {robot_id} -> {status}")

        except Exception as e:
            self.logger.error(f"ロボット状態更新失敗 {robot_id}: {e}")

    def update_robot_telemetry(self, robot_id: str, telemetry_data: Dict[str, Any]):
        """
        ロボットのテレメトリデータを更新

        Args:
            robot_id: ロボットのドキュメントID
            telemetry_data: テレメトリデータ
                例: {'battery_percent': 85.5, 'speed': 0.22, ...}
        """
        try:
            doc_ref = self.db.collection('robots').document(robot_id)

            # SERVER_TIMESTAMP を追加
            telemetry_data['last_sensor_update'] = firestore.SERVER_TIMESTAMP

            doc_ref.update({'telemetry': telemetry_data})
            self.logger.debug(f"テレメトリ更新完了: {robot_id}")
        except Exception as e:
            self.logger.error(f"テレメトリ更新失敗 {robot_id}: {e}")

    def get_robot_data(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """
        指定されたロボットのデータを取得

        Args:
            robot_id: ロボットのドキュメントID

        Returns:
            ロボットデータ（存在しない場合はNone）
        """
        try:
            doc_ref = self.db.collection('robots').document(robot_id)
            doc = doc_ref.get()

            if doc.exists:
                return doc.to_dict()
            else:
                self.logger.warning(f"ロボットデータが存在しません: {robot_id}")
                return None

        except Exception as e:
            self.logger.error(f"ロボットデータ取得失敗 {robot_id}: {e}")
            return None

    def close_listeners(self):
        """すべてのリアルタイムリスナーを停止"""
        for collection_name, listener in self.listeners.items():
            try:
                listener.unsubscribe()
                self.logger.info(f"リスナー停止完了: {collection_name}")
            except Exception as e:
                self.logger.error(f"リスナー停止失敗 {collection_name}: {e}")

        self.listeners.clear()

    def __del__(self):
        """デストラクタ"""
        self.close_listeners()