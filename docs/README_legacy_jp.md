# mobility-ros2-firebase (Legacy Docker環境)

> ⚠️ **注意**: この環境は現在非推奨です。Docker + Nav2による旧実装です。  
> 現行のUbuntu環境については `README_current_jp.md` を参照してください。

## 概要

宇都宮大学 機械システム工学科 Yugo Obanaが開発した、ROS2 Humble + Firebase + Nav2による自律移動ロボットシステムの初期実装です。

**主な特徴:**
- Docker Compose環境で完結
- Nav2による自律ナビゲーション
- Firebase Firestoreとの双方向通信
- TurtleBot3 Gazeboシミュレーション
- GPU加速（WSLg対応）

## アーキテクチャ
```
Docker Container (ROS2 Humble)
├── Gazebo (TurtleBot3 World)
├── Nav2 (自律ナビゲーション)
├── Firebase Bridge (Python)
│   ├── Firestore Listener
│   ├── 座標変換 (GPS ↔ Map)
│   └── Goal Publisher
└── RViz (可視化)
```

## 構成ファイル

### コアディレクトリ
- `archive/docker/` - Docker環境定義
  - `Dockerfile` - ROS2 Humble + Nav2 + 依存関係
  - `devcontainer.json` - VSCode Dev Container設定
- `archive/legacy/launch/` - 起動スクリプト
  - `phase2_minimal.launch.py` - 最小構成
  - `multi_robot.launch.py` - 複数ロボット対応
- `archive/legacy/scripts/` - デバッグツール
  - `coordinate_debug.py` - 座標変換検証
  - `test_infinite_loop.py` - 無限ループ防止テスト

### 設定ファイル
- `archive/docker/rviz/firebase_config.yaml` - Firebase接続設定
```yaml
  firebase:
    service_account_key: "/path/to/serviceAccountKey.json"
  coordinate_system:
    origin_latitude: 36.5513
    origin_longitude: 139.9286
    scale_factor: 0.01
```

## 依存関係

**ROS2パッケージ:**
- `ros-humble-nav2-bringup`
- `ros-humble-turtlebot3-gazebo`
- `ros-humble-gazebo-ros-pkgs`

**Pythonライブラリ:**
- `firebase-admin>=6.0.0`
- `google-cloud-firestore>=2.11.0`

**システム要件:**
- Docker Desktop (WSL2バックエンド推奨)
- GPU: Intel/NVIDIA (WSLg経由)
- メモリ: 8GB以上

## セットアップ

### 1. Firebase認証設定
```bash
# serviceAccountKey.json を配置
cp /path/to/serviceAccountKey.json config/
```

### 2. Docker環境起動
```bash
# VSCode Dev Containerで開く
# または
docker-compose up -d
docker exec -it ros2_firebase_container bash
```

### 3. パッケージビルド
```bash
cd /workspace
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash
```

### 4. 実行
```bash
# 最小構成（単一ロボット）
ros2 launch ros2_firebase_bridge phase2_minimal.launch.py

# 複数ロボット
ros2 launch ros2_firebase_bridge multi_robot.launch.py
```

## 主要機能

### Firebase連携
- **Destination監視**: Firestore `robots/{robot_id}/destination` の変更をリアルタイム検知
- **位置同期**: `/odom` トピックからGPS座標に変換してFirestore更新
- **無限ループ防止**: ハッシュベースの重複検知

### 座標変換
- GPS (WGS84) ↔ Gazebo Map座標系
- スケールファクター: `0.01` (GPS 100m = Map 1m)
- 原点: 宇都宮大学陽東キャンパス7号館

### Nav2統合
- AMCL自己位置推定
- DWB Local Planner
- Global/Local Costmap
- Goal到達判定（tolerance: 0.5m）

## トラブルシューティング

### Nav2が起動しない
```bash
# Nav2パッケージ確認
ros2 pkg list | grep nav2

# アクションサーバー確認
ros2 action list
```

### Firebase接続エラー
```bash
# 認証ファイル確認
ls -l /workspace/config/serviceAccountKey.json

# 接続テスト
python3 scripts/test_firebase.py
```

### 座標ずれ
```bash
# 座標変換デバッグ
python3 archive/legacy/scripts/coordinate_debug.py
```

## 制限事項

- Docker環境依存（ホストUbuntuでは動作不可）
- GPU必須（WSLg/Mesa）
- Nav2初期化に15秒程度必要
- 同時接続: 最大3ロボットまで検証済み

## 移行ガイド

**現行Ubuntu環境への移行:**
1. `README_current_jp.md` 参照
2. Nav2削除、軽量ナビゲーションに切り替え
3. Dockerなしでネイティブ実行

## 開発履歴

- **Phase 1**: 基本Firebase連携（2024年10月）
- **Phase 2**: 無限ループ防止、位置同期最適化（2024年11月）
- **Phase 3**: 複数ロボット対応（2024年12月）
- **Deprecated**: Ubuntu環境移行に伴いレガシー化（2025年1月）

## ライセンス

MIT License

## 連絡先

開発者: Yugo Obana  
所属: 宇都宮大学 機械システム工学科  
プロジェクト: Cloud-Robotics Integration Platform