⚠️ Work in Progress — This repository is under continuous update
# ROS2 Firebase Bridge for Personal Mobility Platform

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![Firebase](https://img.shields.io/badge/Firebase-Admin_SDK-orange.svg)](https://firebase.google.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS2ロボットシステムとFirebase Firestoreをリアルタイムで双方向接続するブリッジシステム。クラウドベースのフリート管理、遠隔監視、Webベースの制御インターフェースを実現します。

## 🎯 プロジェクト概要

本プロジェクトは、ROS2（Robot Operating System 2）とGoogle Firebaseを統合する本格的な実装で、パーソナルモビリティプラットフォームや自律配送ロボット向けに設計されています。以下の機能を提供します：

- **リアルタイム位置追跡**: GPS同期によるロボット位置更新と座標変換
- **遠隔ナビゲーション**: クラウド経由での目的地指示とNav2統合
- **センサーテレメトリ**: 集約されたセンサーデータストリーミング（LiDAR、IMU、バッテリー、オドメトリ）
- **複数ロボット対応**: 同時に複数のロボットを管理できるスケーラブルなアーキテクチャ
- **メモリリーク防止**: 自動クリーンアップ機構を備えた高度な重複検知システム

### 主要機能

✅ **双方向同期**: ROS2 ↔ Firestore リアルタイムデータフロー
✅ **無限ループ防止**: ハッシュベースの重複排除による冪等な目的地処理
✅ **位置更新の最適化**: 距離閾値ベースのフィルタリング（0.5m閾値）
✅ **座標系変換**: GPS ↔ ROS2マップ座標の変換（スケール係数設定可能）
✅ **Nav2統合**: ROS2 Navigation2スタックとのシームレスな統合
✅ **Docker開発環境**: WSLg GPU対応の事前設定済み`.devcontainer`

---

## 🏗️ システムアーキテクチャ

```
┌─────────────────────────────────────────────────────────┐
│                  Webインターフェース                     │
│            (Firebase Realtime Database)                 │
└───────────────────┬─────────────────────────────────────┘
                    │ Firestore API
                    │
┌───────────────────▼─────────────────────────────────────┐
│             Firebase Bridgeノード                       │
│  ┌──────────────────────────────────────────────────┐  │
│  │  • 目的地リスナー (Firestore → ROS2)             │  │
│  │  • 状態パブリッシャー (ROS2 → Firestore)         │  │
│  │  • 座標変換器 (GPS ↔ マップ)                     │  │
│  │  • センサー集約器 (テレメトリ)                   │  │
│  └──────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────────────────────┘
                    │ ROS2 トピック・アクション
                    │
┌───────────────────▼─────────────────────────────────────┐
│              ROS2 ナビゲーションスタック                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │  • Nav2 (navigate_to_poseアクション)             │  │
│  │  • AMCL (自己位置推定)                           │  │
│  │  • Costmap (障害物回避)                          │  │
│  └──────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────────────────────┘
                    │ センサーデータ
                    │
┌───────────────────▼─────────────────────────────────────┐
│           TurtleBot3 / 実機ロボット                     │
│  • オドメトリ (/odom)                                   │
│  • LiDARスキャン (/scan)                                │
│  • IMU (/imu)                                           │
│  • バッテリー状態 (/battery_state)                      │
└─────────────────────────────────────────────────────────┘
```

---

## 📋 前提条件

### ソフトウェア要件

- **ROS2 Humble** (Desktop Fullインストール推奨)
- **Ubuntu 22.04 LTS** (または互換性のあるLinuxディストリビューション)
- **Python 3.10+**
- **Docker** (コンテナ開発用)
- **VS Code** + Remote-Containers拡張機能 (推奨)

### ハードウェア対応

- **TurtleBot3 Waffle** (シミュレーション・実機両対応)
- **LiDARセンサー** (例: LDS-01/02)
- **IMUセンサー** (オプション、自己位置推定精度向上用)
- **GPSモジュール** (屋外運用時)

### Firebase セットアップ

1. [Firebase Console](https://console.firebase.google.com/)でFirebaseプロジェクトを作成
2. **Firestore Database**を有効化（ネイティブモード）
3. **サービスアカウントキー**（JSONファイル）をダウンロード
4. キーを`/workspace/config/serviceAccountKey.json`に配置

---

## 🚀 クイックスタート

### 方法1: Docker開発コンテナ（推奨）

```bash
# リポジトリをクローン
git clone https://github.com/yourusername/ros2-firebase-bridge.git
cd ros2-firebase-bridge

# VS CodeでDev Containerを開く
code .
# 実行: Ctrl+Shift+P → "Dev Containers: Reopen in Container"

# コンテナ内でワークスペースをビルド
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash

# ブリッジを実行
ros2 run ros2_firebase_bridge firebase_bridge --ros-args -p robot_id:=robot_001
```

### 方法2: ネイティブインストール

```bash
# ROS2 Humbleをインストール（未インストールの場合）
# 参照: https://docs.ros.org/en/humble/Installation.html

# Python依存関係をインストール
pip3 install firebase-admin google-cloud-firestore python-dotenv --break-system-packages

# クローン・ビルド
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2-firebase-bridge.git
cd ~/ros2_ws
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash

# Firebase認証情報を設定
cp serviceAccountKey.json ~/ros2_ws/src/ros2-firebase-bridge/config/

# シミュレーション + ブリッジを起動
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py
ros2 run ros2_firebase_bridge firebase_bridge
```

---

## 📦 パッケージ構造

```
ros2_firebase_bridge/
├── ros2_firebase_bridge/
│   ├── __init__.py
│   ├── firebase_bridge_node.py       # メインブリッジノード
│   ├── firebase_client.py            # Firestore接続マネージャー
│   ├── coordinate_converter.py       # GPS ↔ マップ座標変換
│   ├── state_publisher.py            # フィルタリング付きスマート状態発行
│   └── sensor_aggregator.py          # センサーデータ集約
├── config/
│   └── rviz/
│       └── firebase_config.yaml      # 設定ファイル
├── launch/
│   ├── multi_robot_firebase_bridge.launch.py
│   └── phase2_minimal.launch.py
├── scripts/
│   ├── firebase_bridge               # 実行エントリポイント
│   ├── test_firebase.py              # 接続テストスクリプト
│   ├── test_infinite_loop.py         # ループ防止テスト
│   └── phase2_validation_tests.py    # 包括的テストスイート
├── worlds/
│   └── phase2_minimal.world          # 軽量Gazeboワールド
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## ⚙️ 設定

### Firebase設定 (`config/rviz/firebase_config.yaml`)

```yaml
firebase:
  service_account_key: "/workspace/config/serviceAccountKey.json"
  database_url: "https://your-project.firebaseio.com"

ros2:
  robot_namespace: "/turtlebot3"
  goal_topic: "/goal_pose"
  odom_topic: "/odom"

coordinate_system:
  origin_latitude: 36.55077    # マップ原点のGPS座標
  origin_longitude: 139.92957
  scale_factor: 0.01           # GPS距離 → マップ距離 (0.01 = GPS100m → マップ1m)

  map_bounds:
    x_min: -4.0
    x_max: 4.0
    y_min: -4.0
    y_max: 4.0

telemetry:
  update_interval: 2.0          # テレメトリ送信レート（秒）
  position_update_interval: 2.0 # 位置更新レート
```

### 起動パラメータ

```bash
# カスタムIDの単一ロボット
ros2 run ros2_firebase_bridge firebase_bridge \
  --ros-args \
  -p robot_id:=robot_001 \
  -p robot_namespace:=/turtlebot3

# 複数ロボット（launchファイル使用）
ros2 launch ros2_firebase_bridge multi_robot_firebase_bridge.launch.py
```

---

## 🔧 コアコンポーネント

### 1. Firebase Bridgeノード (`firebase_bridge_node.py`)

全サブシステムを管理するメインオーケストレーター。

**主な責務:**
- Firestoreリアルタイムリスナーによる目的地更新
- Nav2アクションクライアントによるナビゲーション目標配信
- 重複目的地検出（ハッシュベース + タイムスタンプ追跡）
- メモリリーク防止（古いハッシュの自動クリーンアップ）
- Nav2初期化シーケンス管理

**無限ループ防止機構:**
```python
# 浮動小数点誤差に対応した丸め処理付きハッシュ計算
lat_rounded = round(destination.latitude, 5)  # 約1.1m精度
lng_rounded = round(destination.longitude, 5)
hash_input = f"{lat_rounded:.5f}_{lng_rounded:.5f}"
destination_hash = hashlib.md5(hash_input.encode()).hexdigest()

# スレッドセーフな重複検知
with self.destination_lock:
    if destination_hash == self.last_processed_destination_hash:
        return  # 重複をスキップ
```

### 2. 座標変換器 (`coordinate_converter.py`)

GPS ↔ ROS2マップ座標の変換を処理。

**機能:**
- ハバーサイン公式による正確な距離計算
- シミュレーション環境用の設定可能なスケール係数
- マップ境界の自動適用
- 範囲外座標の安全な補正

**使用例:**
```python
converter = CoordinateConverter(
    origin_lat=36.55077,
    origin_lng=139.92957,
    scale_factor=0.01  # GPS100m = マップ1m
)

# GPS → マップ
map_coords = converter.gps_to_map_coordinates(36.55080, 139.92960)
# 戻り値: {'x': 0.33, 'y': 0.03}

# マップ → GPS
gps_coords = converter.map_to_gps_coordinates(1.0, 2.0)
# 戻り値: {'lat': 36.55257, 'lng': 139.93757}
```

### 3. 状態パブリッシャー (`state_publisher.py`)

レート制限と閾値ベースフィルタリングを備えたインテリジェントな状態発行。

**最適化機能:**
- **位置閾値**: 更新前に最低0.5m移動必要
- **時間ベースのレート制限**: 最大1秒に1回の更新
- **方位角閾値**: 最低0.1ラジアンの回転

**メリット:**
- Firestore書き込み操作の削減（コスト削減）
- Firebase→ROS2エコーループの防止
- Web UIマーカー更新の滑らかさ向上

### 4. センサー集約器 (`sensor_aggregator.py`)

複数のROS2トピックからセンサーデータを収集・集約。

**対応センサー:**
- **LiDAR** (`/scan`): 障害物検出、最小距離
- **IMU** (`/imu`): 加速度マグニチュード、姿勢
- **バッテリー** (`/battery_state`): 電圧、電流、充電率
- **オドメトリ** (`/odom`): 速度、位置

**テレメトリデータ形式（Firestore）:**
```json
{
  "telemetry": {
    "speed": 0.22,
    "battery_percent": 85.5,
    "obstacle_detected": false,
    "min_obstacle_distance": 2.35,
    "distance_to_goal": 3.42
  }
}
```

---

## 🧪 テスト

### 接続テスト

```bash
# Firebase接続をテスト
python3 scripts/test_firebase.py

# 期待される出力:
# ✅ Firebase Admin SDK初期化成功
# ✅ Firestore接続成功
# ✅ テストデータ書き込み成功
```

### 無限ループ防止テスト

```bash
# 重複目的地処理をテスト
python3 scripts/test_infinite_loop.py

# テスト内容:
# 1. 重複目的地フィルタリング
# 2. 高頻度更新処理
# 3. ROS2エコーバック防止
```

### Phase 2検証スイート

```bash
# 包括的統合テスト
python3 scripts/phase2_validation_tests.py

# テスト内容:
# 1. 無限ループ防止（100回配車）
# 2. 位置同期パフォーマンス（30秒テスト）
# 3. 複数ロボット動作（3台）
```

---

## 📊 パフォーマンス特性

### 位置更新の最適化

| 指標 | 最適化前 | 最適化後 |
|------|---------|---------|
| Firestore書き込み/分 | 約60回 | 約6回 |
| 平均更新間隔 | 1秒 | 5-10秒 |
| 移動閾値 | なし | 0.5m |
| コスト削減率 | ベースライン | **90%** |

### メモリ管理

- **自動ハッシュクリーンアップ**: 古い目的地ハッシュ（1時間以上）を10分ごとに削除
- **スレッドセーフ操作**: すべての目的地処理でミューテックスロックを使用
- **制限されたメモリ成長**: ハッシュ辞書は長期運用中も一定サイズを維持

---

## 🔍 トラブルシューティング

### Firebase Bridgeが起動しない

**症状:** `firebase_bridge`ノードの起動に失敗

**解決策:**
1. サービスアカウントキーのパスを確認:
   ```bash
   ls -la /workspace/config/serviceAccountKey.json
   ```
2. Firebase認証情報を検証:
   ```bash
   python3 scripts/test_firebase.py
   ```
3. ログ出力を確認:
   ```bash
   ros2 run ros2_firebase_bridge firebase_bridge --ros-args --log-level DEBUG
   ```

### ナビゲーション目標が受け付けられない

**症状:** Nav2が目標を"REJECTED"ステータスで拒否

**解決策:**
1. RVizで初期位置を設定:
   - "2D Pose Estimate"をクリック
   - ロボットがいる位置の地図上をクリック
2. AMCL自己位置推定を確認:
   ```bash
   ros2 topic echo /amcl_pose
   ```
3. コストマップを確認:
   ```bash
   ros2 topic list | grep costmap
   ```

### Firebaseで位置が更新されない

**症状:** Webインターフェースでロボット位置が固定

**解決策:**
1. オドメトリトピックを確認:
   ```bash
   ros2 topic hz /odom
   ```
2. ブリッジがオドメトリを受信しているか確認:
   ```bash
   ros2 node info /phase2_firebase_bridge
   ```
3. 位置閾値を確認（0.5m以上移動が必要な場合あり）:
   ```python
   # firebase_config.yaml内
   telemetry:
     position_update_interval: 1.0  # 頻度を増やす
   ```

---

## 🛣️ ロードマップ

### 完了済み（Phase 2）
- ✅ ハッシュベースの重複排除による無限ループ防止
- ✅ 位置同期最適化（閾値ベースフィルタリング）
- ✅ 複数ロボット対応（名前空間ベースアーキテクチャ）
- ✅ メモリリーク防止（自動クリーンアップ）
- ✅ Nav2初期化シーケンス処理

### 計画中（今後のフェーズ）
- 🔲 AWS IoT Core統合（ハイブリッドクラウド）
- 🔲 リアルタイム映像ストリーミング
- 🔲 フリート全体のタスクスケジューリング
- 🔲 予知保全アラート
- 🔲 WebクライアントむけGraphQL API
- 🔲 ROS2 Jazzyサポート

---

## 📚 ドキュメント

### 関連リソース

- [ROS2 Humbleドキュメント](https://docs.ros.org/en/humble/)
- [Firebase Admin Python SDK](https://firebase.google.com/docs/admin/setup)
- [Nav2ドキュメント](https://navigation.ros.org/)
- [TurtleBot3マニュアル](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

### 学術的背景

本プロジェクトは、宇都宮大学におけるパーソナルモビリティ研究の一環として開発され、自律配送システム向けのクラウドロボティクス統合に焦点を当てています。

---

## 🤝 コントリビューション

コントリビューションを歓迎します！以下のガイドラインに従ってください：

1. リポジトリをフォーク
2. 機能ブランチを作成 (`git checkout -b feature/amazing-feature`)
3. 変更をコミット (`git commit -m 'Add amazing feature'`)
4. ブランチにプッシュ (`git push origin feature/amazing-feature`)
5. プルリクエストを作成

### コードスタイル

- Pythonコードは[PEP 8](https://pep8.org/)に従う
- 説明的な変数名を使用
- すべての関数にdocstringを追加
- 適用可能な箇所で型ヒントを含める

---

## 📄 ライセンス

本プロジェクトはMITライセンスの下でライセンスされています。詳細は[LICENSE](LICENSE)ファイルを参照してください。

---

## 👤 著者

**尾花　優冴（Yugo Obana）**
宇都宮大学 機械工学科
専門分野: クラウドロボティクス統合、自律モビリティシステム

### 連絡先
- LinkedIn: [your-profile](www.linkedin.com/in/yugo-dev)
- GitHub: [your-github](https://github.com/Iruazu)

---

## 🙏 謝辞

- **ROS2コミュニティ** - 堅牢なロボティクスミドルウェアの提供
- **Google Firebase** - スケーラブルなクラウドインフラの提供
- **ROBOTIS** - TurtleBot3プラットフォームとドキュメントの提供
- **宇都宮大学** - 研究支援

---

## 📧 サポート

質問や問題がある場合:
- [Issue](https://github.com/Iruazu/mobility-ros2-firebase/issues)を開く
- メール: ygnk0805@outlook.jp

---

**自律モビリティとクラウドロボティクスのために ❤️ を込めて開発**