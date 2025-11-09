# mobility-ros2-firebase (現行Ubuntu環境)

> 🚀 **現行実装**: Ubuntu 22.04 + ROS2 Humble + Firebase Dispatch連携  
> Nav2非依存の軽量リアルタイム配車システム

## 概要

宇都宮大学 機械システム工学科 Yugo Obanaが開発した、Firebase経由でリアルタイム配車を行うROS2システムです。

**主な特徴:**
- **ネイティブUbuntu実行** (Docker不要)
- **Firebase Dispatch連携** (GPS座標 → Gazebo制御)
- **カスタムキャンパスモデル** (宇都宮大学陽東キャンパス)
- **軽量ナビゲーション** (Nav2なし、直接cmd_vel制御)
- **リアルタイム位置同期** (Firestore ↔ ROS2 Odometry)

## アーキテクチャ
```
Ubuntu 22.04 (Native)
├── Gazebo
│   └── Utsunomiya Campus Model (.dae)
├── ROS2 Nodes
│   ├── simple_goal_navigator (メインノード)
│   │   ├── Firebase Realtime DB Polling
│   │   ├── GPS → Gazebo 座標変換
│   │   ├── cmd_vel Control
│   │   └── Firestore Position Sync
│   └── robot_state_publisher
└── Firebase
    ├── Realtime DB (goal配信)
    └── Firestore (位置・状態同期)
```

## 構成

### メインコード
- `src/ros2_firebase_bridge/ros2_firebase_bridge/simple_goal_navigator.py`
  - Firebase Realtime DBからgoal取得 (2Hz polling)
  - GPS → Gazebo座標変換
  - PID風制御でロボット移動
  - Firestoreへ位置同期 (5Hz)

### Launchファイル
- `src/ros2_firebase_bridge/launch/utsunomiya_campus.launch.py`
  - Gazebo起動
  - キャンパスモデル読み込み
  - TurtleBot3 spawn

### 3Dモデル
- `src/ros2_firebase_bridge/models/utsunomiya_campus/`
  - `utunomiya-yoto7.dae` - Blender GISで生成
  - `model.sdf` - Gazebo定義

### 設定
- `src/ros2_firebase_bridge/config/firebase_config.yaml`
```yaml
  firebase:
    service_account_key: "/home/obana/mobility-ros2-firebase/src/ros2_firebase_bridge/config/serviceAccountKey.json"
    database_url: "https://mobility-map-ae58e-default-rtdb.asia-southeast1.firebasedatabase.app/"
  map:
    origin:
      latitude: 36.5546169518534
      longitude: 139.87190842628479
    scale: 111320.0
```

## セットアップ

### 前提条件
- Ubuntu 22.04 LTS
- ROS2 Humble (Desktop Full)
- Gazebo 11
- Python 3.10+

### 1. 依存関係インストール
```bash
# ROS2 Humble (未インストールの場合)
sudo apt update
sudo apt install ros-humble-desktop-full

# Gazebo & TurtleBot3
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3*

# Python依存関係
pip install firebase-admin google-cloud-firestore PyYAML --break-system-packages
```

### 2. Firebase設定
```bash
# serviceAccountKey.json を配置
mkdir -p ~/mobility-ros2-firebase/src/ros2_firebase_bridge/config
cp /path/to/serviceAccountKey.json ~/mobility-ros2-firebase/src/ros2_firebase_bridge/config/
```

### 3. ビルド
```bash
cd ~/mobility-ros2-firebase
colcon build --packages-select ros2_firebase_bridge
source install/setup.bash
```

### 4. 環境変数
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/mobility-ros2-firebase/src/ros2_firebase_bridge/models
```

## 実行

### 基本起動
```bash
# Terminal 1: Gazebo + Campus Model
ros2 launch ros2_firebase_bridge utsunomiya_campus.launch.py

# Terminal 2: Simple Goal Navigator
ros2 run ros2_firebase_bridge simple_goal_navigator
```

### 動作確認
1. **Firebase Realtime DBにgoal設定**
```json
   // Path: robot/goal
   {
     "x": 36.551291,  // 緯度
     "y": 139.928716  // 経度
   }
```

2. **Gazeboで移動確認**
   - TurtleBot3が目標に向かって移動
   - 到達後 (tolerance 0.3m) にgoal削除

3. **Firestoreで位置確認**
```javascript
   // Collection: robots/robot_001
   {
     position: GeoPoint(36.551291, 139.928716),
     status: "moving" | "idle"
   }
```

## 座標変換ロジック

### GPS → Gazebo
```python
# 基準点: Gazebo (0,0) = GPS (36.551291, 139.928716)
base_lat = 36.551291
base_lng = 139.928716

# 1度あたりの距離
meters_per_lat = 111320.0  # 緯度
meters_per_lng = 91290.0   # 経度 (日本付近)

# 変換
gazebo_y = (gps_lat - base_lat) * meters_per_lat
gazebo_x = (gps_lng - base_lng) * meters_per_lng
```

### Gazebo → GPS
```python
gps_lat = base_lat + (gazebo_y / meters_per_lat)
gps_lng = base_lng + (gazebo_x / meters_per_lng)
```

## ナビゲーション制御

### 制御ループ (10Hz)
```python
# 距離・角度計算
dx = goal_x - current_x
dy = goal_y - current_y
distance = sqrt(dx² + dy²)
goal_angle = atan2(dy, dx)
angle_diff = normalize(goal_angle - current_yaw)

# 制御
if distance < 0.3:  # ゴール到達
    stop()
elif abs(angle_diff) > 0.2:  # 回転
    rotate(angular_speed)
else:  # 前進
    move_forward(linear_speed)
```

## トラブルシューティング

### ロボットが動かない
```bash
# Odometry確認
ros2 topic echo /odom --once

# cmd_vel確認
ros2 topic echo /cmd_vel
```

### Firebase接続エラー
```bash
# 認証ファイル確認
ls -l ~/mobility-ros2-firebase/src/ros2_firebase_bridge/config/serviceAccountKey.json

# 接続テスト
python3 scripts/test_firebase.py
```

### Gazeboモデルが表示されない
```bash
# モデルパス確認
echo $GAZEBO_MODEL_PATH

# モデルファイル確認
ls -l src/ros2_firebase_bridge/models/utsunomiya_campus/
```

### Firestore Quota Exceeded
```
⚠️ 無料プラン: 20,000 writes/day
対策: 位置更新頻度を5Hzから1Hzに削減
```

## 制限事項

- **Nav2非対応**: 障害物回避なし
- **単一ロボット**: 現状robot_001のみ
- **GPS精度**: ±1m程度の誤差あり
- **Firestore制限**: 書き込み頻度に注意

## Work in Progress

以下の機能は開発中です:

- [ ] Nav2統合 (自律障害物回避)
- [ ] 複数ロボット同時制御
- [ ] Web UIダッシュボード
- [ ] 実機テスト (実際のTurtleBot3)

## 開発ロードマップ

1. **Phase 4** (現在): Firebase Dispatch完成
2. **Phase 5** (2025年3月): Nav2統合
3. **Phase 6** (2025年6月): 実機デプロイ

## ライセンス

MIT License

## 連絡先

開発者: Yugo Obana  
所属: 宇都宮大学 機械システム工学科  
目標: Mobility × Cloud × AI Architect  