#!/bin/bash
# Phase 2 複数ロボット実動作テストスクリプト

set -e

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║    Phase 2 複数ロボット実動作テスト                            ║"
echo "║    3台のロボットが同時に動作できるか検証                       ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# ROS2環境設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash 2>/dev/null || true

export TURTLEBOT3_MODEL=waffle

# 色付きログ
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ログ関数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ===== Step 1: 前提条件チェック =====
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 1: 前提条件チェック"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Firebase認証ファイルチェック
if [ ! -f "/workspace/config/serviceAccountKey.json" ]; then
    log_error "Firebase認証ファイルが見つかりません: /workspace/config/serviceAccountKey.json"
    exit 1
fi
log_success "Firebase認証ファイル確認完了"

# パッケージビルドチェック
if [ ! -d "/workspace/install/ros2_firebase_bridge" ]; then
    log_warning "パッケージがビルドされていません。ビルドを実行します..."
    cd /workspace
    colcon build --packages-select ros2_firebase_bridge
fi
log_success "パッケージビルド確認完了"

# ===== Step 2: Gazebo + Nav2起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 2: Gazebo + Nav2 起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "TurtleBot3 Worldを起動しています..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
log_info "Gazebo PID: $GAZEBO_PID"

sleep 10
log_success "Gazebo起動完了"

log_info "Navigation2を起動しています..."
ros2 launch turtlebot3_navigation2 navigation2.launch.py > /tmp/nav2.log 2>&1 &
NAV2_PID=$!
log_info "Nav2 PID: $NAV2_PID"

sleep 10
log_success "Nav2起動完了"

# ===== Step 3: Firebase Bridgeを複数起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 3: Firebase Bridge 3台起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Robot 1
log_info "Robot 001 を起動中..."
ros2 run ros2_firebase_bridge firebase_bridge \
    --ros-args \
    -p robot_id:=robot_001 \
    -p robot_namespace:=/robot1 \
    > /tmp/robot_001.log 2>&1 &
ROBOT1_PID=$!
sleep 3
log_success "Robot 001 起動完了 (PID: $ROBOT1_PID)"

# Robot 2
log_info "Robot 002 を起動中..."
ros2 run ros2_firebase_bridge firebase_bridge \
    --ros-args \
    -p robot_id:=robot_002 \
    -p robot_namespace:=/robot2 \
    > /tmp/robot_002.log 2>&1 &
ROBOT2_PID=$!
sleep 3
log_success "Robot 002 起動完了 (PID: $ROBOT2_PID)"

# Robot 3
log_info "Robot 003 を起動中..."
ros2 run ros2_firebase_bridge firebase_bridge \
    --ros-args \
    -p robot_id:=robot_003 \
    -p robot_namespace:=/robot3 \
    > /tmp/robot_003.log 2>&1 &
ROBOT3_PID=$!
sleep 3
log_success "Robot 003 起動完了 (PID: $ROBOT3_PID)"

# ===== Step 4: プロセス動作確認 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 4: プロセス動作確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

sleep 5

# プロセスチェック
ALL_RUNNING=true

if ! ps -p $GAZEBO_PID > /dev/null; then
    log_error "Gazebo が停止しています"
    ALL_RUNNING=false
else
    log_success "Gazebo 動作中"
fi

if ! ps -p $NAV2_PID > /dev/null; then
    log_error "Nav2 が停止しています"
    ALL_RUNNING=false
else
    log_success "Nav2 動作中"
fi

if ! ps -p $ROBOT1_PID > /dev/null; then
    log_error "Robot 001 が停止しています"
    ALL_RUNNING=false
else
    log_success "Robot 001 動作中"
fi

if ! ps -p $ROBOT2_PID > /dev/null; then
    log_error "Robot 002 が停止しています"
    ALL_RUNNING=false
else
    log_success "Robot 002 動作中"
fi

if ! ps -p $ROBOT3_PID > /dev/null; then
    log_error "Robot 003 が停止しています"
    ALL_RUNNING=false
else
    log_success "Robot 003 動作中"
fi

# ===== Step 5: ROS2ノード確認 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 5: ROS2ノード確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "ROS2ノードリスト:"
ros2 node list

echo ""
log_info "Firebase Bridgeノード確認:"
ros2 node list | grep firebase_bridge || log_warning "Firebase Bridgeノードが見つかりません"

# ===== Step 6: Firebaseデータ確認（Pythonスクリプト呼び出し） =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 6: Firebase データ確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Phase 2検証テストを実行中..."
python3 /workspace/scripts/phase2_validation_tests.py

if [ $? -eq 0 ]; then
    log_success "Phase 2検証テスト合格"
else
    log_warning "Phase 2検証テストで問題が検出されました"
fi

# ===== Step 7: 結果サマリー =====
echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    テスト結果サマリー                          ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

if [ "$ALL_RUNNING" = true ]; then
    log_success "すべてのプロセスが正常に動作しています"
    echo ""
    echo "📝 次のステップ:"
    echo "  1. Webブラウザで http://localhost:8000 を開く"
    echo "  2. 地図上でロボットマーカーを確認"
    echo "  3. センサーダッシュボードでテレメトリデータを確認"
    echo ""
    echo "🛑 停止方法:"
    echo "  Ctrl+C を押すか、以下のコマンドを実行:"
    echo "  kill $GAZEBO_PID $NAV2_PID $ROBOT1_PID $ROBOT2_PID $ROBOT3_PID"
else
    log_error "一部のプロセスが正常に起動していません"
    echo "ログファイルを確認してください:"
    echo "  - /tmp/gazebo.log"
    echo "  - /tmp/nav2.log"
    echo "  - /tmp/robot_001.log"
    echo "  - /tmp/robot_002.log"
    echo "  - /tmp/robot_003.log"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# プロセスIDを保存（手動で停止できるように）
cat > /tmp/multi_robot_pids.txt <<EOF
GAZEBO_PID=$GAZEBO_PID
NAV2_PID=$NAV2_PID
ROBOT1_PID=$ROBOT1_PID
ROBOT2_PID=$ROBOT2_PID
ROBOT3_PID=$ROBOT3_PID
EOF

log_info "プロセスIDを /tmp/multi_robot_pids.txt に保存しました"

# 無限ループで待機（Ctrl+Cで停止）
log_info "テスト実行中... Ctrl+C で停止します"
trap "echo ''; log_info 'テストを停止しています...'; kill $GAZEBO_PID $NAV2_PID $ROBOT1_PID $ROBOT2_PID $ROBOT3_PID 2>/dev/null; exit 0" INT

# ログのtail表示（オプション）
read -p "ログをリアルタイムで表示しますか? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    tail -f /tmp/robot_001.log /tmp/robot_002.log /tmp/robot_003.log
else
    # 無限待機
    while true; do
        sleep 10
        # プロセスが全て死んでいたら終了
        if ! ps -p $GAZEBO_PID > /dev/null && \
           ! ps -p $NAV2_PID > /dev/null && \
           ! ps -p $ROBOT1_PID > /dev/null && \
           ! ps -p $ROBOT2_PID > /dev/null && \
           ! ps -p $ROBOT3_PID > /dev/null; then
            log_error "すべてのプロセスが停止しました"
            exit 1
        fi
    done
fi