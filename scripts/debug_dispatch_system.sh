#!/bin/bash
# 配車デモ用統合起動スクリプト（改善版）

set -e

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║         配車デモ統合起動スクリプト（改善版）                  ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# ROS2環境設定
source /opt/ros/humble/setup.bash

if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
else
    echo "⚠️ Workspaceがビルドされていません"
    echo "以下を実行してください:"
    echo "  cd /workspace && colcon build --packages-select ros2_firebase_bridge"
    exit 1
fi

export TURTLEBOT3_MODEL=waffle

# 色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ===== Step 1: 前提条件チェック =====
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 1: 前提条件チェック"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Firebase認証ファイル確認
if [ ! -f "/workspace/config/serviceAccountKey.json" ]; then
    log_error "Firebase認証ファイルが見つかりません"
    exit 1
fi
log_success "Firebase認証ファイル確認完了"

# パッケージ確認
if [ ! -f "/workspace/install/ros2_firebase_bridge/lib/ros2_firebase_bridge/firebase_bridge" ]; then
    log_error "Firebase Bridgeの実行ファイルが見つかりません"
    log_info "パッケージをビルドしてください: cd /workspace && colcon build"
    exit 1
fi
log_success "Firebase Bridge実行ファイル確認完了"

# ===== Step 2: ログディレクトリ作成 =====
LOG_DIR="/tmp/dispatch_demo_logs"
mkdir -p $LOG_DIR
log_info "ログディレクトリ: $LOG_DIR"

# ===== Step 3: Gazebo起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 2: Gazebo起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "TurtleBot3 Worldを起動しています..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > $LOG_DIR/gazebo.log 2>&1 &
GAZEBO_PID=$!
log_info "Gazebo PID: $GAZEBO_PID"

log_info "Gazeboの起動を待機中（15秒）..."
sleep 15
log_success "Gazebo起動完了"

# ===== Step 4: Nav2起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 3: Navigation2起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Navigation2を起動しています..."
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true > $LOG_DIR/nav2.log 2>&1 &
NAV2_PID=$!
log_info "Nav2 PID: $NAV2_PID"

log_info "Nav2の起動を待機中（20秒）..."
sleep 20
log_success "Nav2起動完了"

# ===== Step 5: Firebase Bridge起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 4: Firebase Bridge起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Firebase Bridgeを起動しています..."
ros2 run ros2_firebase_bridge firebase_bridge \
    --ros-args \
    -p robot_id:=robot_001 \
    -p robot_namespace:=/turtlebot3 \
    --log-level INFO \
    > $LOG_DIR/firebase_bridge.log 2>&1 &
BRIDGE_PID=$!

log_info "Firebase Bridge PID: $BRIDGE_PID"

log_info "Firebase Bridgeの初期化を待機中（10秒）..."
sleep 10

# ===== Step 6: プロセス動作確認 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 5: プロセス動作確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

ALL_RUNNING=true

if ! ps -p $GAZEBO_PID > /dev/null; then
    log_error "Gazebo が停止しています"
    log_info "ログ確認: tail -50 $LOG_DIR/gazebo.log"
    ALL_RUNNING=false
else
    log_success "Gazebo 動作中"
fi

if ! ps -p $NAV2_PID > /dev/null; then
    log_error "Nav2 が停止しています"
    log_info "ログ確認: tail -50 $LOG_DIR/nav2.log"
    ALL_RUNNING=false
else
    log_success "Nav2 動作中"
fi

if ! ps -p $BRIDGE_PID > /dev/null; then
    log_error "Firebase Bridge が停止しています"
    log_info "ログ確認: tail -50 $LOG_DIR/firebase_bridge.log"
    echo ""
    echo "最後の50行を表示:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    tail -50 $LOG_DIR/firebase_bridge.log
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    ALL_RUNNING=false
else
    log_success "Firebase Bridge 動作中"
fi

# ===== Step 7: ROS2ノード確認 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 6: ROS2ノード確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Firebase Bridgeノード検索:"
if ros2 node list | grep -q firebase; then
    log_success "Firebase Bridgeノードが起動しています"
    ros2 node list | grep firebase
else
    log_warning "Firebase Bridgeノードが見つかりません"
fi

log_info ""
log_info "重要なトピック確認:"
ros2 topic list | grep -E "(odom|scan|goal_pose)" || log_warning "一部のトピックが見つかりません"

# ===== Step 8: Web UIの起動案内 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 7: Web UI起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Web UIを起動するには、別のターミナルで以下を実行:"
echo ""
echo "  cd /workspace"
echo "  python3 -m http.server 8000"
echo ""
log_info "その後、ブラウザで http://localhost:8000 を開いてください"

# ===== 結果サマリー =====
echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    起動結果サマリー                            ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

if [ "$ALL_RUNNING" = true ]; then
    log_success "すべてのプロセスが正常に動作しています"
    echo ""
    echo "📝 配車デモの手順:"
    echo "  1. Web UI (http://localhost:8000) を開く"
    echo "  2. 地図上にロボットマーカーが表示されることを確認"
    echo "  3. 地図をクリックして配車リクエスト"
    echo "  4. ロボットが移動することを確認"
    echo ""
    echo "🔍 デバッグ情報:"
    echo "  ログファイル:"
    echo "    - Gazebo: $LOG_DIR/gazebo.log"
    echo "    - Nav2: $LOG_DIR/nav2.log"
    echo "    - Firebase Bridge: $LOG_DIR/firebase_bridge.log"
    echo ""
    echo "  ROS2コマンド:"
    echo "    - ros2 node list"
    echo "    - ros2 topic echo /goal_pose"
    echo "    - ros2 topic echo /odom"
    echo ""
    echo "🛑 停止方法:"
    echo "  Ctrl+C または以下のコマンド:"
    echo "  kill $GAZEBO_PID $NAV2_PID $BRIDGE_PID"
else
    log_error "一部のプロセスが正常に起動していません"
    echo ""
    echo "🔧 トラブルシューティング:"
    echo ""
    echo "1. ログファイルを確認:"
    echo "   tail -100 $LOG_DIR/firebase_bridge.log"
    echo ""
    echo "2. Firebase認証を確認:"
    echo "   python3 /workspace/scripts/test_firebase.py"
    echo ""
    echo "3. パッケージを再ビルド:"
    echo "   cd /workspace && colcon build --packages-select ros2_firebase_bridge"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# プロセスIDを保存
cat > $LOG_DIR/pids.txt <<EOF
GAZEBO_PID=$GAZEBO_PID
NAV2_PID=$NAV2_PID
BRIDGE_PID=$BRIDGE_PID
EOF

log_info "プロセスIDを $LOG_DIR/pids.txt に保存しました"

# 待機
if [ "$ALL_RUNNING" = true ]; then
    log_info "実行中... Ctrl+C で停止します"
    trap "echo ''; log_info 'プロセスを停止しています...'; kill $GAZEBO_PID $NAV2_PID $BRIDGE_PID 2>/dev/null; exit 0" INT

    echo ""
    read -p "Firebase Bridgeのログをリアルタイムで表示しますか? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        tail -f $LOG_DIR/firebase_bridge.log
    else
        while true; do
            sleep 10
            if ! ps -p $GAZEBO_PID > /dev/null || \
               ! ps -p $NAV2_PID > /dev/null || \
               ! ps -p $BRIDGE_PID > /dev/null; then
                log_error "プロセスが停止しました"
                exit 1
            fi
        done
    fi
else
    exit 1
fi