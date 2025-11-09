#!/bin/bash
# Phase 2 単一ロボット起動スクリプト（Docker環境対応版）

set -e

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║    Phase 2 単一ロボット起動（Docker環境対応）                  ║"
echo "║    軽量環境での動作確認                                        ║"
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
NC='\033[0m'

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

if [ ! -f "/workspace/config/serviceAccountKey.json" ]; then
    log_error "Firebase認証ファイルが見つかりません"
    exit 1
fi
log_success "Firebase認証ファイル確認完了"

if [ ! -f "/workspace/install/ros2_firebase_bridge/lib/ros2_firebase_bridge/firebase_bridge" ]; then
    log_error "Firebase Bridgeの実行ファイルが見つかりません"
    log_info "パッケージをビルドしてください: cd /workspace && colcon build"
    exit 1
fi
log_success "Firebase Bridge実行ファイル確認完了"

# ===== Step 2: Gazebo起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 2: Gazebo起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "TurtleBot3 Worldを起動しています..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > /tmp/gazebo_single.log 2>&1 &
GAZEBO_PID=$!
log_info "Gazebo PID: $GAZEBO_PID"

sleep 15
log_success "Gazebo起動完了"

# ===== Step 3: Nav2起動 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 3: Navigation2起動"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Navigation2を起動しています..."
ros2 launch turtlebot3_navigation2 navigation2.launch.py > /tmp/nav2_single.log 2>&1 &
NAV2_PID=$!
log_info "Nav2 PID: $NAV2_PID"

sleep 15
log_success "Nav2起動完了"

# ===== Step 4: Firebase Bridge起動（Docker対応版） =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 4: Firebase Bridge起動（Docker環境対応）"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Firebase Bridgeを起動しています..."
log_info "実行パス: /workspace/install/ros2_firebase_bridge/lib/ros2_firebase_bridge/firebase_bridge"

/workspace/install/ros2_firebase_bridge/lib/ros2_firebase_bridge/firebase_bridge \
    --ros-args \
    -p robot_id:=robot_001 \
    -p robot_namespace:=/turtlebot3 \
    --log-level INFO \
    > /tmp/firebase_bridge.log 2>&1 &
BRIDGE_PID=$!

log_info "Firebase Bridge PID: $BRIDGE_PID"

sleep 10

# ===== Step 5: プロセス動作確認 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 5: プロセス動作確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

ALL_RUNNING=true

if ! ps -p $GAZEBO_PID > /dev/null; then
    log_error "Gazebo が停止しています"
    log_info "ログ確認: tail -50 /tmp/gazebo_single.log"
    ALL_RUNNING=false
else
    log_success "Gazebo 動作中"
fi

if ! ps -p $NAV2_PID > /dev/null; then
    log_error "Nav2 が停止しています"
    log_info "ログ確認: tail -50 /tmp/nav2_single.log"
    ALL_RUNNING=false
else
    log_success "Nav2 動作中"
fi

if ! ps -p $BRIDGE_PID > /dev/null; then
    log_error "Firebase Bridge が停止しています"
    log_info "ログ確認: tail -50 /tmp/firebase_bridge.log"
    log_info ""
    log_info "最後の50行を表示:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    tail -50 /tmp/firebase_bridge.log
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    ALL_RUNNING=false
else
    log_success "Firebase Bridge 動作中"
fi

# ===== Step 6: ROS2ノード確認 =====
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 6: ROS2ノード確認"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

log_info "Firebase Bridgeノード検索:"
ros2 node list | grep firebase || log_warning "Firebase Bridgeノードが見つかりません"

log_info ""
log_info "利用可能なトピック確認:"
ros2 topic list | grep -E "(odom|scan|goal_pose)" || true

# ===== Step 7: 結果サマリー =====
echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    起動結果サマリー                            ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

if [ "$ALL_RUNNING" = true ]; then
    log_success "すべてのプロセスが正常に動作しています"
    echo ""
    echo "📝 次のステップ:"
    echo "  1. Webブラウザでhttp://localhost:8000を開く"
    echo "  2. 地図上でロボットマーカーを確認"
    echo "  3. 地図をクリックして配車テスト"
    echo ""
    echo "🛑 停止方法:"
    echo "  Ctrl+C または以下のコマンド:"
    echo "  kill $GAZEBO_PID $NAV2_PID $BRIDGE_PID"
    echo ""
    echo "🔍 デバッグ:"
    echo "  ログファイル:"
    echo "    - /tmp/gazebo_single.log"
    echo "    - /tmp/nav2_single.log"
    echo "    - /tmp/firebase_bridge.log"
else
    log_error "一部のプロセスが正常に起動していません"
    echo ""
    echo "🔧 トラブルシューティング:"
    echo ""
    echo "1. Firebase Bridge起動エラーの場合:"
    echo "   tail -100 /tmp/firebase_bridge.log"
    echo ""
    echo "2. 設定ファイル確認:"
    echo "   cat /workspace/config/rviz/firebase_config.yaml"
    echo ""
    echo "3. 手動起動テスト:"
    echo "   /workspace/install/ros2_firebase_bridge/lib/ros2_firebase_bridge/firebase_bridge \\"
    echo "     --ros-args -p robot_id:=robot_001 --log-level INFO"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# プロセスIDを保存
cat > /tmp/single_robot_pids.txt <<EOF
GAZEBO_PID=$GAZEBO_PID
NAV2_PID=$NAV2_PID
BRIDGE_PID=$BRIDGE_PID
EOF

log_info "プロセスIDを /tmp/single_robot_pids.txt に保存しました"

# 待機
if [ "$ALL_RUNNING" = true ]; then
    log_info "実行中... Ctrl+C で停止します"
    trap "echo ''; log_info 'プロセスを停止しています...'; kill $GAZEBO_PID $NAV2_PID $BRIDGE_PID 2>/dev/null; exit 0" INT

    read -p "Firebase Bridgeのログをリアルタイムで表示しますか? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        tail -f /tmp/firebase_bridge.log
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