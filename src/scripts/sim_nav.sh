#!/usr/bin/env bash
# =============================================================================
# sim_nav.sh — 一键启动仿真闭环导航
# =============================================================================
# 功能：
#   ① 清理残留的 Gazebo / Nav2 / RViz2 进程
#   ② source ROS2 + 工作空间环境
#   ③ 启动 sim_nav_full.launch.py：
#        Gazebo（含机器人模型）→ Nav2（AMCL + 规划 + 控制）→ RViz2
#   ④ 在 RViz2 里点击 "Nav2 Goal" 即可指定目标点，
#      Nav2 规划路径并通过 cmd_vel_relay 把速度指令写入
#      Gazebo 的 /controller/cmd_vel，小车在 Gazebo 里同步移动。
#
# 用法：
#   bash sim_nav.sh                          # 默认地图 map_01，有 Gazebo GUI
#   bash sim_nav.sh map:=map_02              # 切换地图
#   bash sim_nav.sh use_gui:=false           # 关闭 Gazebo 可视化（节省资源）
#   bash sim_nav.sh use_teb:=true            # 使用 TEB 局部控制器
#   bash sim_nav.sh map:=map_02 use_gui:=false use_teb:=true
#
# 依赖：
#   - ROS2 Humble（/opt/ros/humble/setup.bash 必须存在）
#   - 已完成 colcon build（install/ 目录存在）
#   - topic_tools（ros2 pkg list 里有 topic_tools）
# =============================================================================

set -e  # 任何命令失败即退出

# ─── 颜色输出 ────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'  # No Color

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ─── 确定工作空间根目录 ──────────────────────────────────────────────────────
# 脚本可能放在项目内任意子目录（如 src/simulations/scripts/），
# 向上查找包含 install/setup.bash 的目录作为工作空间根。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR"
while [[ "$WS_ROOT" != "/" ]]; do
    [[ -f "$WS_ROOT/install/setup.bash" ]] && break
    WS_ROOT="$(dirname "$WS_ROOT")"
done
if [[ "$WS_ROOT" == "/" ]]; then
    error "无法自动定位工作空间根目录（未找到 install/setup.bash）"
    error "请确认已执行: colcon build --symlink-install"
    exit 1
fi
info "工作空间根目录: $WS_ROOT"

# ─── 步骤 1：清理残留进程 ────────────────────────────────────────────────────
info "清理残留仿真进程..."
killall -9 gzserver gzclient 2>/dev/null || true
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "ruby.*ign.*gazebo" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "relay" 2>/dev/null || true
pkill -9 -f "odom_to_tf" 2>/dev/null || true
pkill -9 -f "scan_frame_fix" 2>/dev/null || true
sleep 1
ok "残留进程清理完毕"

# ─── 步骤 2：source 环境 ─────────────────────────────────────────────────────
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_ROOT/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    error "未找到 ROS2 Humble 环境: $ROS_SETUP"
    exit 1
fi
if [[ ! -f "$WS_SETUP" ]]; then
    error "未找到工作空间安装目录: $WS_SETUP"
    error "请先执行: cd $WS_ROOT && colcon build --symlink-install"
    exit 1
fi

# shellcheck source=/dev/null
source "$ROS_SETUP"
# shellcheck source=/dev/null
source "$WS_SETUP"
ok "ROS2 + 工作空间环境已加载"

# ─── 步骤 3：检查必要包是否存在 ─────────────────────────────────────────────
info "检查必要 ROS2 包..."
for PKG in navigation rosorin_description slam topic_tools; do
    if ! ros2 pkg list 2>/dev/null | grep -q "^${PKG}$"; then
        error "缺少 ROS2 包: ${PKG}，请确认已编译并 source。"
        exit 1
    fi
done
ok "所有必要包均已就绪"

# ─── 步骤 4：启动仿真导航（双终端模式） ──────────────────────────────────────
info "启动仿真闭环导航（双终端模式）..."
info "  终端 1（当前）：Gazebo + Nav2"
info "  终端 2（新窗口）：RViz2"
info "  在 RViz2 点击 '2D Nav Goal' 即可让小车运动"
echo ""

# ── 检测可用的终端模拟器 ─────────────────────────────────────────────────────
TERM_CMD=""
if command -v gnome-terminal &>/dev/null; then
    TERM_CMD="gnome-terminal"
elif command -v x-terminal-emulator &>/dev/null; then
    TERM_CMD="x-terminal-emulator"
elif command -v xterm &>/dev/null; then
    TERM_CMD="xterm"
fi

# ── RViz2 启动脚本（写入临时文件，供新终端执行） ─────────────────────────────
NAV_PKG_SHARE="$(ros2 pkg prefix navigation 2>/dev/null)/share/navigation"
RVIZ_CONFIG="${NAV_PKG_SHARE}/rviz/navigation.rviz"

RVIZ_SCRIPT=$(mktemp /tmp/rviz2_launch_XXXXXX.sh)
cat > "$RVIZ_SCRIPT" << RVIZ_EOF
#!/usr/bin/env bash
# RViz2 独立终端启动脚本（自动生成，用完即删）
source /opt/ros/humble/setup.bash
source ${WS_ROOT}/install/setup.bash

echo "[RViz2] 等待 Nav2 就绪（15 秒后启动）..."
sleep 15

echo "[RViz2] 启动 RViz2..."
echo "[RViz2] 在工具栏点击 '2D Nav Goal' → 在地图上点击拖拽 → 小车自动导航"
ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true
RVIZ_EOF
chmod +x "$RVIZ_SCRIPT"

# ── 在新终端窗口中启动 RViz2 ─────────────────────────────────────────────────
if [[ -n "$TERM_CMD" ]]; then
    info "在新终端窗口中启动 RViz2..."
    case "$TERM_CMD" in
        gnome-terminal)
            gnome-terminal --title="RViz2 - 仿真导航" -- bash "$RVIZ_SCRIPT" &
            ;;
        *)
            $TERM_CMD -e "bash $RVIZ_SCRIPT" &
            ;;
    esac
else
    warn "未找到图形终端模拟器，RViz2 将在后台启动"
    bash "$RVIZ_SCRIPT" &
fi

# ── 当前终端启动 Gazebo + Nav2（不含 RViz2） ────────────────────────────────
# launch_rviz:=false 告诉 launch 文件不要内部启动 RViz2
exec ros2 launch navigation sim_nav_full.launch.py launch_rviz:=false "$@"
