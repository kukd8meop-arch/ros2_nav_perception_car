# 功能：
#   ① 清理残留的 Gazebo / Nav2 / RViz2 进程
#   ② source ROS2 + 工作空间环境
#   ③ 启动仿真导航（两种模式）：
#        mode:=nav（默认）→ 预先加载地图 + AMCL 定位导航
#        mode:=slam        → 无需地图，雷达实时建图 + Nav2 规划
#   ④ 在 RViz2 里点击 "2D Nav Goal" 即可指定目标点
#
# 用法：
#   bash sim_nav.sh                               # 预建图导航，默认地图 map_01
#   bash sim_nav.sh map:=map_test                 # 切换地图
#   bash sim_nav.sh mode:=slam                    # 实时建图模式（不加载预建地图）
#   bash sim_nav.sh mode:=slam map:=map_test      # 建图模式 + 加载对应 Gazebo 世界
#   bash sim_nav.sh use_gui:=false                # 关闭 Gazebo 可视化（节省资源）
#   bash sim_nav.sh use_teb:=true                 # 使用 TEB 局部控制器
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
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "wait_for_gazebo" 2>/dev/null || true
# 等待进程真正退出，避免 launch 检测到残留进程死亡触发 shutdown
sleep 2
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

# ─── 步骤 4：启动仿真导航（独立终端模式） ──────────────────────────────────────
info "启动仿真闭环导航（独立终端模式）..."
info "  新终端 1：Gazebo + Nav2"
info "  新终端 2：RViz2（15s 后自动启动）"
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

# ── Gazebo + Nav2 启动脚本（写入临时文件，供新终端执行） ─────────────────────
# 解析 mode 参数，决定用哪个 launch 文件
LAUNCH_MODE="nav"  # 默认预建图导航
for arg in "$@"; do
    if [[ "$arg" == "mode:=slam" ]]; then
        LAUNCH_MODE="slam"
    fi
done

if [[ "$LAUNCH_MODE" == "slam" ]]; then
    LAUNCH_FILE="sim_slam_nav.launch.py"
    info "模式：实时 SLAM 建图 + Nav2 规划（无预建地图）"
else
    LAUNCH_FILE="sim_nav_full.launch.py"
    info "模式：预建图 AMCL 定位导航"
fi

# 过滤掉 mode:= 参数，避免传入 launch 文件（它不认识此参数）
LAUNCH_ARGS=()
for arg in "$@"; do
    [[ "$arg" == mode:=* ]] && continue
    LAUNCH_ARGS+=("$arg")
done

NAV_SCRIPT=$(mktemp /tmp/nav_launch_XXXXXX.sh)
cat > "$NAV_SCRIPT" << NAV_EOF
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ${WS_ROOT}/install/setup.bash
echo "[Nav] 启动 Gazebo + Nav2（若异常退出请查看上方日志）..."
ros2 launch navigation ${LAUNCH_FILE} launch_rviz:=false ${LAUNCH_ARGS[@]}
echo ""
echo "[Nav] launch 已退出（按 Enter 关闭此窗口）"
read
NAV_EOF
chmod +x "$NAV_SCRIPT"

# ── 在新终端窗口中分别启动 Gazebo+Nav2 和 RViz2 ────────────────────────────
if [[ -n "$TERM_CMD" ]]; then
    info "在新终端窗口中启动 Gazebo + Nav2..."
    info "在新终端窗口中启动 RViz2..."
    case "$TERM_CMD" in
        gnome-terminal)
            gnome-terminal --title="Gazebo + Nav2" -- bash "$NAV_SCRIPT" &
            gnome-terminal --title="RViz2 - 仿真导航" -- bash "$RVIZ_SCRIPT" &
            ;;
        *)
            $TERM_CMD -e "bash $NAV_SCRIPT" &
            $TERM_CMD -e "bash $RVIZ_SCRIPT" &
            ;;
    esac
else
    warn "未找到图形终端模拟器，Gazebo+Nav2 和 RViz2 将在后台启动"
    bash "$NAV_SCRIPT" &
    bash "$RVIZ_SCRIPT" &
fi

ok "已在独立终端窗口中启动仿真，当前终端可以关闭。"
