#!/bin/bash

# ========================================================
# mpc_track_fastplanner 启动前系统检查（Go/No-Go）
# 基于 GPT 建议的检查清单
# ========================================================

echo "========================================================="
echo "  mpc_track_fastplanner 系统检查"
echo "  Go/No-Go Checklist"
echo "========================================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

# 检查函数
check_pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((PASS_COUNT++))
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ((FAIL_COUNT++))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    ((WARN_COUNT++))
}

# ========== 1. 检查点云话题 ==========
echo "1. 检查点云话题 /merged_cloud"
if rostopic hz /merged_cloud -c 1 &>/dev/null; then
    FRAME_ID=$(rostopic echo /merged_cloud -n 1 2>/dev/null | grep "frame_id:" | head -1 | awk '{print $2}' | tr -d '"')
    check_pass "点云话题正常，frame_id: $FRAME_ID"
else
    check_fail "点云话题 /merged_cloud 不存在或无数据"
fi
echo ""

# ========== 2. 检查里程计话题 ==========
echo "2. 检查里程计话题 /odom"
if rostopic hz /odom -c 1 &>/dev/null; then
    check_pass "里程计话题正常"
else
    check_fail "里程计话题 /odom 不存在或无数据"
fi
echo ""

# ========== 3. 检查 TF 变换 ==========
echo "3. 检查 TF 变换"
if rosrun tf tf_echo odom hf_base_link 2>/dev/null | grep -q "At time"; then
    check_pass "TF 变换 odom → hf_base_link 存在"
else
    check_fail "TF 变换 odom → hf_base_link 不存在"
fi

if rostopic echo /merged_cloud -n 1 2>/dev/null | grep -q "frame_id"; then
    CLOUD_FRAME=$(rostopic echo /merged_cloud -n 1 2>/dev/null | grep "frame_id:" | head -1 | awk '{print $2}' | tr -d '"')
    if rosrun tf tf_echo odom $CLOUD_FRAME 2>/dev/null | grep -q "At time"; then
        check_pass "TF 变换 odom → $CLOUD_FRAME 存在"
    else
        check_warn "TF 变换 odom → $CLOUD_FRAME 不存在，点云转换可能失败"
    fi
fi
echo ""

# ========== 4. 检查 use_sim_time ==========
echo "4. 检查时钟设置"
USE_SIM_TIME=$(rosparam get /use_sim_time 2>/dev/null)
if [ "$USE_SIM_TIME" = "false" ]; then
    check_pass "use_sim_time=false（实机正确配置）"
elif [ "$USE_SIM_TIME" = "true" ]; then
    check_warn "use_sim_time=true（仿真模式，实机运行请设为 false）"
else
    check_warn "use_sim_time 未设置（将在 launch 中设为 false）"
fi
echo ""

# ========== 5. 检查 twist_mux 状态 ==========
echo "5. 检查 twist_mux 状态"
if rostopic list | grep -q "/hf_platform/twist_mux/cmd_vel"; then
    check_pass "twist_mux 话题存在"
    
    # 检查是否有其他控制源
    if rostopic echo /hf_platform/twist_mux/selected -n 1 2>/dev/null | grep -q "data:"; then
        SELECTED=$(rostopic echo /hf_platform/twist_mux/selected -n 1 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d '"')
        check_warn "当前 twist_mux 选择: $SELECTED（MPC 启动后应切换到 MPC 通道）"
    fi
else
    check_fail "twist_mux 话题不存在，请检查 junjun_bringup 是否正常启动"
fi
echo ""

# ========== 6. 检查编译状态 ==========
echo "6. 检查编译状态"
if [ -f "/home/a/hf_move/devel/lib/mpc_tracking/mpc_tracking_node" ]; then
    check_pass "mpc_tracking_node 已编译"
else
    check_fail "mpc_tracking_node 未编译"
fi

if [ -f "/home/a/hf_move/devel/lib/lidar2world/lidar2world_node" ]; then
    check_pass "lidar2world_node 已编译"
else
    check_fail "lidar2world_node 未编译"
fi
echo ""

# ========== 总结 ==========
echo "========================================================="
echo "检查结果："
echo -e "  ${GREEN}通过: $PASS_COUNT${NC}"
echo -e "  ${YELLOW}警告: $WARN_COUNT${NC}"
echo -e "  ${RED}失败: $FAIL_COUNT${NC}"
echo "========================================================="
echo ""

if [ $FAIL_COUNT -gt 0 ]; then
    echo -e "${RED}❌ 系统检查未通过，请修复上述问题后再启动${NC}"
    exit 1
elif [ $WARN_COUNT -gt 0 ]; then
    echo -e "${YELLOW}⚠️  系统检查通过但有警告，建议检查后继续${NC}"
    echo ""
    read -p "是否继续启动？(y/N): " confirm
    if [[ ! $confirm =~ ^[Yy]$ ]]; then
        exit 0
    fi
else
    echo -e "${GREEN}✅ 系统检查全部通过！${NC}"
fi

echo ""
echo "========================================================="
echo "  启动选项："
echo "  1. 正常模式（控制底盘）"
echo "  2. 调试模式（不控制底盘）"
echo "  3. 仅检查，不启动"
echo "========================================================="
read -p "请选择 (1/2/3): " choice

case $choice in
    1)
        echo ""
        echo "🚀 启动正常模式..."
        sleep 1
        roslaunch mpc_track_fastplanner hf_platform.launch
        ;;
    2)
        echo ""
        echo "🐛 启动调试模式..."
        sleep 1
        roslaunch mpc_track_fastplanner hf_platform_debug.launch
        ;;
    3)
        echo ""
        echo "✓ 检查完成，退出"
        exit 0
        ;;
    *)
        echo "无效选择，退出"
        exit 1
        ;;
esac
