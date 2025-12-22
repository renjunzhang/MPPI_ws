#!/bin/bash

# ========================================================
# mpc_track_fastplanner 实物平台快速启动脚本
# 作者：自动生成
# 日期：2025-11-14
# ========================================================

echo "========================================================="
echo "  mpc_track_fastplanner for hf_platform"
echo "  FastPlanner + MPC 轨迹跟踪系统"
echo "========================================================="
echo ""

# 检查是否已编译
if [ ! -f "/home/a/hf_move/devel/lib/mpc_tracking/mpc_tracking_node" ]; then
    echo "❌ 错误：mpc_tracking 未编译"
    echo "请先运行："
    echo "  cd /home/a/hf_move"
    echo "  catkin_make --pkg mpc_tracking lidar2world"
    exit 1
fi

# 检查 junjun_bringup 是否在运行
if ! rostopic list | grep -q "/odom"; then
    echo "⚠️  警告：未检测到 /odom 话题"
    echo "请先在另一个终端启动 hf_platform："
    echo "  roslaunch junjun_bringup junjun_bringup.launch"
    echo ""
    read -p "是否已启动 junjun_bringup？(y/N): " confirm
    if [[ ! $confirm =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

echo ""
echo "✅ 环境检查通过，正在启动系统..."
echo ""
echo "📌 提示："
echo "  1. 系统启动后在 RViz 中使用 '2D Nav Goal' 设置目标点"
echo "  2. FastPlanner 会自动规划避障轨迹"
echo "  3. MPC 控制器会跟踪生成的轨迹"
echo "  4. 速度命令发送到: /hf_platform/twist_mux/cmd_vel"
echo ""
echo "========================================================="
sleep 2

# 启动系统
roslaunch mpc_track_fastplanner hf_platform.launch
