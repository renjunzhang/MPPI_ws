
# 启动仿真环境并遥控 Ridgeback

```bash
source /data/a/MPPI_ws/devel/setup.bash      # 按你自己的工作空间改
# 第一步
roslaunch ridgeback_gazebo ridgeback_world.launch gui:=false headless:=true

# 第二步
gzclient
# 或 rosrun gazebo_ros gzclient

# 第三步
roslaunch ridgeback_navigation ridgeback_description.launch

# 第四步 启动遥控节点
roslaunch ridgeback_navigation teleop.launch

```