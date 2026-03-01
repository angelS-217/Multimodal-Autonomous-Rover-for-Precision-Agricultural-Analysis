#!/bin/bash
echo "🏗️ 正在强制构建机器人坐标系骨架..."

# A. 核心骨架：base_footprint (投影中心) -> base_link (车体中心)
#ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" base_footprint base_link &
#sleep 1

# B. 感知器官：base_link -> base_laser (雷达位置)
# 根据你之前 STL-19P 的日志，雷达 frame 叫 base_laser
#ros2 run tf2_ros static_transform_publisher "0" "0" "0.18" "0" "0" "0" base_link base_laser &
#sleep 1

echo "👀 启动雷达与里程计..."
# 启动雷达（使用我们改好的包名和 launch）
ros2 launch ldlidar_stl_ros2 ld19.launch.py &
sleep 2

# 启动里程计 (它应该会发布 odom -> base_footprint)
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

echo "✅ 坐标系桥梁已架设，等待 Nav2 接管..."
wait
