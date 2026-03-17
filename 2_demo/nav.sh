#!/bin/bash
echo "正在启动 Nav2 导航系统..."
echo "加载地图: /home/rover/ros2_ws/final_map.yaml"


ros2 launch nav2_bringup bringup_launch.py \
    map:=/home/rover/ros2_ws/final_map.yaml \
    use_sim_time:=false \
    autostart:=true \
    initial_x:=0.0 \
    initial_y:=0.0 \
    initial_yaw:=0.0
    