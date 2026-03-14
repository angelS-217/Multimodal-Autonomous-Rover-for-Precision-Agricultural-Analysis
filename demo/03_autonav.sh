#!/bin/bash
# ==============================================================================
# Phase 3: Autonomous Patrol (Optimized for Senior Design)
# ==============================================================================

echo "🤖 Transitioning to Autonomous Navigation..."

# 1. Targeted Purge
echo "🧹 Sweeping previous nodes..."
pkill -9 -f "foxglove_bridge"
pkill -9 -f "slam_toolbox"
pkill -9 -f "ros2 run"
pkill -9 -f "ros2 launch"
pkill -9 -f "rover_core.py"
pkill -9 -f "rf2o_laser_odometry"
pkill -9 -f "ldlidar"
pkill -9 -f "garden_monitor.py"
rm -rf /dev/shm/fastrtps*

sleep 2 
echo "✅ Clean slate achieved."

# 2. Export Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✅ Eclipse Cyclone DDS Active."

# 3. Cleanup Trap (Targeted to prevent camera lock-up)
cleanup() {
    echo -e "\n🛑 Shutting down Navigation nodes..."
    pkill -9 -f "rover_core.py"
    pkill -9 -f "garden_monitor.py"
    pkill -9 -f "nav2"
    pkill -9 -f "ros2"
    exit 0
}
trap cleanup SIGINT

# 4. Launch Hardware
echo "➡️ Starting Rover Core (Motors & LoRa)..."
python3 ~/ros2_ws/demo/rover_core.py &

echo "➡️ Starting Lidar..."
ros2 launch ldlidar_stl_ros2 ld19.launch.py &

# Use the verified launch file instead of the brittle /tmp workaround
echo "➡️ Starting Laser Odometry..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

echo "➡️ Publishing Static Transforms..."
# Footprint to Link
ros2 run tf2_ros static_transform_publisher 0 0 0.045 0 0 0 base_footprint base_link --ros-args -p use_sim_time:=false &
# Link to Laser (Updated frame_id to base_laser to match ld19.launch.py)
ros2 run tf2_ros static_transform_publisher 0.06 0 0.095 0 0 0 base_link base_laser --ros-args -p use_sim_time:=false &

echo "➡️ Starting Foxglove WebSocket Bridge..."
ros2 launch /home/rover/ros2_ws/demo/foxglove_bridge_launch.xml &

# 5. Launch Nav2 Brain
echo "🧠 Starting Nav2 Brain..."
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/rover/ros2_ws/maps/field_map.yaml \
  use_sim_time:=False \
  params_file:=/home/rover/ros2_ws/demo/fat_to_thin_params.yaml > ~/ros2_ws/nav2_background.log 2>&1 &

# 6. Auto-Wakeup Nav2
echo "⏳ Waiting for Nav2 to initialize..."
sleep 15

echo "📍 Injecting Initial Pose..."
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

# 7. Start Mission Commander
echo "🚀 Launching Mission Commander..."
python3 ~/ros2_ws/demo/garden_monitor.py

wait