#!/bin/bash

# ==============================================================================
# Phase 1: Base Initialization & Manual Mapping Auto-Launcher
# Includes Foxglove Telemetry (Video Stream Disabled)
# ==============================================================================

echo "🚀 Starting Rover Mapping & Telemetry System..."

# 1. Activate the required Python virtual environment
source ~/envs/ros_env/bin/activate
echo "✅ Virtual environment activated."

# 2. Setup Cleanup Trap (Runs when you press Ctrl+C)
cleanup() {
    echo -e "\n🛑 Shutting down mapping nodes and cleaning up..."
    pkill -9 -f ros2
    pkill -9 -f python3
    rm -rf /dev/shm/fastrtps*
    echo "✅ Cleanup complete. Goodbye!"
    exit 0
}
trap cleanup SIGINT

# 3. Launch Foxglove Telemetry Bridge (Blacklisting video to save bandwidth)
echo "➡️ Starting Foxglove WebSocket Bridge (Video Blacklisted)..."
ros2 run foxglove_bridge foxglove_bridge --ros-args -p topic_blacklist:="['/image.*', '/camera.*', '/video.*']" &

# 4. Launch Motor Driver
echo "➡️ Starting Motor Driver..."
python3 ~/ros2_ws/motor_driver.py &

# 5. Launch Lidar & Odometry
echo "➡️ Starting Lidar & Odometry..."
ros2 launch ldlidar_stl_ros2 ld19.launch.py &
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

# 6. Launch TF Tree (Static Transforms)
echo "➡️ Publishing Static Transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint --ros-args -p use_sim_time:=false &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link --ros-args -p use_sim_time:=false &
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link base_laser --ros-args -p use_sim_time:=false &

# 7. Launch SLAM Toolbox
echo "➡️ Starting SLAM Toolbox (Mapping)..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false -p odom_frame:=odom -p base_frame:=base_footprint -p map_frame:=map &

echo "=============================================================================="
echo "✅ All Phase 1 systems (including Telemetry) are running!"
echo "📡 Foxglove is ready. Connect your laptop to ws://<PI_IP_ADDRESS>:8765"
echo "🎮 ACTION REQUIRED: Use the gamepad to drive the rover around the test environment."
echo "🛑 Press [Ctrl+C] at any time to stop mapping."
echo "=============================================================================="

# Wait indefinitely until the user presses Ctrl+C
wait