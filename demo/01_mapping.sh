#!/bin/bash
# ==============================================================================
# Phase 1: Base Initialization & Manual Mapping
# ==============================================================================

echo "🚀 Starting Rover Mapping & Telemetry System..."

# 1. Activate Python environment
source ~/envs/ros_env/bin/activate

# 2. Configure cleanup for ctrl+c command
cleanup() {
    echo -e "\n🛑 Shutting down mapping nodes and cleaning up..."
    pkill -9 -f ros2
    pkill -9 -f python3
    rm -rf /dev/shm/fastrtps*
    echo "✅ Cleanup complete. Goodbye!"
    exit 0
}
trap cleanup SIGINT

# 3. Launch Foxglove Telemetry Bridge (Video Blacklisted to save bandwidth)
echo "➡️ Starting Foxglove WebSocket Bridge..."
ros2 run foxglove_bridge foxglove_bridge --ros-args -p topic_blacklist:="['/image.*', '/camera.*', '/video.*']" &

# 4. Launch VK-172 GPS Driver (Update /dev/ttyACM0 if your Pi mounted it differently)
echo "➡️ Starting USB GPS Driver..."
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyACM0 -p baud:=9600 &

# 5. Launch the Hardware Gateway (Formerly motor_driver.py)
echo "➡️ Starting Rover Core (Motors & LoRa)..."
python3 ~/ros2_ws/rover_core.py &

# 6. Launch Lidar & Odometry
echo "➡️ Starting Lidar & Odometry..."
ros2 launch ldlidar_stl_ros2 ld19.launch.py &
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

# 7. Launch TF Tree (Static Transforms)
echo "➡️ Publishing Static Transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint --ros-args -p use_sim_time:=false &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link --ros-args -p use_sim_time:=false &
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link base_laser --ros-args -p use_sim_time:=false &

# 8. Launch SLAM Toolbox
echo "➡️ Starting SLAM Toolbox (Mapping)..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false -p odom_frame:=odom -p base_frame:=base_footprint -p map_frame:=map &

echo "=============================================================================="
echo "✅ Phase 1 Running. Connect to Foxglove and drive manually to build map."
echo "🛑 Press [Ctrl+C] to end operation."
echo "=============================================================================="
wait