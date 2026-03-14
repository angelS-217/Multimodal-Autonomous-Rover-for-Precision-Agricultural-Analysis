#!/bin/bash
# ==============================================================================
# Phase 1: Base Initialization & Manual Mapping
# ==============================================================================

echo "🚀 Starting Rover Mapping & Telemetry System..."

# 1. Export Cyclone Data Distribution Service (DDS)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✅ Eclipse Cyclone DDS Active."

# 2. Configure cleanup for ctrl+c command
cleanup() {
    echo -e "\n🛑 Shutting down mapping nodes and cleaning up..."
    # Kill the bridge specifically, then everything else
    pkill -9 -f foxglove_bridge
    pkill -9 -f ros2
    pkill -9 -f python3
    rm -rf /dev/shm/fastrtps*
    echo "✅ Cleanup complete. Goodbye."
    exit 0
}
trap cleanup SIGINT

# 3. Launch Foxglove Telemetry Bridge (Updated for proper port binding)
echo "➡️ Starting Foxglove WebSocket Bridge..."
ros2 launch /home/rover/ros2_ws/demo/foxglove_bridge_launch.xml &

# 4. Launch VK-172 GPS Driver (Update /dev/ttyACM0 if your Pi mounted it differently)
echo "➡️ Starting USB GPS Driver..."
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyACM0 -p baud:=9600 &

# 5. Launch the Hardware Gateway (Formerly motor_driver.py)
echo "➡️ Starting Rover Core (Motors & LoRa)..."
python3 ~/ros2_ws/demo/rover_core.py &

# 6. Launch Lidar & Odometry
echo "➡️ Starting Lidar & Odometry..."
ros2 launch ldlidar_stl_ros2 ld19.launch.py &
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

# 7. Launch TF Tree (Static Transforms)
echo "➡️ Publishing Static Transforms..."
# Transform 1: Ground to Robot Base (Z = +4.5cm)
ros2 run tf2_ros static_transform_publisher 0 0 0.045 0 0 0 base_footprint base_link --ros-args -p use_sim_time:=false &
# Transform 2: Robot Base to Lidar (X = +6.0cm forward, Z = +9.5cm up)
ros2 run tf2_ros static_transform_publisher 0.06 0 0.095 0 0 0 base_link base_laser --ros-args -p use_sim_time:=false &

# 8. Launch SLAM Toolbox (Using Custom YAML)
echo "➡️ Starting SLAM Toolbox (Mapping)..."
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/home/rover/ros2_ws/demo/map_params.yaml &

echo "=============================================================================="
echo "✅ Phase 1 Running. Connect to Foxglove and drive manually to build map."
echo "🛑 Press [Ctrl+C] to end operation."
echo "=============================================================================="
wait