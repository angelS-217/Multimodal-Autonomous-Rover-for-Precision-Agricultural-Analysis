#!/bin/bash
# ==============================================================================
# Phase 3: Autonomous Navigation & Agricultural Patrol
# ==============================================================================

echo "🤖 Starting Autonomous Navigation & Plant Analysis System..."

# 1. Activate the required Python virtual environment
source ~/envs/ros_env/bin/activate
echo "✅ Virtual environment activated."

# 2. Setup Cleanup Trap (Runs when you press Ctrl+C)
cleanup() {
    echo -e "\n🛑 Shutting down Nav2 and cleaning up..."
    pkill -9 -f ros2
    pkill -9 -f python3
    rm -rf /dev/shm/fastrtps*
    echo "✅ Cleanup complete. Rover stopped!"
    exit 0
}
trap cleanup SIGINT

# 3. Launch Nav2 with custom parameters (Runs in background, logs hidden)
echo "🧠 Starting Nav2 Brain..."
echo "📂 Nav2 logs are being saved to ~/ros2_ws/nav2_background.log"
# NOTE: Path updated to the new 'demo' folder
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/rover/ros2_ws/demo/fat_to_thin_params.yaml > ~/ros2_ws/nav2_background.log 2>&1 &

# 4. Wait for Nav2 to initialize 
echo "⏳ Waiting 15 seconds for Nav2 to fully boot up..."
sleep 15
echo "✅ Nav2 is ready!"

# 5. Launch Auto Cruise (Runs in foreground so you can see its output)
echo "🚀 Starting Agricultural Auto Cruise Script..."
# NOTE: Path updated to the new 'demo' folder
python3 ~/ros2_ws/demo/auto_cruise.py

# Keep script running
wait