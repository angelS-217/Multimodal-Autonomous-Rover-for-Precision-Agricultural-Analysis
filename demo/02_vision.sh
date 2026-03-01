#!/bin/bash
# ==============================================================================
# Phase 2: Target Detection (Vision) Auto-Launcher
# Execute after manual mapping is completed.
# ==============================================================================

echo "📸 Starting Vision System..."

# 1. Activate the required Python virtual environment
source ~/envs/ros_env/bin/activate
echo "✅ Virtual environment activated."

# Setup Cleanup Trap so Ctrl+C closes the camera safely
cleanup() {
    echo -e "\n🛑 Shutting down camera node..."
    pkill -9 -f garden_vision_node.py
    echo "✅ Camera stopped."
    exit 0
}
trap cleanup SIGINT

# 2. Navigate to the code directory 
# (Crucial so garden_vision_node.py can find its local helper files/processors)
cd /home/rover/ros2_ws/src/garden_vision/garden_vision/ || { echo "❌ Directory not found!"; exit 1; }

# 3. Check for the node and launch it
if [ -f "garden_vision_node.py" ]; then
    echo "➡️ Publishing video feed..."
    python3 garden_vision_node.py &
else
    echo "❌ ERROR: Cannot find garden_vision_node.py! Check the path."
    exit 1
fi

echo "=============================================================================="
echo "✅ Phase 2 Running. Camera is active and ready to run auto_cruise.py."
echo "=============================================================================="

# Keep the script running in the foreground so the trap works
wait