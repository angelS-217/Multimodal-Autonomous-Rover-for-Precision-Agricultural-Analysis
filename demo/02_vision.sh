#!/bin/bash
# ==============================================================================
# Phase 2: Target Detection (Vision) Auto-Launcher
# Execute after manual mapping is completed.
# ==============================================================================

echo "📸 Starting Vision System..."

# 1. Export Cyclone Data Distribution Service (DDS)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✅ Eclipse Cyclone DDS Active."

# Setup Cleanup Trap so Ctrl+C closes everything safely
cleanup() {
    echo -e "\n🛑 Shutting down camera node..."
    pkill -9 -f garden_vision_node.py
    pkill -9 rpicam-vid
    echo "✅ Processes stopped."
    exit 0
}
trap cleanup SIGINT

# 2. Navigate to the code directory 
cd /home/rover/ros2_ws/src/garden_vision/garden_vision/ || { echo "❌ Directory not found!"; exit 1; }

# 3. Launch the Gimbal Controller Node
if [ -f "garden_vision_node.py" ]; then
    echo "➡️ Publishing video feed and awaiting commands..."
    python3 garden_vision_node.py &
else
    echo "❌ ERROR: Cannot find garden_vision_node.py! Check the path."
    exit 1
fi

echo "=============================================================================="
echo "✅ Phase 2 Running. Camera is active and waiting for Mission Commander."
echo "=============================================================================="

wait