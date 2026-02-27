#!/bin/bash

# ==============================================================================
# Phase 2: Target Detection (Vision) Auto-Launcher
# Execute after manual mapping is completed.
# ==============================================================================

echo "📷 Starting Vision System..."

# 1. Activate the required Python virtual environment
source ~/envs/ros_env/bin/activate
echo "✅ Virtual environment activated."

# 2. Launch the existing vision script
bash ~/ros2_ws/src/garden_vision/start_vision.sh