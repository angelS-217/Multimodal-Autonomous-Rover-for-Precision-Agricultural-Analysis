#!/bin/bash
# ==============================================================================
# Phase 5: Image Download Web Server
# Run this after your patrol to download your PlantCV images and CSV logs.
# ==============================================================================

echo "🌐 Starting Local Web Server for Image Downloads..."

# 1. Navigate to the photos directory
cd /home/rover/ros2_ws/photos || { echo "❌ Photos directory not found! Have you taken any photos yet?"; exit 1; }

echo "📂 Hosting files from: $(pwd)"
echo "💻 Please open a web browser on your PC and go to http://<PI_IP_ADDRESS>:8000"
echo "🛑 Press [Ctrl+C] in this terminal to safely stop the server when you are done."
echo "=============================================================================="

# 2. Start the Python HTTP server on port 8000
python3 -m http.server 8000