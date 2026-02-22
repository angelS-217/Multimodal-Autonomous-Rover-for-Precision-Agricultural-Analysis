#!/bin/bash

echo "👁️ 正在启动 Garden Vision 视觉识别系统..."

# 1. 切换到代码所在的文件夹 (这一步很关键，不然它找不到旁边的 processor)
cd /home/rover/ros2_ws/src/garden_vision/garden_vision/

# 2. 检查文件是否存在
if [ -f "garden_vision_node.py" ]; then
    # 3. 运行它！
    python3 garden_vision_node.py
else
    echo "❌ 坏了，找不到 garden_vision_node.py！请检查路径。"
    exit 1
fi
