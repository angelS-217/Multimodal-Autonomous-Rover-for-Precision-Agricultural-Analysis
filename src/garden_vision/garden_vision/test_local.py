# test_local.py
# 这是一个 "假" 的主程序，专门用来在没有 ROS 的 Windows/Mac 电脑上跑
# 它可以帮你测试：
# 1. 摄像头能不能打开
# 2. 能不能识别到红色
# 3. 舵机控制逻辑算得对不对

import cv2
import time
import numpy as np

# 导入你写的模块
# 注意：确保 test_local.py 和这两个文件在同一个文件夹里
from hardware_manager import HardwareInterface
from vision_processor import VisionProcessor

def main():
    print("--- 启动本地测试模式 (无 ROS) ---")
    print("按 'q' 退出测试")
    
    # 1. 初始化虚拟硬件和视觉处理器
    hw = HardwareInterface()
    vp = VisionProcessor()
    
    # 打开电脑摄像头
    cap = hw.set_camera()
    
    if not cap.isOpened():
        print("错误：无法打开摄像头！")
        return

    # 模拟舵机当前角度
    current_pan = 90.0
    current_tilt = 90.0
    
    # PID 参数 (在电脑上调好大概的感觉)
    kp_pan = 0.05
    kp_tilt = 0.05

    while True:
        ret, frame = cap.read()
        if not ret: 
            print("无法读取画面")
            break
        
        # 镜像翻转 (因为是自拍模式调试，翻转后左手就是左边，方便直觉判断)
        frame = cv2.flip(frame, 1)

        # --- 核心逻辑开始 ---
        
        # 2. 调用你的视觉算法
        # processed_frame: 画了框的图
        # target_pos: (x, y) 目标中心坐标
        # found: 是否找到目标 (True/False)
        processed_frame, target_pos, found = vp.process_frame(frame)

        if found:
            cx, cy = target_pos
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            
            # 3. 计算误差
            err_x = center_x - cx
            err_y = center_y - cy

            # 4. 模拟 PID 控制
            # 如果误差大于 10 个像素，就调整角度
            if abs(err_x) > 10: 
                current_pan += err_x * kp_pan
            
            if abs(err_y) > 10: 
                current_tilt += err_y * kp_tilt
            
            # 5. 假装驱动硬件 (HardwareInterface 会检测到不是树莓派，只打印不报错)
            hw.move_servo(current_pan, current_tilt)
            
            # 6. 在终端打印调试信息 (模拟 ROS Logger)
            # \r 可以让打印在同一行刷新，不会刷屏
            print(f"\r[追踪中] 误差X:{err_x:3d} | Pan角度:{current_pan:.1f}° | Tilt角度:{current_tilt:.1f}°", end="")
            
            # 在画面上显示一行字
            cv2.putText(processed_frame, f"ERR_X: {err_x}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        else:
            print(f"\r[搜索中] 未检测到红色目标...", end="")

        # --- 核心逻辑结束 ---

        # 显示画面
        cv2.imshow("Local Test (Press q to exit)", processed_frame)
        
        # 按 'q' 退出
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("\n测试结束")

if __name__ == "__main__":
    main()