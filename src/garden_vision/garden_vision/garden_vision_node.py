#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import os

# Import local modules
from hardware_manager import HardwareInterface
from vision_processor import VisionProcessor

class GardenVisionNode(Node):
    def __init__(self):
        super().__init__('garden_vision_node')
        
        self.get_logger().info("Initializing vision system...")

        # --- 1. Init modules ---
        self.hw = HardwareInterface()
        self.vp = VisionProcessor()
        self.bridge = CvBridge()
        
        # Setup photo directory
        self.photo_dir = os.path.expanduser("~/ros2_ws/photos")
        os.makedirs(self.photo_dir, exist_ok=True)
        self.last_photo_time = 0
        
        # --- 2. Open camera ---
        self.cap = self.hw.set_camera()
        
        if not self.cap.isOpened():
            self.get_logger().error("Critical Error: Cannot open camera!")
        
        # --- 3. Control Parameters (Tuned for Stability) ---
        # Lower Kp = Slower, smoother movement
        # Previous was 0.04, now 0.02
        self.kp_pan = 0.02   
        self.kp_tilt = 0.02
        
        # Deadband: If error is less than this (pixels), DON'T move.
        # This prevents jittering.
        self.deadband = 40 
        
        # Initial angles
        self.current_pan = 90.0
        self.current_tilt = 90.0
        
        self.is_tracking = True
        
        # --- 4. ROS Communication ---
        self.pub_image = self.create_publisher(Image, 'vision/camera_feed', 10)
        self.pub_status = self.create_publisher(String, 'vision/status', 10)
        self.create_subscription(String, 'vision/cmd', self.cmd_callback, 10)

        # --- 5. Timer ---
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Vision node started (30Hz)")

    def cmd_callback(self, msg):
        cmd = msg.data
        if cmd == "START":
            self.is_tracking = True
        elif cmd == "STOP":
            self.is_tracking = False

    def timer_callback(self):
        # 1. Read frame
        ret, frame = self.cap.read()
        if not ret:
            return

        # Optional: frame = cv2.flip(frame, -1) 

        status_msg = "IDLE"
        final_display_frame = frame.copy() # Copy for display

        if self.is_tracking:
            processed_frame, target_pos, found = self.vp.process_frame(frame)
            final_display_frame = processed_frame
            
            if found:
                status_msg = "TRACKING"
                cx, cy = target_pos
                h, w, _ = frame.shape
                center_x, center_y = w // 2, h // 2
                
                # Calculate Error
                error_x = center_x - cx
                error_y = center_y - cy
                
                # --- Stability Logic: Deadband ---
                # Only move if error is BIGGER than deadband (40 pixels)
                if abs(error_x) > self.deadband:
                    self.current_pan += error_x * self.kp_pan
                    status_msg = "ADJUSTING"
                
                if abs(error_y) > self.deadband:
                    self.current_tilt += error_y * self.kp_tilt
                    status_msg = "ADJUSTING"
                
                # Clamp angles
                self.current_pan = max(0, min(180, self.current_pan))
                self.current_tilt = max(45, min(135, self.current_tilt))

                # Move Servo
                self.hw.move_servo(self.current_pan, self.current_tilt)
                
                # --- Photo Logic: Auto-Capture ---
                # If target is roughly centered (inside deadband)
                if abs(error_x) < self.deadband and abs(error_y) < self.deadband:
                    status_msg = "LOCKED"
                    current_time = time.time()
                    # Cooldown: Take photo only every 3 seconds
                    if current_time - self.last_photo_time > 3.0:
                        self.take_photo(frame)
                        self.last_photo_time = current_time
                        
            else:
                status_msg = "SEARCHING"
        
        # 3. Publish
        self.pub_status.publish(String(data=status_msg))
        try:
            # Add status text to image
            cv2.putText(final_display_frame, f"Mode: {status_msg}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ros_img = self.bridge.cv2_to_imgmsg(final_display_frame, "bgr8")
            self.pub_image.publish(ros_img)
        except Exception:
            pass

    def take_photo(self, frame):
        """Save the current frame to disk"""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"{self.photo_dir}/target_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        self.get_logger().info(f"📸 SNAPSHOT SAVED: {filename}")
        print(f"--- [Camera] Photo saved to {filename} ---")

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = GardenVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
