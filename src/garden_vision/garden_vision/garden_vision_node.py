#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time

# Import local modules
from hardware_manager import HardwareInterface
from vision_processor import VisionProcessor

class GardenVisionNode(Node):
    def __init__(self):
        super().__init__('garden_vision_node')
        self.get_logger().info("Initializing vision tracking system...")

        self.hw = HardwareInterface()
        self.vp = VisionProcessor()
        self.bridge = CvBridge()
        
        self.cap = self.hw.set_camera()
        if not self.cap.isOpened():
            self.get_logger().error("Critical Error: Cannot open camera!")
        
        # --- Tracking Parameters ---
        self.kp_pan = 0.02   
        self.kp_tilt = 0.02
        self.deadband = 40 
        self.current_pan = 90.0
        self.current_tilt = 90.0
        
        # --- Scanning Parameters ---
        self.scan_pan_dir = 1.0   
        self.scan_tilt_dir = 1.0  
        self.scan_speed = 1.5     
        self.tilt_step = 15.0     

        self.get_logger().info("Waking up servos and centering...")
        self.hw.move_servo(self.current_pan, self.current_tilt)
        time.sleep(0.5) 
        
        # Start idle until commanded by the monitor
        self.is_tracking = False
        
        # --- ROS Communication ---
        self.pub_image = self.create_publisher(Image, 'vision/camera_feed', 10)
        self.pub_status = self.create_publisher(String, 'vision/status', 10)
        self.create_subscription(String, 'vision/cmd', self.cmd_callback, 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

    def cmd_callback(self, msg):
        if msg.data == "START":
            self.is_tracking = True
            self.get_logger().info("Received START: Hunting for flag...")
        elif msg.data == "STOP":
            self.is_tracking = False
            self.get_logger().info("Received STOP: Going idle.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        status_msg = "IDLE"
        final_display_frame = frame.copy() 

        if self.is_tracking:
            processed_frame, target_pos, found = self.vp.process_frame(frame)
            final_display_frame = processed_frame
            
            if found:
                cx, cy = target_pos
                h, w, _ = frame.shape
                center_x, center_y = w // 2, h // 2
                
                # The 150px offset to aim below the flag
                y_offset_pixels = 150 
                
                error_x = center_x - cx
                error_y = center_y - (cy + y_offset_pixels)
                
                needs_movement = False
                status_msg = "ADJUSTING"
                
                if abs(error_x) > self.deadband:
                    self.current_pan += error_x * self.kp_pan
                    needs_movement = True
                
                if abs(error_y) > self.deadband:
                    self.current_tilt += error_y * self.kp_tilt
                    needs_movement = True
                
                self.current_pan = max(0.0, min(180.0, self.current_pan))
                self.current_tilt = max(45.0, min(135.0, self.current_tilt))

                if needs_movement:
                    self.hw.move_servo(self.current_pan, self.current_tilt)
                
                if not needs_movement:
                    status_msg = "LOCKED"
                        
            else:
                status_msg = "SEARCHING"
                # Raster Scan Logic
                self.current_pan += (self.scan_speed * self.scan_pan_dir)
                if self.current_pan >= 180.0 or self.current_pan <= 0.0:
                    self.scan_pan_dir *= -1.0  
                    self.current_tilt += (self.tilt_step * self.scan_tilt_dir)
                    if self.current_tilt >= 135.0 or self.current_tilt <= 45.0:
                        self.scan_tilt_dir *= -1.0  
                
                self.current_pan = max(0.0, min(180.0, self.current_pan))
                self.current_tilt = max(45.0, min(135.0, self.current_tilt))
                self.hw.move_servo(self.current_pan, self.current_tilt)
        
        self.pub_status.publish(String(data=status_msg))
        try:
            cv2.putText(final_display_frame, f"Mode: {status_msg}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ros_img = self.bridge.cv2_to_imgmsg(final_display_frame, "bgr8")
            self.pub_image.publish(ros_img)
        except Exception:
            pass

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