#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import os

# Import local modules
from hardware_manager import HardwareInterface

class PlantDemoNode(Node):
    def __init__(self):
        super().__init__('plant_demo_node')
        self.get_logger().info("🌱 Initializing Plant Demo System...")

        self.hw = HardwareInterface()
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
        
        # State control
        self.is_active = True        
        self.has_analyzed = False    
        
        # --- ROS Communication ---
        self.pub_image = self.create_publisher(Image, 'vision/demo_feed', 10)
        self.pub_status = self.create_publisher(String, 'vision/status', 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        status_msg = "IDLE"
        final_display_frame = frame.copy() 

        if self.is_active and not self.has_analyzed:
            # --- GREEN PLANT DETECTION ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # HSV bounds for green leaves
            lower_green = np.array([130, 65, 50])
            upper_green = np.array([125, 75, 80])
            
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # Clean up noise
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            found = False
            cx, cy = 0, 0
            
            if contours:
                # Find the largest green blob
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Minimum area threshold to ignore tiny weeds/noise
                if cv2.contourArea(largest_contour) > 1000: 
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        found = True
                        
                        # Visual feedback: Draw contour and center point
                        cv2.drawContours(final_display_frame, [largest_contour], -1, (0, 255, 0), 2)
                        cv2.circle(final_display_frame, (cx, cy), 5, (0, 0, 255), -1)
            
            # --- TRACKING & GIMBAL CONTROL ---
            if found:
                h, w, _ = frame.shape
                center_x, center_y = w // 2, h // 2
                
                error_x = center_x - cx
                error_y = center_y - cy
                
                needs_movement = False
                
                if abs(error_x) > self.deadband:
                    self.current_pan += error_x * self.kp_pan
                    needs_movement = True
                
                if abs(error_y) > self.deadband:
                    self.current_tilt += error_y * self.kp_tilt
                    needs_movement = True
                
                self.current_pan = max(0.0, min(180.0, self.current_pan))
                self.current_tilt = max(45.0, min(135.0, self.current_tilt))

                if needs_movement:
                    status_msg = "ADJUSTING"
                    self.hw.move_servo(self.current_pan, self.current_tilt)
                else:
                    status_msg = "LOCKED"
                    self.get_logger().info("🎯 Target locked. Initiating analysis...")
                    self.perform_analysis(frame)
                        
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
                
        elif self.has_analyzed:
            status_msg = "ANALYSIS COMPLETE"
        
        self.pub_status.publish(String(data=status_msg))
        try:
            cv2.putText(final_display_frame, f"Demo: {status_msg}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            ros_img = self.bridge.cv2_to_imgmsg(final_display_frame, "bgr8")
            self.pub_image.publish(ros_img)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish image: {e}")

    def perform_analysis(self, frame):
        """Captures the frame, saves for PlantCV, and runs a quick HSV threshold."""
        self.has_analyzed = True
        
        # 1. Ensure directory exists and capture the image
        photo_dir = "/home/rover/ros2_ws/photos"
        os.makedirs(photo_dir, exist_ok=True)
        
        # Save as target_*.jpg so the background PlantCV script finds it
        timestamp = int(time.time())
        filename = os.path.join(photo_dir, f"target_{timestamp}.jpg")
        
        self.get_logger().info("📸 Capturing high-resolution image...")
        cv2.imwrite(filename, frame)
        self.get_logger().info(f"💾 Image saved to {filename}")
        
        # 2. On-the-fly HSV Thresholding (Green Leaf Detection)
        self.get_logger().info("🔍 Running live HSV leaf detection...")
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        green_pixels = cv2.countNonZero(green_mask)
        total_pixels = frame.shape[0] * frame.shape[1]
        green_percent = (green_pixels / total_pixels) * 100
        
        # 3. Output results
        self.get_logger().info("✅ Live Quick-Scan Complete:")
        self.get_logger().info(f"   - Green Canopy Coverage: {green_percent:.2f}%")
        
        if green_percent > 2.0:
            self.get_logger().info("   - Plant Verified: Handing off to PlantCV for disease mapping...")
        else:
            self.get_logger().info("   - Warning: Low green content detected. PlantCV may return 0%.")

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = PlantDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Demo interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()