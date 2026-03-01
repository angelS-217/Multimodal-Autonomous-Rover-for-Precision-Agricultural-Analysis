#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

import cv2
import numpy as np
from plantcv import plantcv as pcv
import time
import threading
import os
import csv

# --- PlantCV Configuration ---
pcv.params.debug = None

def compute_disease(disease_mask, plant_mask):
    """Calculates the ratio of diseased pixels to total plant pixels."""
    plant_pixels = np.sum(plant_mask == 255)
    disease_pixels = np.sum(disease_mask == 255)
    if plant_pixels == 0:
        return 0.0
    return (disease_pixels / plant_pixels) * 100

def process_image(frame):
    """
    Optimized for WHOLE PLANT (Canopy) analysis.
    Aggressively separates green biomass from background noise before checking for disease.
    """
    a_channel = pcv.rgb2gray_lab(frame, 'a')
    plant_mask = pcv.threshold.otsu(a_channel, object_type="dark")
    plant_mask = pcv.fill(plant_mask, size=500)

    b_channel = pcv.rgb2gray_lab(frame, 'b')
    disease_mask = pcv.threshold.otsu(b_channel, object_type="light")

    disease_mask = pcv.apply_mask(disease_mask, plant_mask, "black")

    h, w, _ = frame.shape
    pseudocolor = np.zeros((h, w, 3), dtype=np.uint8)
    healthy_mask = cv2.bitwise_and(plant_mask, cv2.bitwise_not(disease_mask))

    pseudocolor[healthy_mask == 255] = (0, 255, 0)  
    pseudocolor[disease_mask == 255] = (0, 0, 255)  

    percent = compute_disease(disease_mask, plant_mask)
    return percent, pseudocolor

# =======================================================
# 📡 Multi-Sensor Background Node
# =======================================================
class SensorListener(Node):
    def __init__(self):
        super().__init__('rover_sensors')
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_gps = None
        
        self.cam_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        
        self.disease_pub = self.create_publisher(Float32, '/crop_health/disease_severity', 10)
        self.lora_pub = self.create_publisher(String, '/lora_telemetry', 10)

    def image_callback(self, msg):
        self.latest_image = msg

    def gps_callback(self, msg):
        self.latest_gps = msg


# =======================================================
# 📍 Waypoint Loader
# =======================================================
def load_waypoints(filepath):
    """Reads X, Y coordinates from a text file, ignoring comments."""
    waypoints = []
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                # Ignore empty lines and comments
                if not line or line.startswith('#'):
                    continue
                try:
                    # Split by comma and convert to floats
                    x, y = map(float, line.split(','))
                    waypoints.append((x, y))
                except ValueError:
                    print(f"⚠️ Skipping invalid waypoint line: {line}")
        print(f"✅ Loaded {len(waypoints)} waypoints from {filepath}")
    else:
        print(f"❌ ERROR: {filepath} not found! Defaulting to origin (0,0).")
        waypoints = [(0.0, 0.0)]
    return waypoints


def main():
    rclpy.init()
    
    sensor_node = SensorListener()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    navigator = BasicNavigator()
    print("🚀 Starting Autonomous Agricultural Patrol...")
    navigator.waitUntilNav2Active()

    # 📍 LOAD WAYPOINTS FROM TEXT FILE
    route_file = "/home/rover/ros2_ws/demo/route.txt"
    waypoints = load_waypoints(route_file)

    save_dir = "/home/rover/ros2_ws/photos"
    os.makedirs(save_dir, exist_ok=True)
    
    csv_path = os.path.join(save_dir, 'field_analysis_log.csv')
    log_exists = os.path.isfile(csv_path)
    
    if not log_exists:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Waypoint_ID', 'Latitude', 'Longitude', 'Disease_Severity_Pct', 'Raw_Image_File', 'Map_Image_File'])

    # Infinite patrol loop
    while rclpy.ok():
        for i, (target_x, target_y) in enumerate(waypoints):
            print(f"\n🚜 Navigating to Plant {i+1}: (x={target_x:.2f}, y={target_y:.2f})")
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = target_x
            goal_pose.pose.position.y = target_y
            goal_pose.pose.orientation.w = 1.0

            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print(f"Driving... Distance remaining: {feedback.distance_remaining:.2f} meters", end='\r')
                time.sleep(0.5)

            result = navigator.getResult()
            if result == rclpy.task.TaskResult.SUCCEEDED:
                print(f"\n✅ Arrived at Plant {i+1}!")
                print("📸 Stabilizing camera and extracting leaf image...")
                time.sleep(1.0) 
                
                if sensor_node.latest_image is not None:
                    try:
                        cv_img = sensor_node.bridge.imgmsg_to_cv2(sensor_node.latest_image, desired_encoding='bgr8')
                        
                        lat = sensor_node.latest_gps.latitude if sensor_node.latest_gps else 0.0
                        lon = sensor_node.latest_gps.longitude if sensor_node.latest_gps else 0.0
                        
                        print("🔬 Analyzing plant health...")
                        disease_percent, health_map = process_image(cv_img)
                        print(f"📊 RESULT: Plant {i+1} is {disease_percent:.2f}% diseased.")
                        print(f"📍 LOCATION: Lat {lat:.6f}, Lon {lon:.6f}")
                        
                        severity_msg = Float32()
                        severity_msg.data = float(disease_percent)
                        sensor_node.disease_pub.publish(severity_msg)
                        
                        timestamp = int(time.time())
                        raw_filename = f"plant_{i+1}_raw_{timestamp}.jpg"
                        map_filename = f"plant_{i+1}_map_{disease_percent:.0f}pct_{timestamp}.jpg"
                        
                        cv2.imwrite(os.path.join(save_dir, raw_filename), cv_img)
                        cv2.imwrite(os.path.join(save_dir, map_filename), health_map)
                        
                        csv_row = f"{timestamp},{i+1},{lat:.6f},{lon:.6f},{disease_percent:.2f}"
                        with open(csv_path, 'a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow([timestamp, i+1, lat, lon, f"{disease_percent:.2f}", raw_filename, map_filename])
                            
                        lora_msg = String()
                        lora_msg.data = f"LOG:{csv_row}"
                        sensor_node.lora_pub.publish(lora_msg)
                            
                        print(f"💾 Log updated and images saved to: {save_dir}")
                        
                    except Exception as e:
                        print(f"❌ Failed to process plant data: {e}")
                else:
                    print("⚠️ No camera feed detected!")
                
                time.sleep(2.0)

            elif result == rclpy.task.TaskResult.CANCELED:
                print(f"\n⚠️ Waypoint to Plant {i+1} was canceled.")
            elif result == rclpy.task.TaskResult.FAILED:
                print(f"\n❌ Failed to reach Plant {i+1}. Skipping to next...")

        print("\n🔄 Field patrol complete! Restarting route...")

    rclpy.shutdown()

if __name__ == '__main__':
    main()