import cv2
import numpy as np

class VisionProcessor:
    def __init__(self):
        # 🚩 Orange range in HSV space
        # Hue: 5-25 strictly targets orange (0-5 is red, 25-35 is yellow)
        # Saturation: 120-255 ensures we only look for vivid orange, ignoring washed-out colors
        # Value: 120-255 ensures we ignore dark shadows
        self.lower_orange = np.array([2, 80, 80])
        self.upper_orange = np.array([25, 255, 255])

    def process_frame(self, frame):
        """
        Core Algorithm for Orange Flag Detection
        Return: (processed_frame, (target_x, target_y), found_bool)
        """
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        
        # 1. Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 2. Color Extraction (Targeting Orange)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # 3. Denoise (Erode & Dilate)
        # This removes tiny specks of orange noise in the background
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # 4. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_cx, target_cy = 0, 0
        found = False
        
        if contours:
            # Find the largest contour (the biggest orange object in view)
            c = max(contours, key=cv2.contourArea)
            
            # Filter out small noise (e.g., area < 300 pixels)
            if cv2.contourArea(c) > 300:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    target_cx = int(M["m10"] / M["m00"])
                    target_cy = int(M["m01"] / M["m00"])
                    found = True
                    
                    # --- Visualization ---
                    # Draw contour around the flag (Orange box)
                    cv2.drawContours(frame, [c], -1, (0, 165, 255), 2) 
                    # Draw center point of the flag (Red dot)
                    cv2.circle(frame, (target_cx, target_cy), 5, (0, 0, 255), -1)
                    # Draw aiming line from camera center to the flag (Cyan line)
                    cv2.line(frame, (center_x, center_y), (target_cx, target_cy), (255, 255, 0), 1)

        return frame, (target_cx, target_cy), found