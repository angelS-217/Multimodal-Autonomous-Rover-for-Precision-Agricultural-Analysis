import cv2
import numpy as np

class VisionProcessor:
    def __init__(self):
        # Red has two ranges in HSV space (0-10 and 170-180)
        # Note: Thresholds might need fine-tuning based on actual lighting
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def process_frame(self, frame):
        """
        Core Algorithm
        Return: (processed_frame, (target_x, target_y), found_bool)
        """
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        
        # 1. Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 2. Color Extraction
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = mask1 + mask2
        
        # 3. Denoise (Erode & Dilate)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # 4. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_cx, target_cy = 0, 0
        found = False
        
        if contours:
            # Find the largest contour
            c = max(contours, key=cv2.contourArea)
            # Filter out small noise (e.g., area < 300 pixels)
            if cv2.contourArea(c) > 300:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    target_cx = int(M["m10"] / M["m00"])
                    target_cy = int(M["m01"] / M["m00"])
                    found = True
                    
                    # --- Visualization ---
                    # Draw contour
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    # Draw center point
                    cv2.circle(frame, (target_cx, target_cy), 5, (0, 0, 255), -1)
                    # Draw aiming line
                    cv2.line(frame, (center_x, center_y), (target_cx, target_cy), (255, 255, 0), 1)

        return frame, (target_cx, target_cy), found