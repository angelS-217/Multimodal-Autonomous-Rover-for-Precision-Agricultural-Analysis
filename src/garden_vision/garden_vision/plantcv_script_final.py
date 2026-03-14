# IMPORTS
from plantcv import plantcv as pcv
import numpy as np
import time
import glob
import os
import csv
from datetime import datetime
import warnings

# Suppress the PlantCV FutureWarning to keep the terminal clean
warnings.filterwarnings("ignore", category=FutureWarning)

# NO OUTPUT DEBUG
pcv.params.debug = None

# DISEASE PERCENTAGE FUNCTION
def compute_disease(disease_mask, plant_mask):
    plant_pixels = np.sum(plant_mask == 255)
    disease_pixels = np.sum(disease_mask == 0)
    
    # Safety check: prevent division by zero if no plant is in the frame
    if plant_pixels == 0:
        return 0.0
        
    percent = (disease_pixels / plant_pixels) * 100
    return percent

# IMAGE PROCESSING FUNCTION
def process_image(frame):
    plant_gray = pcv.rgb2gray_hsv(frame, "h")
    plant_mask = pcv.threshold.otsu(plant_gray)
    plant_mask = pcv.fill(plant_mask, 50000)

    disease_gray = pcv.rgb2gray_lab(frame, "a")
    disease_mask = pcv.threshold.otsu(disease_gray)
    disease_mask = pcv.apply_mask(disease_mask, plant_mask, "white")

    percent = compute_disease(disease_mask, plant_mask)

    return percent, plant_mask, disease_mask

def main():
    photo_directory = "/home/rover/ros2_ws/photos/"
    csv_file_path = os.path.join(photo_directory, "plant_health_log.csv")
    last_processed = None

    # Initialize CSV with headers if it doesn't exist yet
    if not os.path.isfile(csv_file_path):
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Filename", "Disease_Percentage"])

    while True:
        image_files = glob.glob(os.path.join(photo_directory, "target_*.jpg"))

        if image_files:
            image_files.sort()
            newest_file = image_files[-1]
            
            if newest_file != last_processed:
                # Brief pause to ensure the camera is completely finished writing the JPEG to disk
                time.sleep(0.5)
                
                frame, path, filename = pcv.readimage(filename=newest_file)
                percent, plant_mask, disease_mask = process_image(frame)
                
                # 1. Log the data to the CSV
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                with open(csv_file_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([timestamp, filename, f"{percent:.2f}"])
                
                # 2. Force the output to bypass the background buffer and show in terminal
                print(f"[{timestamp}] Processed: {filename} | Health: {percent:.2f}% | Logged to CSV", flush=True)
                
                last_processed = newest_file
                
        time.sleep(5)

if __name__ == "__main__":
    main()