# IMPORTS
from plantcv import plantcv as pcv
from plantcv.parallel import WorkflowInputs
import numpy as np
import time
import glob
import os

# NO OUTPUT DEBUG
pcv.params.debug = None

# DISEASE PERCENTAGE FUNCTION
def compute_disease(disease_mask, plant_mask):
    plant_pixels = np.sum(plant_mask == 255)
    disease_pixels = np.sum(disease_mask == 0)
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
    last_processed = None

    while True:

        image_files = glob.glob(os.path.join(photo_directory, "target_*.jpg"))

        if image_files:
            image_files.sort()
            newest_file = image_files[-1]
            
            if newest_file != last_processed:
                frame, path, filename = pcv.readimage(filename=newest_file)
                percent, plant_mask, disease_mask = process_image(frame)
                
                print(f"Processing: {filename}")
                print(f"Overall Plant Health: {percent:.2f}%")
                last_processed = newest_file
                
        time.sleep(5)

if __name__ == "__main__":
    main()