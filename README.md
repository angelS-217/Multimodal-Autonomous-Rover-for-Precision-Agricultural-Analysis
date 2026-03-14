🛠️ Pre-Deployment: The Bench Check

    Batteries: Ensure the Rover LiPo is fully charged, and the 8BitDo controller is powered on/paired.

    Environment: If going deep off-grid, ensure your laptop is broadcasting a Mobile Hotspot and the Pi is configured to automatically connect to it for Phase 4.

    Mission Control: Plug the Controller Heltec ESP32 into your laptop via USB. Open a terminal on your laptop and run `python base_station.py`.

        *Leave this terminal visible on your screen. This is where your live field data will appear!*

🗺️ **Step 1: Mapping the Field (Phase 1)**

    Turn on the rover and SSH into the Raspberry Pi from your laptop.

    Run the mapping script:
    Bash

        ```bash
        ~/ros2_ws/demo/01_mapping.sh
        ```

    Use an XBOX ONE S controller  (or 8BitDo controller), drive the rover manually through the crop rows. Drive slowly to allow the Lidar to build a crisp, high-resolution map of the environment.

    Once the area is mapped, park the rover safely. **Do not turn it off or kill Phase 1**.

🎯 Step 2: Foxglove Waypoint Extraction

    Open Foxglove Studio on your laptop.

    Open a **Data Source** connection and connect to the Pi's WebSocket `(ws://<PI_IP_ADDRESS>:8765)`.

    Add a 3D Panel (or 2D Map Panel) to view the live SLAM map you just created.

    Select the Publish Pose tool from the Foxglove toolbar. Click on the map exactly where you want the rover to park in front of a plant.

    Open the Raw Messages panel, look at the /goal_pose topic, and write down the x and y position numbers.

    Repeat this for every plant you want to inspect.

📝 Step 3: Programming the Mission

    Open a second SSH terminal to the Pi.

    Open your new route text file:
    Bash

    nano ~/ros2_ws/demo/route.txt

    Type in the X, Y coordinates you extracted from Foxglove (e.g., 1.5, 2.0).

    Add 0.0, 0.0 as the final line so the rover returns to home base! Save and exit.

👁️ Step 4: Vision & Autonomy Bringup (Phases 2 & 3)

    In your second Pi terminal, start the camera:
    Bash

`~/ros2_ws/demo/02_vision.sh`

Open a third SSH terminal to the Pi.

CRITICAL HARDWARE STEP: Press the 'X' button on your 8BitDo controller. Watch the OLED screen on your laptop's Heltec module confirm that you have switched into AUTO NAV mode.

Launch the autonomy stack:
Bash

    `~/ros2_ws/demo/03_autonav.sh`

🚀 Step 5: The Autonomous Patrol

At this point, you take your hands off the controller!

    The rover will drive itself to the first coordinate in route.txt.

    It will stop, pause to let the camera stabilize, and snap a picture.

    The PlantCV LAB-color algorithm will run, mapping the healthy green vs. diseased tissue.

    The moment it finishes computing, you will hear/see a blip on your laptop! Look at your base_station.py terminal to see the incoming LoRa CSV string populating live.

    The rover will proceed to the next plant.

📡 Step 6: Data Retrieval (Phase 4)

    Wait for the rover to finish its patrol and return to 0.0, 0.0 (Home Base), where it is safely back in range of your Wi-Fi or laptop hotspot.

    On the Pi terminals, press Ctrl+C to shut down Phases 1, 2, and 3.

    Start the local web server:
    Bash

`~/ros2_ws/demo/04_web_server.sh`

On your laptop, open Google Chrome or Firefox and navigate to `http://<PI_IP_ADDRESS>:8000`.

Click and download your high-resolution pseudocolored plant maps and your final `field_analysis_log.csv` file!

Once downloaded, hit Ctrl+C on the web server terminal to shut it down, and power off the rover.