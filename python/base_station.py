'''
Python script to read controller input from an 8BitDo Pro Controller 2,
send commands to an ESP32 via USB Serial, AND listen for incoming 
CSV telemetry logs from the rover.
Accepted Controller Input:
    D-Pad         - 12/30/2025
    Left Joystick - 01/04/2026
    + Start       - 02/10/2026
    X Button      - 02/15/2026
author: JoseAngel Socorro Pulido
last updated: 2/27/2026
'''

import pygame
import serial
import time
import sys
import csv
import os

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'   # check 'Device Manager' -> 'Ports (COM & LPT)'
BAUD_RATE = 115200
DEADZONE = 0.15        # ignore stick movements smaller than this (0.0 to 1.0)
TX_RATE = 0.15       # minimum seconds between LoRa packets (prevents flooding)
CSV_FILENAME = 'crop_health_log.csv'

def init_ctrlr():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found! Check Bluetooth connection.")
        sys.exit()
    js = pygame.joystick.Joystick(0)  # read from joystick 0 (left stick)
    js.init()
    try:
        # Open serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # Reduced timeout for non-blocking reads
        # Assert DTR and RTS for native USB stack to accept data
        ser.dtr = True  # Data Terminial Ready
        ser.rts = True  # Request to Send
        print(f"Serial connected on {SERIAL_PORT}")
        # Wait for ESP32 to reboot and finish setup
        print("Waiting 2 seconds for ESP32 to boot...")
        time.sleep(2)
    except Exception as e:
        print(f"Serial error: {e}")
        sys.exit()
    print(f"Connected to: {js.get_name()}")  # get joystick system name
    print(f"Axes found: {js.get_numaxes()}")  # get number of axes on joystick
    return js, ser

def apply_deadzone(val):
    """Returns 0.0 if value is within the deadzone."""
    if abs(val) < DEADZONE:
        return 0.0
    return val

def setup_csv():
    """ Creates the CSV file and headers if it doesn't exist. """
    if not os.path.isfile(CSV_FILENAME):
        with open(CSV_FILENAME, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Waypoint_ID', 'Latitude', 'Longitude', 'Disease_Severity_Pct'])
        print(f"📁 Created new log file: {CSV_FILENAME}")

def main():
    joystick, ser = init_ctrlr()
    setup_csv()
    
    print("Bridge Running... (Ctrl+C to stop)")

    is_auto = False  # Mode Flag
    last_send_time = 0
    was_stopping = False  
    
    # --- NEW: Track previous button states for edge detection ---
    prev_btn3_state = False
    prev_btn7_state = False

    try:
        while True:
            pygame.event.pump()  # update internal state

            # ==========================================
            # 1. READ INCOMING LORA TELEMETRY
            # ==========================================
            while ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("LOG:"):
                        clean_data = line.replace("LOG:", "")
                        data_list = clean_data.split(",")
                        
                        # Append to CSV
                        with open(CSV_FILENAME, 'a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(data_list)
                            
                        print(f"\n📥 LOG SAVED: {data_list}\n")
                except Exception:
                    pass

            # ==========================================
            # 2. READ & SEND CONTROLLER COMMANDS
            # ==========================================
            
            # Get current button states
            curr_btn3_state = joystick.get_button(3)
            curr_btn7_state = joystick.get_button(7)

            # Shutdown Logic (check for +Start (Btn 7))
            # ONLY trigger if currently pressed AND previously not pressed
            if curr_btn7_state and not prev_btn7_state:
                print("!!! Shutting Down !!!")
                cmd = "CMD:SHUTDOWN\n"  
                ser.write(cmd.encode('utf-8'))
                ser.flush()
                print(f"TX:  {cmd.strip()}")
                # Removed time.sleep(2.0)
            
            # Switch Mode Logic (check for 'X' (Btn 3))
            # ONLY trigger if currently pressed AND previously not pressed
            elif curr_btn3_state and not prev_btn3_state:
                is_auto = not is_auto  # Toggle Mode

                if is_auto:
                    print("Switching to AutoNav...")
                    cmd = "CMD:MODE_AUTO\n"
                else:
                    print("Switching to ManualNav...")
                    cmd = "CMD:MODE_MANUAL\n"
                    
                ser.write(cmd.encode('utf-8'))
                ser.flush()
                print(f"TX: {cmd.strip()}")
                # Removed time.sleep(1.0)
            
            # Update the previous states for the next loop iteration
            prev_btn7_state = curr_btn7_state
            prev_btn3_state = curr_btn3_state

            # read Analog Stick -------------------
            raw_x = joystick.get_axis(0)
            raw_y = joystick.get_axis(1)

            # Apply Deadzone
            x_val = apply_deadzone(raw_x)
            y_val = apply_deadzone(raw_y)

            # Read D-Pad (hat 0) ------------------
            hat_x, hat_y = joystick.get_hat(0)

            # Prioritize D-Pad if Stick is idle ---
            if x_val == 0 and y_val == 0:
                if hat_x != 0 or hat_y != 0:
                    x_val = float(hat_x)
                    y_val = float(hat_y) * -1.0

            # Define command ----------------------
            cmd = f"M:{x_val:.2f},{y_val:.2f}\n"

            # Send data ---------------------------
            curr_time = time.time()
            is_stopping = (x_val == 0.0 and y_val == 0.0)
            
            if is_stopping:
                # Only send the stop command ONCE when the stick returns to center
                if not was_stopping:
                    ser.write(cmd.encode('utf-8'))
                    ser.flush()
                    was_stopping = True
                    last_send_time = curr_time
            else:
                # We are moving! Apply the TX_RATE limit
                was_stopping = False
                if (curr_time - last_send_time) > TX_RATE:
                    ser.write(cmd.encode('utf-8'))
                    ser.flush()
                    print(f"TX: {cmd.strip()}")
                    last_send_time = curr_time
                
            time.sleep(0.01)  # sleep to prevent CPU hogging

    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()

if __name__ == "__main__":
    main()