'''
Python script to read controller input from an 8BitDo Pro Controller 2 and send the 
commands to an ESP32 via USB Serial
Accepted Controller Input:
    D-Pad         - 12/30/2025
    Left Joystick - 01/04/2026
    + Start       - 02/10/2026
author: JoseAngel Socorro Pulido
last updated: 1/04/2026
'''

import pygame
import serial
import time
import sys

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'   # check 'Device Manager' -> 'Ports (COM & LPT)'
BAUD_RATE = 115200
DEADZONE = 0.15        # ignore stick movements smaller than this (0.0 to 1.0)
TX_RATE = 0.15       # minimum seconds between LoRa packets (prevents flooding)

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
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
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

def main():
    joystick, ser = init_ctrlr()
    print("Bridge Running... (Ctrl+C to stop)")

    is_auto = False  # Mode Flag
    last_send_time = 0

    try:
        while True:
            pygame.event.pump()  # update internal state

            # Shutdown Logic (check for +Start (Btn 7) -----------
            if joystick.get_button(7):
                print("!!! Shutting Down !!!")
                cmd = "CMD:SHUTDOWN\n"  # Send Command string
                ser.write(cmd.encode('utf-8'))
                ser.flush()
                print(f"TX:  {cmd.strip()}")
                time.sleep(2.0)  # Sleep to debounce
                continue
            
            # Switch Mode Logic (check for 'X' (Btn 3))
            elif joystick.get_button(3):
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
                time.sleep(1.0)  # Debounce
                continue

            # 1. read Analog Stick -------------------
            # Axis 0 = left stick horizontal (-1 Left, +1 Right)
            # Axis 1 = left stick vertical   (-1 Up, +1 Down) *Note: Y is often inverted
            raw_x = joystick.get_axis(0)
            raw_y = joystick.get_axis(1)

            # Apply Deadzone
            x_val = apply_deadzone(raw_x)
            y_val = apply_deadzone(raw_y)

            # 2. Read D-Pad (hat 0) ------------------
            hat_x, hat_y = joystick.get_hat(0)

            # 3. Prioritze D-Pad  if Stick is idle ---
            if x_val == 0 and y_val == 0:
                if hat_x != 0 or hat_y != 0:
                    # map D-Pad to float values
                    x_val = float(hat_x)
                    # invert D-Pad to match Stick Y
                    y_val = float(hat_y) * -1.0

            # 4. Define command ----------------------
            # format to 2 decimal places to save LoRa bandwidth
            # example message: "M:-0.50,0.80"
            cmd = f"M:{x_val:.2f},{y_val:.2f}\n"

            # 5. Send data ---------------------------
            curr_time = time.time()
            # Detect if we are trying to STOP (inputs are zero)
            is_stopping = (x_val == 0.0 and y_val == 0.0)
            # Send immediately if stopping, otherwise wait for timer
            if is_stopping or (curr_time - last_send_time) > TX_RATE:
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