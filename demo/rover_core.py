#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # IMPORT ADDED FOR AUTO-NAV
import serial
import os
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# --- CONFIGURATION ---
LORA_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

# --- MOTOR PINS ---
L_EN = PWMOutputDevice(13)
L_IN1 = DigitalOutputDevice(26)
L_IN2 = DigitalOutputDevice(19)

R_EN = PWMOutputDevice(16)
R_IN3 = DigitalOutputDevice(20)
R_IN4 = DigitalOutputDevice(21)

class LoraNode(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.get_logger().info('Rover Node Started. Connecting to LoRa...')

        self.is_auto = False
        self.ser = None

        # 1. ROS 2 Subscriber for Auto-Navigation
        # Listens to /cmd_vel topic when sensors drive the rover
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 2. Serial Setup
        try:
            self.ser = serial.Serial(LORA_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Success: Connected to {LORA_PORT}')
        except Exception as e:
            self.get_logger().error(f'FAILED to open Serial: {e}')
            self.get_logger().warn('Make sure the LoRa module is plugged in!')

        # 3. Timer: Check LoRa for Manual Inputs (0.01s)
        self.create_timer(0.01, self.check_serial)

    def cmd_vel_callback(self, msg):
        """ 
        Callback for ROS 2 Navigation Stack.
        Only executes if we are in AUTO mode.
        """
        if self.is_auto:
            # ROS uses Linear X (forward) and Angular Z (turn)
            linear = msg.linear.x
            angular = msg.angular.z
            
            # Simple mixing for differential drive
            # Tune these multipliers if it turns too fast/slow!
            left = linear - angular
            right = linear + angular
            
            # Send to motors
            self.set_motor(L_EN, L_IN1, L_IN2, max(min(left, 1.0), -1.0))
            self.set_motor(R_EN, R_IN3, R_IN4, max(min(right, 1.0), -1.0))

    def set_motor(self, enable, in1, in2, spd):
        """ Controls a single motor channel """
        if spd > 0:
            in1.on()
            in2.off()
            enable.value = spd
        elif spd < 0:
            in1.off()
            in2.on()
            enable.value = abs(spd)
        else:
            in1.off()
            in2.off()
            enable.value = 0

    def drive_manual(self, x, y):
        """ Mixing logic for Manual Tank Drive """
        SPEED_LIMIT = 0.6 
        x = x * SPEED_LIMIT
        y = y * SPEED_LIMIT
        
        left = max(min(y + x, 1.0), -1.0)
        right = max(min(y - x, 1.0), -1.0)
        
        self.set_motor(L_EN, L_IN1, L_IN2, left)
        self.set_motor(R_EN, R_IN3, R_IN4, right)

    def check_serial(self):
        """ Reads all available data but only processes the very last line """
        if self.ser is None:
            return

        last_line = None
        
        # Clear buffer to get the freshest command
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    last_line = line
            except Exception:
                pass
        
        if last_line:
            self.procCmd(last_line)

    def procCmd(self, line):
        """ Decodes the string command and acts on it """
        try:
            # SHUTDOWN (Always Active)
            if line == "CMD:SHUTDOWN":
                self.get_logger().fatal("SHUTDOWN COMMAND RECEIVED")
                self.drive_manual(0, 0)
                os.system("sudo shutdown now")

            # AUTO MODE
            elif line == "CMD:MODE_AUTO":
                self.get_logger().info("Switched to Auto Nav")
                self.is_auto = True
                self.drive_manual(0, 0) # Stop momentary

            # MANUAL MODE
            elif line == "CMD:MODE_MANUAL":
                self.get_logger().info("Switched to Manual Nav")
                self.is_auto = False
                self.drive_manual(0, 0)

            # MOVEMENT (Only valid if NOT in auto)
            elif line.startswith("M:"):
                if not self.is_auto:
                    parts = line.split(":")
                    if len(parts) > 1:
                        coords = parts[1].split(",")
                        if len(coords) == 2:
                            self.drive_manual(float(coords[0]), float(coords[1]))

        except Exception as e:
            self.get_logger().warn(f'Parse Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LoraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive_manual(0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()