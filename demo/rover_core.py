#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  
from std_msgs.msg import String      # ADDED FOR LORA TELEMETRY
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
        super().__init__('rover_core')
        self.get_logger().info('Rover Node Started. Connecting to LoRa...')

        self.is_auto = False
        self.ser = None

        # 1. ROS 2 Subscriber for Auto-Navigation
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 2. ROS 2 Subscriber for Outgoing LoRa Telemetry
        self.telemetry_sub = self.create_subscription(
            String,
            '/lora_telemetry',
            self.telemetry_callback,
            10
        )
        
        # --- NEW: ROS 2 Publisher for Rover Mode ---
        self.mode_pub = self.create_publisher(String, '/rover_mode', 10)

        # 3. Serial Setup
        try:
            self.ser = serial.Serial(LORA_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Success: Connected to {LORA_PORT}')
        except Exception as e:
            self.get_logger().error(f'FAILED to open Serial: {e}')
            self.get_logger().warn('Make sure the LoRa module is plugged in!')

        # 4. Timer: Check LoRa for Manual Inputs
        self.create_timer(0.01, self.check_serial)

    def telemetry_callback(self, msg):
        """ Sends PlantCV Logs to Laptop via LoRa """
        if self.ser is not None and self.ser.is_open:
            try:
                out_str = msg.data + '\n'
                self.ser.write(out_str.encode('utf-8'))
            except Exception:
                pass

    def clamp_speed(self, speed):
        """Clamp speed between 0.0 and 1.0 for gpiozero PWM"""
        return max(0.0, min(abs(speed), 1.0))

    def cmd_vel_callback(self, msg):
        """ Callback for ROS 2 Navigation Stack using direct_driver logic. """
        if self.is_auto:
            # 1. Scale Nav2's m/s and rad/s up so they translate better to PWM
            # Reduced multiplier from 1.5 to 1.3 to slow down autonomous movement
            linear_x = -msg.linear.x * 1.3   
            angular_z = -msg.angular.z * 1.3

            left_speed = linear_x - angular_z
            right_speed = linear_x + angular_z

            # 2. Add a minimum power bump to overcome motor whining/stalling
            MIN_POWER = 0.35  # 35% power just to get the heavy wheels moving
            
            def apply_deadband(speed):
                if abs(speed) < 0.05:
                    return 0.0
                # Map the speed into the active power band (0.35 to 1.0)
                sign = 1 if speed > 0 else -1
                bumped_speed = MIN_POWER + (abs(speed) * (1.0 - MIN_POWER))
                return sign * bumped_speed

            left_pwm = apply_deadband(left_speed)
            right_pwm = apply_deadband(right_speed)

            # 3. Apply the power using your clean helper function
            self.set_motor(L_EN, L_IN1, L_IN2, self.clamp_speed(left_pwm))
            self.set_motor(R_EN, R_IN3, R_IN4, self.clamp_speed(right_pwm))
                
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
        """ Mixing logic for Manual Tank Drive with Exponential Curve """
        # Square the inputs to create a smooth acceleration curve
        # (multiplying by abs() preserves the negative sign for reversing)
        x_curve = x * abs(x)
        y_curve = y * abs(y)
        
        # Lowered further (to 0.5) for precise mapping control
        SPEED_LIMIT = 0.4
        
        x_final = x_curve * SPEED_LIMIT
        y_final = y_curve * SPEED_LIMIT
        
        left = max(min(y_final - x_final, 1.0), -1.0)
        right = max(min(y_final + x_final, 1.0), -1.0)
        
        self.set_motor(L_EN, L_IN1, L_IN2, left)
        self.set_motor(R_EN, R_IN3, R_IN4, right)

    def check_serial(self):
        """ Reads data without throwing away packets """
        if self.ser is None:
            return
        
        # FIX 3: Process EVERY command instantly to stop latency
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.procCmd(line)
            except Exception:
                pass

    def procCmd(self, line):
        """ Decodes the string command and acts on it """
        try:
            if line == "CMD:SHUTDOWN":
                self.get_logger().fatal("SHUTDOWN COMMAND RECEIVED")
                self.drive_manual(0, 0)
                os.system("sudo shutdown now")

            elif line == "CMD:MODE_AUTO":
                self.get_logger().info("Switched to Auto Nav")
                self.is_auto = True
                self.drive_manual(0, 0)
                # Broadcast the state!
                msg = String()
                msg.data = "CMD:MODE_AUTO"
                self.mode_pub.publish(msg)

            elif line == "CMD:MODE_MANUAL":
                self.get_logger().info("Switched to Manual Nav")
                self.is_auto = False
                self.drive_manual(0, 0)
                # Broadcast the state!
                msg = String()
                msg.data = "CMD:MODE_MANUAL"
                self.mode_pub.publish(msg)

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