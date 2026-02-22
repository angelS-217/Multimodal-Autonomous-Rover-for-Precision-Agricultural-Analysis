import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import math

# ================= CONFIGURATION (PLEASE EDIT THIS SECTION) =================
# ⚠️ NOTE: Use BCM Pin numbering (GPIO XX), not physical pin numbers!

# --- Left Motor Pins ---
IN1 = 17   # Direction Pin A (Logic)
IN2 = 27   # Direction Pin B (Logic)
ENA = 22   # Speed Control (PWM)

# --- Right Motor Pins ---
IN3 = 5    # Direction Pin C (Logic)
IN4 = 6    # Direction Pin D (Logic)
ENB = 13   # Speed Control (PWM)

# --- Motor Settings ---
# PWM Frequency (Hz), usually 50-100Hz is good for L298N/TB6612
PWM_FREQ = 100 

# Wheel Distance (meters)
# Measure the distance between the center of the left tire and the right tire.
WHEEL_DIST = 0.20 

# Speed Coefficient
# This determines how fast the motor spins for a given command.
# If the robot moves too slow, INCREASE this. If too fast, DECREASE this.
SPEED_COEFFICIENT = 100.0 
# ============================================================================

class DirectDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        self.get_logger().info('Starting Raspberry Pi Direct Motor Driver...')

        # 1. Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        pins = [IN1, IN2, ENA, IN3, IN4, ENB]
        GPIO.setup(pins, GPIO.OUT)

        # 2. Initialize PWM (Range 0-100)
        self.pwm_left = GPIO.PWM(ENA, PWM_FREQ)
        self.pwm_right = GPIO.PWM(ENB, PWM_FREQ)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # 3. Subscribe to /cmd_vel command from Nav2/Teleop
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        # --- Kinematics: Split linear/angular velocity into left/right wheel speeds ---
        v = msg.linear.x      # Linear velocity (forward/backward)
        w = msg.angular.z     # Angular velocity (turning)

        # Differential drive kinematics formula
        v_left = v - (w * WHEEL_DIST / 2.0)
        v_right = v + (w * WHEEL_DIST / 2.0)

        # --- Execute Motor Control ---
        self.set_motor(self.pwm_left, IN1, IN2, v_left)
        self.set_motor(self.pwm_right, IN3, IN4, v_right)

    def set_motor(self, pwm_obj, pin_a, pin_b, speed_val):
        """
        Control GPIO pins based on calculated speed.
        speed_val > 0: Forward
        speed_val < 0: Backward
        speed_val = 0: Stop
        """
        # 1. Speed Mapping: Convert m/s to PWM duty cycle (0-100)
        duty_cycle = abs(speed_val) * SPEED_COEFFICIENT
        
        # Clamp duty cycle between 0 and 100
        if duty_cycle > 100: duty_cycle = 100
        if duty_cycle < 0: duty_cycle = 0

        # 2. Control Direction Pins
        # We add a small deadzone (0.05) to prevent jitter when stopping
        if speed_val > 0.05:  # Forward
            GPIO.output(pin_a, GPIO.HIGH)
            GPIO.output(pin_b, GPIO.LOW)
            pwm_obj.ChangeDutyCycle(duty_cycle)
        elif speed_val < -0.05: # Backward
            GPIO.output(pin_a, GPIO.LOW)
            GPIO.output(pin_b, GPIO.HIGH)
            pwm_obj.ChangeDutyCycle(duty_cycle)
        else: # Stop
            GPIO.output(pin_a, GPIO.LOW)
            GPIO.output(pin_b, GPIO.LOW)
            pwm_obj.ChangeDutyCycle(0)

    def destroy_node(self):
        # Cleanup GPIO on exit to ensure motors stop
        self.get_logger().info('Shutting down: Cleaning up GPIO...')
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DirectDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    ##### Pin Numbers: You must use BCM numbering (e.g., 17, 27, etc., as written in the code), not the physical pin arrangement on the Raspberry Pi (1, 2, 3...).
    ##### Common Ground: The GND of the driver board must be connected to the GND of the Raspberry Pi; otherwise, the signal will not be transmitted.
    ##### Test: If the wheels rotate in the wrong direction after running the program, simply swap the pin numbers for IN1 and IN2 (or IN3 and IN4).