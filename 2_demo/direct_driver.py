import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, OutputDevice

# ================= CONFIGURATION =================
# --- Left Motor Pins (BCM) ---
ENA = 13   # Speed Control (PWM)
IN1 = 19   # Direction A
IN2 = 26   # Direction B

# --- Right Motor Pins (BCM) ---
ENB = 21   # Speed Control (PWM)
IN3 = 16   # Direction C
IN4 = 20   # Direction D

# --- Max Speed Cap ---
# Nav2 might send speeds like 0.22. We cap it at 1.0 for GPIOZero
MAX_SPEED = 1.0 
# ==================================================

class DirectDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        self.get_logger().info('Starting Pi Direct Motor Driver (gpiozero PWM Edition)...')

        # 1. Initialize GPIO via gpiozero
        self.ena_pwm = PWMOutputDevice(ENA, frequency=100)
        self.in1 = OutputDevice(IN1)
        self.in2 = OutputDevice(IN2)

        self.enb_pwm = PWMOutputDevice(ENB, frequency=100)
        self.in3 = OutputDevice(IN3)
        self.in4 = OutputDevice(IN4)

        self.stop_motors()

        # 2. Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('Successfully subscribed to /cmd_vel! Awaiting velocity commands...')

    def clamp_speed(self, speed):
        """Clamp speed between 0.0 and 1.0 for gpiozero PWM"""
        return max(0.0, min(abs(speed), 1.0))

    def stop_motors(self):
        self.in1.off()
        self.in2.off()
        self.ena_pwm.value = 0.0
        
        self.in3.off()
        self.in4.off()
        self.enb_pwm.value = 0.0

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Differential drive math
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        deadzone = 0.05

        # --- Left Motor Control ---
        if left_speed > deadzone:      # Forward
            self.in1.on()
            self.in2.off()
            self.ena_pwm.value = self.clamp_speed(left_speed)
        elif left_speed < -deadzone:   # Backward
            self.in1.off()
            self.in2.on()
            self.ena_pwm.value = self.clamp_speed(left_speed)
        else:                          # Stop
            self.in1.off()
            self.in2.off()
            self.ena_pwm.value = 0.0

        # --- Right Motor Control ---
        if right_speed > deadzone:     # Forward
            self.in3.on()
            self.in4.off()
            self.enb_pwm.value = self.clamp_speed(right_speed)
        elif right_speed < -deadzone:  # Backward
            self.in3.off()
            self.in4.on()
            self.enb_pwm.value = self.clamp_speed(right_speed)
        else:                          # Stop
            self.in3.off()
            self.in4.off()
            self.enb_pwm.value = 0.0

    def destroy_node(self):
        self.get_logger().info('Shutting down: Stopping all motors...')
        self.stop_motors()
        # gpiozero automatically cleans up on exit!
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
