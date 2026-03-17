import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

print("开始测试云台...")

try:
    # 初始化 I2C 总线和 PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50 # 舵机必须是 50Hz

    # 假设你的上下和左右舵机接在通道 0 和 1
    # 如果接在其他通道，请把 0 和 1 改掉
    pan_servo = servo.Servo(pca.channels[0])  
    tilt_servo = servo.Servo(pca.channels[1])

    print("云台归中...")
    pan_servo.angle = 90
    tilt_servo.angle = 90
    time.sleep(1)

    print("云台向左看...")
    pan_servo.angle = 45
    time.sleep(1)

    print("云台向右看...")
    pan_servo.angle = 135
    time.sleep(1)

    print("云台归中并关闭...")
    pan_servo.angle = 90
    pca.deinit()
    print("测试成功结束！")

except Exception as e:
    print(f"发生错误: {e}")