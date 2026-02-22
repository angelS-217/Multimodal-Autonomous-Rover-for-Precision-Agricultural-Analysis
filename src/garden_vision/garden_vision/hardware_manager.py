import cv2
import numpy as np
import subprocess
import select
import time
import sys

class PiCameraPipe:
    """
    Camera handling class (Pipe version)
    """
    def __init__(self, width=640, height=480, framerate=30):
        self.width = width
        self.height = height
        self.frame_size = int(width * height * 1.5)
        
        cmd = [
            "rpicam-vid",
            "-t", "0",
            "--nopreview",
            "--flush",
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(framerate),
            "--codec", "yuv420",
            "-o", "-"
        ]
        
        print(f"[HW] Starting camera pipe: {' '.join(cmd)}")
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None, bufsize=0)

    def isOpened(self):
        return self.process.poll() is None

    def read(self):
        data = bytearray()
        remaining = self.frame_size
        start_time = time.time()
        TIMEOUT = 1.0 
        
        while remaining > 0:
            if time.time() - start_time > TIMEOUT:
                # Timeout protection
                self.process.stdout.flush()
                return False, None

            rlist, _, _ = select.select([self.process.stdout], [], [], 0.1)
            if not rlist:
                continue
                
            chunk_size = min(4096, remaining)
            chunk = self.process.stdout.read(chunk_size)
            if not chunk:
                return False, None
                
            data.extend(chunk)
            remaining -= len(chunk)
            
        try:
            yuv = np.frombuffer(bytes(data), dtype=np.uint8).reshape((int(self.height * 1.5), self.width))
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            return True, bgr
        except Exception as e:
            return False, None

    def release(self):
        self.process.terminate()
        self.process.wait()


class HardwareInterface:
    """
    Servo control class (Adafruit driver support)
    """
    def __init__(self):
        self.kit = None
        self.sim_mode = False
        
        try:
            # Try loading Adafruit library
            from adafruit_servokit import ServoKit
            
            # Arducam B0283 uses PCA9685, default addr 0x40
            # Standard config: 16 channels
            self.kit = ServoKit(channels=16)
            
            # Test move: Reset servos to 90 deg on startup
            # Arducam B0283: 0=Pan, 1=Tilt
            self.kit.servo[0].angle = 90
            self.kit.servo[1].angle = 90
            
            print("[HW] ✅ Connected to Arducam Gimbal (PCA9685)!")
            
        except Exception as e:
            print(f"[HW] ⚠️ Driver load failed: {e}")
            print("[HW] Simulation mode (Print only)")
            self.sim_mode = True

    def set_camera(self):
        return PiCameraPipe()

    def move_servo(self, pan, tilt):
        """
        Control servo rotation
        pan: Horizontal angle (0-180)
        tilt: Vertical angle (0-180)
        """
        if self.sim_mode:
            # Print only in sim mode
            # print(f"[Sim] Servo -> Pan:{pan:.1f}, Tilt:{tilt:.1f}")
            return

        try:
            # 1. Software limit protection
            safe_pan = max(0, min(180, pan))
            safe_tilt = max(0, min(180, tilt))
            
            # 2. Send cmd to hardware
            # Arducam default: 0=Pan, 1=Tilt
            self.kit.servo[0].angle = safe_pan
            self.kit.servo[1].angle = safe_tilt
            
        except Exception as e:
            print(f"[HW Error] I2C Comm Error: {e}")