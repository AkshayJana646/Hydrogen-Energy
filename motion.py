#This code will make the OTV face 0 degrees and move in the + x direction. 
import math
import time

TARGET_HEADING = 0.0          # radians (0° = +x direction)
ANG_TOL = math.radians(5)     # acceptable heading error ±5°
TURN_SPEED = 0.15             # turning rate 
FWD_SPEED = 0.20              # forward speed

def get_heading():
    # Replace with your IMU or simulator heading read (in radians)
    return theta

def set_drive(linear_speed, angular_speed):
    # Replace with actual motor control command
    pass

def stop():
    set_drive(0.0, 0.0)

def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

def orient_to_zero():
    while True:
        theta = get_heading()
        err = angle_wrap(TARGET_HEADING - theta)
        if abs(err) < ANG_TOL:
            break
        ang_cmd = TURN_SPEED if err > 0 else -TURN_SPEED
        set_drive(0.0, ang_cmd)
        time.sleep(0.05)
    stop()

def drive_positive_x():
    set_drive(FWD_SPEED, 0.0)

def run():
    orient_to_zero()
    drive_positive_x()

if __name__ == "__main__":
    run()
