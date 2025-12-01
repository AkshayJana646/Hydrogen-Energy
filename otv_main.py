from enes100 import enes100
from machine import ADC, Pin, PWM
from config import *
import math
import time

# --INITALIZE COMPONENTS ---

    # --- VOLTAGE INITIALIZATION ---
adc = ADC(Pin(VOLTAGE_PIN))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)


    # --- MOTOR INITIALIZATION ---
LEFT_IN1 = Pin(LEFT_IN1_PIN, Pin.OUT)
LEFT_IN2 = Pin(LEFT_IN2_PIN, Pin.OUT)
LEFT_PWM = PWM(Pin(LEFT_PWM_PIN), freq=1000)

RIGHT_IN1 = Pin(RIGHT_IN1_PIN, Pin.OUT)
RIGHT_IN2 = Pin(RIGHT_IN2_PIN, Pin.OUT)
RIGHT_PWM = PWM(Pin(RIGHT_PWM_PIN), freq=1000)


    # --- MARBLE DEPLOYMENT INITIALIZATION ---
MARBLE_SERVO = PWM(Pin(MARBLE_SERVO_PIN, mode=Pin.OUT))
MARBLE_SERVO.freq(MARBLE_SERVO_FREQUENCY)






# --- FUEL CELLS / LED READINGS
ldr1 = ADC(Pin(LDR1)); ldr1.atten(ADC.ATTN_11DB); ldr1.width(ADC.WIDTH_12BIT)
ldr2 = ADC(Pin(LDR2)); ldr2.atten(ADC.ATTN_11DB); ldr2.width(ADC.WIDTH_12BIT)
ldr3 = ADC(Pin(LDR3)); ldr3.atten(ADC.ATTN_11DB); ldr3.width(ADC.WIDTH_12BIT)
ldr4 = ADC(Pin(LDR4)); ldr4.atten(ADC.ATTN_11DB); ldr4.width(ADC.WIDTH_12BIT)
ldr5 = ADC(Pin(LDR5)); ldr5.atten(ADC.ATTN_11DB); ldr5.width(ADC.WIDTH_12BIT)


# --- FUEL CELLS DETECTOR ---
def read_light_sensors(samples):
   total = [0, 0, 0, 0, 0]
    for i in range(samples):
        total[0] += ldr1.read_u16()
        total[1] += ldr2.read_u16()
        total[2] += ldr3.read_u16()
        total[3] += ldr4.read_u16()
        total[4] += ldr5.read_u16()
        
    avg0 = total[0] // samples
    avg1 = total[1] // samples
    avg2 = total[2] // samples
    avg3 = total[3] // samples
    avg4 = total[4] // samples

    return [avg0, avg1, avg2, avg3, avg4]

def detect_fuel_cell_status():
    """
    Detects which fuel cell LED is shining.
    Returns: (status_str, color_str, raw_readings)
    """
    readings = read_light_sensors()
    brightest_index = readings.index(max(readings))  # 0-based index

    # Map LDR positions to fuel cell LED status
    status_map = {
        0: ("Not Working", "White"),
        1: ("No Power", "Red"),
        2: ("Low Power", "Yellow"),
        3: ("Full Power", "Green"),
        4: ("Over Power", "Blue")
    }

    status, color = status_map[brightest_index]
    return status, color, readings



# --- MOTOR CONTROL FUNCTION ---
def set_motor_speeds(left_speed, right_speed):
    """
    Set motor speeds in range -1.0 to 1.0.
    Negative = reverse, Positive = forward.
    Uses PWM duty_u16 range 0–65535.
    """

    # Clamp both to [-1.0, 1.0]
    left_speed = max(min(left_speed, 1.0), -1.0)
    right_speed = max(min(right_speed, 1.0), -1.0)

    # --- LEFT MOTOR ---
    if left_speed > 0:
        LEFT_IN1.value(1)
        LEFT_IN2.value(0)
    elif left_speed < 0:
        LEFT_IN1.value(0)
        LEFT_IN2.value(1)
    else:
        # stop
        LEFT_IN1.value(0)
        LEFT_IN2.value(0)

    LEFT_PWM.duty_u16(int(abs(left_speed) * 65535))


    # --- RIGHT MOTOR ---
    if right_speed > 0:
        RIGHT_IN1.value(1)
        RIGHT_IN2.value(0)
    elif right_speed < 0:
        RIGHT_IN1.value(0)
        RIGHT_IN2.value(1)
    else:
        # stop
        RIGHT_IN1.value(0)
        RIGHT_IN2.value(0)

    RIGHT_PWM.duty_u16(int(abs(right_speed) * 65535))



# --- NAVIGATION HELPERS ---
def get_pose():
    return enes100.x, enes100.y, rad_to_deg(enes100.theta)

def angle_to_point(x_target, y_target):
    x, y, theta_deg = get_pose()
    dx = x_target - x
    dy = y_target - y
    target_angle = rad_to_deg(math.atan2(dy, dx))
    heading_error = angle_wrap(target_angle - theta_deg)
    distance = math.sqrt(dx**2 + dy**2)
    return heading_error, distance, target_angle


# --- TURN TO ANGLE ---
def turn_to_angle(target_angle):
    """
    Turn robot in place until facing target_angle (in degrees).
    Uses PID control to smoothly adjust turn rate.
    """
    print("Turning to", target_angle, "degrees")

    prev_error = 0
    integral = 0

    while True:
        # Get current pose
        x, y, yaw = get_pose()

        # PID control for angular error
        control, prev_error, integral = pid_control(
            target_angle, yaw, prev_error, integral,
            LOOP_DELAY, KP_TURN, KI_TURN, KD_TURN, angle=True
        )

        # Normalize control → turn speed (-1.0 to 1.0)
        turn_speed = max(min(control / 90.0, 1.0), -1.0)

        # Apply tank turn (opposite wheel speeds)
        set_motor_speeds(-turn_speed, turn_speed)

        # Compute heading error for termination
        error_deg = angle_wrap(target_angle - yaw)
        print(f"Yaw={yaw:.1f}°, Target={target_angle:.1f}°, Err={error_deg:.1f}°, Speed={turn_speed:.2f}")

        # Stop when within angle tolerance
        if abs(error_deg) < NAV_TOLERANCE_DEG:
            break

        time.sleep(LOOP_DELAY)

    # Stop both motors at end
    set_motor_speeds(0, 0)
    print("Turn complete")

# DRIVE DISTANCE
def drive_distance(distance):
    """
    Drive forward or backward a specific distance (in meters).
    Uses PID control for precise linear motion.
    """
    print("Driving", distance, "meters forward")

    xc, yc, yaw = get_pose()
    prev_error = 0
    integral = 0

    while True:
        x, y, yaw = get_pose()
        dx = x - xc
        dy = y - yc
        traveled = math.sqrt(dx**2 + dy**2)
        remaining = distance - traveled

        # PID control based on distance
        control, prev_error, integral = pid_control(
            distance, traveled, prev_error, integral,
            LOOP_DELAY, KP_DRIVE, KI_DRIVE, KD_DRIVE
        )

        # Convert PID output to normalized forward speed (-1.0 to 1.0)
        drive_speed = max(min(control, 1.0), -1.0)

        # Apply same speed to both motors
        set_motor_speeds(drive_speed, drive_speed)

        print(f"Dist={traveled:.2f}/{distance:.2f}  Speed={drive_speed:.2f}")

        if remaining <= NAV_TOLERANCE_DIST:
            break

        time.sleep(LOOP_DELAY)

    # Stop both motors when done
    set_motor_speeds(0, 0)
    print("Drive complete")
    
# MARBLE DEPLOYMENT

def open_gate():
    
    MARBLE_SERVO.duty_u16(int(65535 * 0.0375)) # 45 degrees
    time.sleep(5) # Wait 5 seconds for marble to drop
    servo.duty_u16(int(65535 * 0.025))  # Return pose back to original set

# --- VOLTAGE READING ---
def read_voltage():
    return (adc.read() / 4095) * VREF




