# --- CONFIGURATION ---
import math

# Vision System setup
TEAM_NAME = "HydrogenEnergy"
TEAM_TYPE = "DATA"   # Change for your mission type
ARUCO_ID = 5
ROOM_NUM = 1116

# FUEL CELL READER PINS
LDR1 = #
LDR2 = #
LDR3 = #
LDR4 = #
LDR5 = #

# ULTRASONIC SENSORS

    # --- LEFT SENSOR ---
    L_TRIG =
    L_ECHO =
    
    # --- RIGHT SENSOR ---
    R_TRIG = 
    R_ECHO =
    
    # --- FRONT SENSOR ---
    F_TRIG =
    F_ECHO = 





# VOLTAGE 
VOLTAGE_PIN = 34
VREF = 3.3  # Reference voltage

# MOTOR PINS 
LEFT_IN1_PIN = 18
LEFT_IN2_PIN = 19
LEFT_PWM_PIN = 21

RIGHT_IN1_PIN = 22
RIGHT_IN2_PIN = 23
RIGHT_PWM_PIN = 25

# MARBLE DEPLOYMENT PINS
MARBLE_SERVO_PIN = 24
MARBLE_SERVO_FREQUENCY = 50

# Navigation tolerances
NAV_TOLERANCE_DEG = 5
NAV_TOLERANCE_DIST = 0.15  # meters

# Timing
LOOP_DELAY = 0.1  # seconds

# --- PID Constants ---

# DRIVE
KP_DRIVE = 0.0
KI_DRIVE = 0.0
KD_DRIVE = 0.0

# TURN
KP_TURN = 0.0
KI_TURN = 0.0
KD_TURN = 0.0




# --- HELPER FUNCTIONS ---

def rad_to_deg(radians):
    #Convert radians to degrees
    return radians * (180 / math.pi)

def deg_to_rad(degrees):
    #Convert degrees to radians
    return degrees * (math.pi / 180)

def angle_wrap(angle):
    # Ensure turning the shortest angle possible (yaw)
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# PID Control - Modularize it into one function versus a class
def pid_control(setpoint, current, prev_error, integral, dt, kp, ki, kd, angle=False):
    
    error = setpoint - current
    if angle:
        error = angle_wrap(error)
    integral += error * dt
    derivative = (error - prev_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error, integral