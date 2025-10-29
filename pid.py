class PID:
    def __init__(self, kP, kI, kD, setpoint=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.setpoint = setpoint
        self.prevError = 0.0
        self.integral = 0.0

    def compute(self, current):
        # Calculate error
        error = self.setpoint - current

        # Proportional
        P = self.kP * error

        # Integral
        self.integral += error
        I = self.kI * self.integral

        # Derivative
        D = self.kD * (error - self.prevError)

        self.prevError = error

        # Output
        output = P + I + D
        return output

    def reset(self):
        # Resets past accumulated error
        self.integral = 0.0
        self.prevError = 0.0

        
    
        
        
        
        
        
    
        