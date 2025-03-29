class PIDController:
    def __init__(self, Kp=0.1, Ki=0.0, Kd=0.05, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.last_error = 0
        self.integral = 0

    def reset(self):
        self.last_error = 0
        self.integral = 0

    def update(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output
