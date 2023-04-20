class PID:
    def __init__(self, Kp, Ki, Kd):
        self.prev_e = 0
        self.int_e = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def get(self, e, dt):
        dedt = (e - self.prev_e) / dt
        int_e = 0.999 * self.int_e + e * dt

        self.prev_e = e
        self.int_e = int_e

        return self.Kp * e + self.Ki * int_e + self.Kd * dedt


