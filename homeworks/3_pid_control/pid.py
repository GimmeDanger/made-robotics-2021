#!/usr/bin/env python


class PID:

    # TODO: Complete the PID class. You may add any additional desired functions

    def __init__(self, Kp, Ki=0.0, Kd=0.0):
        # TODO: Initialize PID coefficients (and errors, if needed)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ep = 0.0
        self.Ei = 0.0
        self.Ed = 0.0

    def UpdateError(self, cte):
        # TODO: Update PID errors based on cte
        prev_Ep = self.Ep
        self.Ep = cte
        self.Ei = self.Ei + cte
        self.Ed = self.Ep - prev_Ep

    def TotalError(self):
        # TODO: Calculate and return the total error
        return self.Kp * self.Ep + self.Kd * self.Ed + self.Ki * self.Ei
