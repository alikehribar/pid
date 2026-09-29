"""Classic PID controller with measurement and derivative filtering."""

import math

SAMPLE_S = 0.2
MAX_DUTY = 100.0
MAX_TARGET_C = 100.0
MEDIAN_SAMPLES = 5
DEFAULT_D_FILTER_S = 3.0
DEFAULT_KP = 13.321502
DEFAULT_KI = 0.149848
DEFAULT_KD = 296.071540
DEFAULT_RAMP_RATE_C_S = 0.2


def clamp(value, low, high):
    return min(high, max(low, value))


class Controller:
    def __init__(self):
        self.target = None
        self.setpoint = None
        self.sensor = None
        self.last_measurement = None
        self.rate = 0.0
        self.duty = 0.0
        self.integral = 0.0
        self.kp = DEFAULT_KP
        self.ki = DEFAULT_KI
        self.kd = DEFAULT_KD
        self.derivative_filter_s = DEFAULT_D_FILTER_S
        self.ramp_rate = DEFAULT_RAMP_RATE_C_S
        self.history = []

    def median_measurement(self, raw):
        self.history.append(raw)
        if (len(self.history) > MEDIAN_SAMPLES):
            self.history.pop(0)
        ordered = sorted(self.history)
        return ordered[(len(ordered) // 2)]

    def observe(self, raw, dt):
        measured = self.median_measurement(raw)
        if (self.last_measurement is None):
            self.sensor = measured
            self.last_measurement = measured
            self.rate = 0.0
            return measured
        raw_rate = ((measured - self.last_measurement) / dt)
        alpha = (1.0 - math.exp(((-dt) / self.derivative_filter_s)))
        self.rate += ((raw_rate - self.rate) * alpha)
        self.sensor = measured
        self.last_measurement = measured
        return measured

    def set_target(self, value):
        if ((not math.isfinite(value)) or (value < 30.0)
                or (value > MAX_TARGET_C) or (self.sensor is None)):
            return False
        if (self.target is None):
            self.integral = 0.0
            self.setpoint = self.sensor
        self.target = value
        return True

    def stop(self):
        self.target = None
        self.setpoint = self.sensor
        self.duty = 0.0
        self.integral = 0.0

    def step(self, raw, dt):
        if ((raw is None) or (not math.isfinite(raw))
                or (not math.isfinite(dt)) or (dt <= 0.0)
                or (raw < (-20.0)) or (raw > 160.0)):
            self.stop()
            return None
        measured = self.observe(raw, dt)
        if (self.target is None):
            self.duty = 0.0
            return (measured, measured, 0.0, 0.0,
                    self.rate, 0.0, self.integral, 0.0)
        gap = (self.target - self.setpoint)
        maximum_step = (self.ramp_rate * dt)
        self.setpoint += clamp(gap, (-maximum_step), maximum_step)
        error = (self.setpoint - measured)
        p_term = (self.kp * error)
        d_term = ((-self.kd) * self.rate)
        candidate_integral = (self.integral + (self.ki * error * dt))
        candidate_integral = clamp(candidate_integral, 0.0, MAX_DUTY)
        candidate_output = (p_term + candidate_integral + d_term)
        can_integrate = (((candidate_output > 0.0)
                          and (candidate_output < MAX_DUTY))
                         or ((candidate_output >= MAX_DUTY) and (error < 0.0))
                         or ((candidate_output <= 0.0) and (error > 0.0)))
        if (can_integrate):
            self.integral = candidate_integral
        raw_output = (p_term + self.integral + d_term)
        self.duty = clamp(raw_output, 0.0, MAX_DUTY)
        return (self.setpoint, measured, self.duty, error,
                self.rate, p_term, self.integral, d_term)
