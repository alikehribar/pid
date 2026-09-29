import math
import sys
import time

import analogio
import board
import pwmio
import supervisor
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


N_AVG = 1024
R_FIXED = 1000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15
gate = pwmio.PWMOut(board.GP2, frequency=1000, duty_cycle=0)
ntc = analogio.AnalogIn(board.GP26)
controller = Controller()
pending = ""


def read_celsius():
    total = 0
    for _ in range(N_AVG):
        total = (total + ntc.value)
    adc = (total / N_AVG)
    ratio = (adc / 65535.0)
    if ((ratio <= 0.001) or (ratio >= 0.999)):
        return (None, adc)
    resistance = (R_FIXED * (ratio / (1.0 - ratio)))
    kelvin = (1.0 / ((1.0 / T0) + (math.log((resistance / R0)) / BETA)))
    return ((kelvin - 273.15), adc)


def set_command(raw):
    line = raw.strip().lower()
    if (line == "off"):
        controller.stop()
        gate.duty_cycle = 0
        print("# set off")
        return
    (name, _, text) = line.partition("=")
    if (not text):
        (name, text) = ("sp_target", name)
    try:
        value = float(text.strip())
    except ValueError:
        print("# invalid command")
        return
    if (not math.isfinite(value)):
        print("# invalid command")
        return
    if (name == "sp_target"):
        if (controller.set_target(value)):
            print("# set sp_target", value)
        else:
            print("# rejected target; use 30-100 C after a valid measurement")
        return
    bounds = {"kp": (0.0, 100.0), "ki": (0.0, 5.0),
              "kd": (0.0, 1000.0), "derivative_filter_s": (0.2, 30.0),
              "ramp_rate": (0.01, 2.0)}
    if ((name not in bounds) or (value < bounds[name][0])
            or (value > bounds[name][1])):
        print("# invalid setting")
        return
    setattr(controller, name, value)
    print("# set", name, value)


def poll_serial():
    global pending
    while supervisor.runtime.serial_bytes_available:
        char = sys.stdin.read(1)
        if (char in ("\n", "\r")):
            line = pending.strip().lower()
            pending = ""
            if (line):
                set_command(line)
        elif (len(pending) < 32):
            pending += char
        else:
            pending = "#"


t_start = time.monotonic()
t_next = t_start
last_time = t_start
print("# time_s setpoint_c temperature_c duty_pct error_c filtered_rate_c_s"
      " p_pct i_pct d_pct")

while True:
    poll_serial()
    (temperature, _adc) = read_celsius()
    now = time.monotonic()
    dt = max(0.001, (now - last_time))
    last_time = now
    row = controller.step(temperature, dt)
    gate.duty_cycle = int(((controller.duty / 100.0) * 65535))
    if (row is None):
        print("# invalid NTC reading; output off")
    else:
        values = tuple(round(item, 4) for item in row)
        print(round((now - t_start), 3), *values)
    now = time.monotonic()
    t_next = max((t_next + SAMPLE_S), now)
    time.sleep(max(0.0, (t_next - now)))
