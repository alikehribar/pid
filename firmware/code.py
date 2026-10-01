import math
import sys
import time

import analogio
import board
import pwmio
import supervisor

SAMPLE_S = 0.2
MAX_DUTY = 100.0
D_FILTER_S = 3.0
RAMP_C_S = 0.2
R_FIXED = 1000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15

pid = {"kp": 13.321502, "ki": 0.149848, "kd": 296.071540,
       "target": None, "setpoint": None, "last": None, "rate": 0.0,
       "integral": 0.0, "duty": 0.0, "history": []}


def clamp(value, low, high):
    return min(high, max(low, value))


def median_filter(raw):
    pid["history"].append(raw)
    if (len(pid["history"]) > 5):
        pid["history"].pop(0)
    ordered = sorted(pid["history"])
    return ordered[(len(ordered) // 2)]


def set_target(value):
    if ((not (30.0 <= value <= 100.0)) or (pid["last"] is None)):
        return False
    if (pid["target"] is None):
        pid["integral"] = 0.0
        pid["setpoint"] = pid["last"]
    pid["target"] = value
    return True


def stop():
    pid["target"] = None
    pid["setpoint"] = pid["last"]
    pid["duty"] = 0.0
    pid["integral"] = 0.0


def pid_step(raw, dt):
    if ((raw is None) or (raw < (-20.0)) or (raw > 160.0)):
        stop()
        return None
    measured = median_filter(raw)
    if (pid["last"] is not None):
        raw_rate = ((measured - pid["last"]) / dt)
        alpha = (1.0 - math.exp(((-dt) / D_FILTER_S)))
        pid["rate"] += ((raw_rate - pid["rate"]) * alpha)
    pid["last"] = measured
    if (pid["target"] is None):
        pid["duty"] = 0.0
        return (measured, measured, 0.0, 0.0, pid["rate"], 0.0, pid["integral"], 0.0)
    step = (RAMP_C_S * dt)
    pid["setpoint"] += clamp((pid["target"] - pid["setpoint"]), (-step), step)
    error = (pid["setpoint"] - measured)
    p = (pid["kp"] * error)
    d = ((-pid["kd"]) * pid["rate"])
    new_integral = clamp((pid["integral"] + (pid["ki"] * error * dt)), 0.0, MAX_DUTY)
    output = ((p + new_integral) + d)
    if (((output > 0.0) and (output < MAX_DUTY))
            or ((output >= MAX_DUTY) and (error < 0.0))
            or ((output <= 0.0) and (error > 0.0))):
        pid["integral"] = new_integral
    pid["duty"] = clamp(((p + pid["integral"]) + d), 0.0, MAX_DUTY)
    return (pid["setpoint"], measured, pid["duty"], error, pid["rate"], p, pid["integral"], d)


def run_command(line):
    if (line == "off"):
        stop()
        print("# set off")
        return
    (name, _, text) = line.partition("=")
    try:
        value = float(text if text else name)
    except ValueError:
        print("# invalid command")
        return
    if (not text):
        print(("# set target" if set_target(value) else "# rejected target"), value)
    elif ((name in ("kp", "ki", "kd")) and (value >= 0.0)):
        pid[name] = value
        print("# set", name, value)
    else:
        print("# invalid command")


gate = pwmio.PWMOut(board.GP2, frequency=1000, duty_cycle=0)
ntc = analogio.AnalogIn(board.GP26)
typed = ""


def read_celsius():
    total = 0
    for _ in range(1024):
        total = (total + ntc.value)
    ratio = ((total / 1024) / 65535.0)
    if ((ratio <= 0.001) or (ratio >= 0.999)):
        return None
    resistance = (R_FIXED * (ratio / (1.0 - ratio)))
    return ((1.0 / ((1.0 / T0) + (math.log((resistance / R0)) / BETA))) - 273.15)


def read_commands():
    global typed
    while supervisor.runtime.serial_bytes_available:
        char = sys.stdin.read(1)
        if (char in ("\n", "\r")):
            if typed.strip():
                run_command(typed.strip().lower())
            typed = ""
        else:
            typed += char


t_start = time.monotonic()
t_next = t_start
last_time = t_start
print("# time_s setpoint_c temperature_c duty_pct error_c filtered_rate_c_s p_pct i_pct d_pct")
while True:
    read_commands()
    temperature = read_celsius()
    now = time.monotonic()
    dt = max(0.001, (now - last_time))
    last_time = now
    row = pid_step(temperature, dt)
    gate.duty_cycle = int(((pid["duty"] / 100.0) * 65535))
    if (row is None):
        print("# invalid NTC reading; output off")
    else:
        print(round((now - t_start), 3), *[round(value, 4) for value in row])
    t_next = max((t_next + SAMPLE_S), time.monotonic())
    time.sleep(max(0.0, (t_next - time.monotonic())))
