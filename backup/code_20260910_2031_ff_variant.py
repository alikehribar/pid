import math
import sys
import time

import analogio
import board
import pwmio
import supervisor

SP_TARGET = 60.0
SP_RATE = 0.2
SP_TAU = 2.0
KP = 0.03
KI = 0.0005
H_PREDICT = 10
U_MAX = 1.0
I_MAX = 0.20
FF_OFFSET = -0.0817
FF_SLOPE = 0.002612
FF_RATE = 0.75
TAU_D = 15
DT = 0.05
R_FIXED = 1000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15
gate = pwmio.PWMOut(board.GP2, frequency=1000, duty_cycle=0)
ntc = analogio.AnalogIn(board.GP26)


N_AVG = 64


def read_adc():
    """Average N_AVG conversions instead of trusting one."""
    total = 0
    for _ in range(N_AVG):
        total += ntc.value
    return (total / N_AVG)


def to_celsius(voltage):
    ratio = (voltage / 3.3)
    if ((ratio <= 0.000001) or (ratio >= 0.9999999)):
        return None
    resistance = (R_FIXED * (ratio / (1.0 - ratio)))
    kelvin = (1.0 / ((1.0 / T0) + (math.log((resistance / R0)) / BETA)))
    return (kelvin - 273.15)


def pid(setpoint, sp_prev, pv, pv_prev, integral, rate_filt, dt):
    raw_rate = ((pv - pv_prev) / dt)
    rate_filt += (((raw_rate - rate_filt) * dt) / TAU_D)
    sp_rate = ((setpoint - sp_prev) / dt)
    target = (setpoint + (H_PREDICT * sp_rate))
    predicted = (pv + (H_PREDICT * rate_filt))
    error = (target - predicted)
    integral_error = (setpoint - pv)
    u_ff = ((FF_OFFSET + (FF_SLOPE * setpoint)) + (FF_RATE * sp_rate))
    u_raw = ((u_ff + (KP * error)) + (KI * integral))
    u = min(U_MAX, max(0.0, u_raw))
    if (KI > 0.0):
        stuck_high = ((u_raw > U_MAX) and (integral_error > 0.0))
        stuck_low = ((u_raw < 0.0) and (integral_error < 0.0))
        if (not (stuck_high or stuck_low)):
            integral += (integral_error * dt)
        integral = min((I_MAX / KI), max(((-I_MAX) / KI), integral))
    return (u, integral, rate_filt, error)


def reference(sp_target, sp_cmd, sp_filt, dt):
    step = (SP_RATE * dt)
    sp_cmd += max((-step), min(step, (sp_target - sp_cmd)))
    sp_filt += (((sp_cmd - sp_filt) * dt) / SP_TAU)
    return (sp_cmd, sp_filt)


pending = ""
TUNABLE = ("KP", "KI", "H_PREDICT", "TAU_D", "I_MAX", "SP_RATE",
           "FF_OFFSET", "FF_SLOPE", "FF_RATE")


def apply_command(line, target):
    """Bare number -> setpoint. 'name value' -> one tunable constant."""
    parts = line.split()
    try:
        if (len(parts) == 1):
            return float(parts[0])
        if ((len(parts) == 2) and (parts[0].upper() in TUNABLE)):
            globals()[parts[0].upper()] = float(parts[1])
            print("#", parts[0].upper(), "=", parts[1])
    except ValueError:
        pass
    return target


def poll_target(target):
    global pending
    while supervisor.runtime.serial_bytes_available:
        char = sys.stdin.read(1)
        if (char in ("\n", "\r")):
            target = apply_command(pending, target)
            pending = ""
        else:
            pending += char
    return target


(sp_cmd, sp_filt) = (None, None)
(integral, rate_filt, pv_prev) = (0.0, 0.0, None)
(err_real, err_pred) = (0.0, 0.0)
(predicted, rate_filt_out) = (0.0, 0.0)
t_start = time.monotonic()
t_next = t_start

while True:
    SP_TARGET = poll_target(SP_TARGET)
    voltage = ((read_adc() * 3.3) / 65535.0)
    celsius = to_celsius(voltage)
    duty = 0.0
    if (celsius is not None):
        if (sp_cmd is None):
            (sp_cmd, sp_filt) = (celsius, celsius)
        if (pv_prev is None):
            pv_prev = celsius
        sp_prev = sp_filt
        (sp_cmd, sp_filt) = reference(SP_TARGET, sp_cmd, sp_filt, DT)
        (duty, integral, rate_filt, err_pred) = pid(
            sp_filt, sp_prev, celsius, pv_prev, integral, rate_filt, DT)
        err_real = (sp_filt - celsius)
        predicted = (celsius + (H_PREDICT * rate_filt))
        rate_filt_out = rate_filt
        pv_prev = celsius
    gate.duty_cycle = int((duty * 65535))
    print(round((time.monotonic() - t_start), 3),
          round((0.0 if (sp_filt is None) else sp_filt), 2),
          round((0.0 if (celsius is None) else celsius), 2),
          round((duty * 100.0), 2),
          round(err_real, 3),
          round(err_pred, 3),
          round(predicted, 2),
          round(rate_filt_out, 4))
    t_next += DT
    time.sleep(max(0.0, (t_next - time.monotonic())))
    
