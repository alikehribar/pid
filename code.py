import math
import sys
import time

import analogio
import board
import pwmio
import supervisor

SP_TARGET = 60.0
SP_RATE = 0.2
SP_TAU = 5.0
KP = 0.06175
KI = 0.00055
KD = 0.5
H_PREDICT = 12
U_MAX = 1.0
I_MAX = 0.7
TAU_D = 20
DT = 0.1
R_FIXED = 1000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15
gate = pwmio.PWMOut(board.GP2, frequency=1000, duty_cycle=0)
ntc = analogio.AnalogIn(board.GP26)


OVERSAMPLE = 256            

def read_adc():
    total = 0
    for _ in range(OVERSAMPLE):
        total += ntc.value
    return (total / OVERSAMPLE)


def to_celsius(voltage):
    ratio = (voltage / 3.3)
    if ((ratio <= 0.0001) or (ratio >= 0.9999)):
        return None
    resistance = (R_FIXED * (ratio / (1.0 - ratio)))
    kelvin = (1.0 / ((1.0 / T0) + (math.log((resistance / R0)) / BETA)))
    return (kelvin - 273.15)


MAD_WIN = 8           # samples the median spans (7 * DT = 0.35 s)
MAD_K = 3.5                # reject a reading further out than k robust sigmas
MAD_FLOOR = 0.1           # C, ~1.5 ADC codes; keeps the threshold off zero
mad_buf = []


def mad_filter(reading):
    mad_buf.append(reading)
    del mad_buf[:(-MAD_WIN)]
    if (len(mad_buf) < MAD_WIN):
        return reading
    med = sorted(mad_buf)[(MAD_WIN // 2)]
    mad = sorted([abs((x - med)) for x in mad_buf])[(MAD_WIN // 2)]
    sigma = max((1.4826 * mad), MAD_FLOOR)
    if (abs((reading - med)) > (MAD_K * sigma)):
        # Reject this sample only. The raw reading stays in the buffer so a
        # genuine step drags the median across within MAD_WIN // 2 samples;
        # writing the median back latched the filter on the old level forever.
        return med
    return reading


def pid(setpoint, sp_prev, pv, pv_prev, integral, rate_filt, dt):
    raw_rate = ((pv - pv_prev) / dt)
    rate_filt += (((raw_rate - rate_filt) * dt) / TAU_D)
    sp_rate = ((setpoint - sp_prev) / dt)
    target = (setpoint + (H_PREDICT * sp_rate))
    predicted = (pv + (H_PREDICT * rate_filt))
    error = (target - predicted)
    integral_error = (setpoint - pv)
    i_output = (KI * integral)
    # KP * error already carries -KP * H_PREDICT * rate_filt in through
    # `predicted`, so a separate -KD * rate_filt left KD controlling only part
    # of the derivative action. Same arithmetic, one visible gain.
    kd_eff = ((KP * H_PREDICT) + KD)
    u_raw = (((KP * (target - pv)) - (kd_eff * rate_filt)) + i_output)
    u = min(U_MAX, max(0.0, u_raw))
    if (KI > 0.0):
        stuck_high = ((u_raw > U_MAX) and (integral_error > 0.0))
        stuck_low = ((u_raw < 0.0) and (integral_error < 0.0))
        if (not (stuck_high or stuck_low)):
            integral += (integral_error * dt)
        integral = min((I_MAX / KI), max(((-I_MAX) / KI), integral))
    return (u, integral, rate_filt, error, i_output)


def reference(sp_target, sp_cmd, sp_filt, dt):
    step = (SP_RATE * dt)
    sp_cmd += max((-step), min(step, (sp_target - sp_cmd)))
    sp_filt += (((sp_cmd - sp_filt) * dt) / SP_TAU)
    return (sp_cmd, sp_filt)


pending = ""


# Accepted range per tunable. TAU_D and SP_TAU divide inside the loop, so
# their floor is DT rather than zero: a zero there raised ZeroDivisionError out
# of the control loop while the gate stayed latched at its last duty.
LIMITS = {
    "KP": (0.0, 10.0),
    "KI": (0.0, 1.0),
    "KD": (0.0, 100.0),
    "H_PREDICT": (0.0, 600.0),
    "TAU_D": (DT, 600.0),
    "I_MAX": (0.0, 10.0),
    "SP_RATE": (0.0, 100.0),
    "SP_TAU": (DT, 600.0),
}
TUNABLE = tuple(LIMITS)
(SP_MIN, SP_MAX) = (0.0, 200.0)


def in_range(value, low, high):
    """NaN fails every comparison, so test it explicitly before the bounds."""
    return ((value == value) and (low <= value <= high))


def apply_command(line, target):
    """Bare number -> setpoint. 'name value' -> one tunable constant."""
    parts = line.split()
    try:
        if (len(parts) == 1):
            value = float(parts[0])
            if in_range(value, SP_MIN, SP_MAX):
                return value
            print("# reject setpoint", parts[0])
            return target
        if (len(parts) == 2):
            name = parts[0].upper()
            limits = LIMITS.get(name)
            if (limits is None):
                return target
            value = float(parts[1])
            if (not in_range(value, limits[0], limits[1])):
                print("# reject", name, parts[1])
                return target
            globals()[name] = value
            print("#", name, "=", value)
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
(duty, integral_output, bad_reads) = (0.0, 0.0, 0)
(predicted, rate_filt_out) = (0.0, 0.0)
t_start = time.monotonic()
t_next = t_start
t_prev = t_start

while True:
    SP_TARGET = poll_target(SP_TARGET)
    # Use the time the last cycle actually took. Assuming DT after an overrun
    # made the ramp and the integral advance slower than the wall clock.
    now = time.monotonic()
    dt = (now - t_prev)
    if ((dt <= 0.0) or (dt > (10.0 * DT))):
        dt = DT
    t_prev = now
    voltage = ((read_adc() * 3.3) / 65535.0)
    celsius = to_celsius(voltage)
    if (celsius is None):
        # Hold the last duty over a single bad conversion, but do not hold it
        # forever: 20 failures in a row means the sensor is gone, so cut power.
        bad_reads += 1
        # The gap breaks the fixed-DT assumption of the rate term, so restart
        # the derivative on the next good reading instead of dividing by DT.
        pv_prev = None
        if (bad_reads > 20):
            duty = 0.0
    else:
        bad_reads = 0
        # An ADC glitch turns into degrees here, so clean it before the PID.
        celsius = mad_filter(celsius)
        if (sp_cmd is None):
            (sp_cmd, sp_filt) = (celsius, celsius)
        if (pv_prev is None):
            pv_prev = celsius
        sp_prev = sp_filt
        (sp_cmd, sp_filt) = reference(SP_TARGET, sp_cmd, sp_filt, dt)
        (duty, integral, rate_filt, err_pred, integral_output) = pid(
            sp_filt, sp_prev, celsius, pv_prev, integral, rate_filt, dt)
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
          round((integral_output * 100.0), 2),
          round(predicted, 2),
          round(rate_filt_out, 4))
    t_next += DT
    now = time.monotonic()
    if (t_next < now):
        # Overran the period. Resync instead of free-running at full speed
        # trying to catch up on slots that are already gone.
        t_next = now
    time.sleep((t_next - now))
    
