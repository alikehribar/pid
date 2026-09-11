import math
import sys
import time

import analogio
import board
import pwmio
import supervisor

SP_TARGET = 60.0
# Retuned against a two-state plant model identified from the live_*.txt logs,
# then checked over 6 step sizes x 16 perturbed plants: heater gain 0.5-2.0x
# (the supply voltage has moved by 1.33x between runs, which is 1.77x in power),
# heater lag x0.7-1.4, loss x0.7-1.5, ambient +-3 C, against both plant
# calibrations. Worst-case overshoot +0.10 C, where the original set gave
# +1.5 C on the same test.
#
# SP_TAU is what does most of the work: smoothing the reference over ~34 s
# instead of 5 s rounds the corner at arrival and all but removes the ramp-
# following lag, so the integral no longer charges on a lag that was never a
# disturbance. That is what let KI grow an order of magnitude without windup.
#
# These gains belong to I_PRED 1 and were searched for it. H_PREDICT is only
# 4.7 because the integral now sees the prediction itself, so the proportional
# term no longer has to aim as far ahead to compensate. Setting I_PRED 0 with
# this set is not the old controller - it is this one with a mismatched
# integral; re-tune if you want to run that way.
SP_RATE = 0.2272
SP_TAU = 34.29
KP = 0.21824
KI = 0.0071573
KD = 0.38398
H_PREDICT = 4.6726
U_MAX = 1.0
TAU_D = 9.9519
DT = 0.1
# 1: the integral is fed the predicted error, the same one P and D act on.
# 0: it is fed the present-day error, as before.
I_PRED = 1.0
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


# Odd, so MAD_WIN // 2 is the true middle element. At 8 it was the upper of
# the two middle samples, biasing the filter up by 0.003 C - negligible next to
# the 0.018 C sensor noise, but free to get right.
MAD_WIN = 7
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


# Duty the plate needs to sit at a temperature. The identified plant model
# under-predicts this by ~1.5x, so the curve comes from the measurements: 17
# settled stretches across 6 logs spanning 60-150 C, residual scatter 7% rms. One log is excluded: 194152 needs 0.56x the duty of every
# other run at BOTH 60 C and 90 C. A constant factor independent of temperature
# is a heater-power difference, not a sensor error - a miscalibrated divider
# shifts 1/T by a constant and so would need a different factor at each
# temperature. It was the DC supply, about 1.33x the voltage of the other runs.
#
# So this curve is tied to one supply voltage: heater power goes as V^2 and the
# duty to hold a temperature as 1/V^2. Change the supply and re-fit.
I_TENV = 25.0
I_K1 = 0.00200045
I_K2 = 3.81446e-06
# The curve is the NOMINAL hold duty. A weaker heater or a lossier plate can
# need roughly twice it, so the allowance is multiplicative (I_SCALE) as well
# as additive (I_MARGIN): an additive margin alone either starves a hot
# setpoint or is so wide at a cold one that it stops bounding anything.
I_SCALE = 3.0
I_MARGIN = 0.20
I_FLOOR = 0.10
I_MAX = 0.9


def hold_duty(celsius):
    over = max((celsius - I_TENV), 0.0)
    return ((I_K1 * over) + (I_K2 * over * over))


def integral_limit(setpoint):
    """Integral authority scaled to the duty this setpoint actually needs.

    One fixed number cannot serve both ends: 0.52 starves a 150 C hold (which
    needs ~31%), while 0.75 hands a 40 C hold (~2%) far more duty than it could
    ever legitimately want. This bounds the integral to the duty the setpoint
    plausibly needs plus a margin for what the curve does not know - heater
    ageing, a colder room, something resting on the plate. I_SCALE is 3 rather
    than 2 because the supply voltage has moved by 1.33x between runs, which is
    1.77x in heater power, and 2 left the integral marginally starved at half
    power into a lossy plate.

    Gives 42% at 60 C and 90% at 150 C, against a flat 75% before.
    """
    return min(I_MAX, max(I_FLOOR, ((I_SCALE * hold_duty(setpoint)) + I_MARGIN)))


def lowpass(state, value, dt, tau):
    """Forward-Euler low-pass, kept stable when dt is long.

    dt is measured, not assumed, and the loop lets it reach 10 * DT after an
    overrun. A step of dt / tau above 2 makes this oscillate instead of smooth
    and above ~2 it diverges outright, so the step is capped: the worst a long
    cycle can do is track the input exactly.
    """
    return (state + ((value - state) * min((dt / tau), 1.0)))


def pid(setpoint, sp_prev, pv, pv_prev, integral, rate_filt, dt):
    raw_rate = ((pv - pv_prev) / dt)
    rate_filt = lowpass(rate_filt, raw_rate, dt, TAU_D)
    sp_rate = ((setpoint - sp_prev) / dt)
    target = (setpoint + (H_PREDICT * sp_rate))
    # The reference stops at SP_TARGET, so aiming past it only invites
    # overshoot. It cannot happen at the shipped constants (H_PREDICT / SP_TAU
    # is 0.38), but both are settable over serial: at sp_tau 5 the aim point
    # reaches 63.0 C for a 60 C target, at sp_tau 1 it reaches 64.2 C.
    if (sp_rate > 0.0):
        target = min(target, SP_TARGET)
    elif (sp_rate < 0.0):
        target = max(target, SP_TARGET)
    predicted = (pv + (H_PREDICT * rate_filt))
    error = (target - predicted)
    # Feeding the integral the same predicted error P and D act on stops it
    # fighting the prediction near arrival. Both fall to the present-day error
    # at steady state, where rate_filt and sp_rate are zero, so this changes no
    # steady-state behaviour - only what the integral does during a move.
    # Worst case over 16 plants x 6 step sizes: overshoot 0.35 -> 0.10 C and
    # offset 0.13 -> 0.06 C, for 11% more settling time at the worst corner.
    integral_error = (error if (I_PRED > 0.5) else (setpoint - pv))
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
        i_limit = integral_limit(setpoint)
        integral = min((i_limit / KI), max(((-i_limit) / KI), integral))
    return (u, integral, rate_filt, error, i_output)


def reference(sp_target, sp_cmd, sp_filt, dt):
    step = (SP_RATE * dt)
    sp_cmd += max((-step), min(step, (sp_target - sp_cmd)))
    sp_filt = lowpass(sp_filt, sp_cmd, dt, SP_TAU)
    return (sp_cmd, sp_filt)


pending = ""


# Accepted range per tunable. A zero TAU_D or SP_TAU raised ZeroDivisionError
# out of the control loop while the gate stayed latched at its last duty, and
# any value under the longest dt the loop allows (10 * DT) would make its filter
# oscillate rather than smooth, so that is the floor for both.
LIMITS = {
    "KP": (0.0, 10.0),
    "KI": (0.0, 1.0),
    "KD": (0.0, 100.0),
    "H_PREDICT": (0.0, 600.0),
    "I_PRED": (0.0, 1.0),
    "TAU_D": ((10.0 * DT), 600.0),
    "I_MAX": (0.0, 1.0),
    "I_MARGIN": (0.0, 1.0),
    "I_SCALE": (0.0, 10.0),
    "SP_RATE": (0.0, 100.0),
    "SP_TAU": ((10.0 * DT), 600.0),
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
    
