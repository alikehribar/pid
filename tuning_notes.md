# Why the error drifts instead of sitting still (run live_20260910_173139.txt)

All numbers below are measured from the logs unless marked derived.

## What the log shows, 30 s averages

| window | setpoint | measured | duty | err_real | integral share |
|---|---|---|---|---|---|
| 150-180 s (ramp) | 56.96 C | 54.09 C | 29.1 % | +2.87 C | 21.4 % |
| 210-240 s (top)  | 59.99 C | 61.37 C | 13.5 % | -1.38 C | 19.4 % |
| 450-480 s (late) | 59.99 C | 60.30 C | 13.7 % | -0.31 C | 14.6 % |

`integral share` is derived: `duty - 100 * KP * err_pred`, i.e. the part of the
duty that the I term is holding.

The error is not oscillating. It is one long integral windup and unwind:

1. The setpoint ramps at `SP_RATE = 0.2 C/s`. Holding a ramp needs extra power
   on top of the steady-state power. With the feedforward removed from the
   board file, that power can only come from `KP * error`, so the error parked
   at +2.9 to +4.2 C for the whole ramp.
2. While the error sat there, the integral charged up to 21.8 % duty. The
   steady-state duty at 60 C is 7.2 % (measured in live_20260910_171835.txt,
   and 31.8 % at 150 C in live_20260910_162310.txt).
3. So at the top of the ramp the loop was carrying about 14 % too much power.
   The temperature overshot to 61.4 C (-1.38 C error).
4. The unwind rate is `KI * error` = 0.00035 * 1.2 = 0.042 %/s (derived), so
   shedding 14 % takes about 330 s. Measured: 21.8 % -> 14.6 % in 270 s, still
   falling at the end of the log. That slow crawl is the drift you see.

Lowering KP made it worse, as expected: any load the integral has not caught
yet shows up as `error = load / KP`. Measured peak-to-peak error at 60 C was
0.42 C with KP = 0.08 and 1.49 C with KP = 0.03, a factor of 3.5 for a factor
of 2.7 in gain.

## Patch, in order of effect

### 1. Put the feedforward back (biggest effect)

The board copy of `code.py` dropped `u_ff`. Restore it in `pid()`:

```python
    u_ff = ((FF_OFFSET + (FF_SLOPE * setpoint)) + (FF_RATE * sp_rate))
    i_output = (KI * integral)
    u_raw = (((u_ff + (KP * error)) + i_output))
```

with the constants

```python
FF_OFFSET = -0.0817
FF_SLOPE = 0.002612
FF_RATE = 0.75
```

The old calibration is still valid: the two measured steady points, 7.2 % at
60 C and 31.8 % at 150 C, give slope 0.00273 duty/C and offset -0.092
(derived), within 5 % of the constants above. `FF_RATE * SP_RATE` = 0.75 * 0.2
= 15 % duty, which is exactly the ramp power the error was straining to make.

### 2. Cap the integral: `I_MAX = 0.5` -> `I_MAX = 0.10`

With the feedforward carrying the steady-state and ramp power, the integral
only has to trim a residual. A 50 % integral authority is what let it wind up
14 % too far and then take 5 minutes to give it back.

### 3. Put KP back up: `KP = 0.03` -> `KP = 0.08`

Steady-state error scales as 1/KP. 0.08 measured 0.42 C peak-to-peak, 0.03
measured 1.49 C.

### 4. Low-pass the measurement (kills the fast spikes, separate issue)

Insert after `mad_filter`:

```python
PV_TAU = 0.3               # s, measurement low-pass; the 18 s lookahead barely notices it
pv_filt = None


def smooth(reading):
    """One-pole low-pass: passes the trend, drops the per-sample ADC noise."""
    global pv_filt
    if (pv_filt is None):
        pv_filt = reading
    pv_filt += (((reading - pv_filt) * DT) / PV_TAU)
    return pv_filt
```

and change the loop line to `celsius = smooth(mad_filter(celsius))`.

Derived from replaying this log: duty step std 0.240 % -> 0.006 %, largest duty
jump 1.68 % -> 0.05 %, err_pred peak-to-peak 0.98 C -> 0.25 C.

## What to expect after 1-3

The ramp error should fall from about +3.5 C to a few tenths of a degree,
the overshoot at the top of the ramp from 1.4 C to under 0.2 C, and the settled
error should stop crawling because the integral no longer has 14 % to unload.
