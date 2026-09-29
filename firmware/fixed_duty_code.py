import math
import sys
import time

import analogio
import board
import pwmio
import supervisor

SAMPLE_S = 0.2
MAX_SAFE_C = 90.0
N_AVG = 1024
R_FIXED = 1000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15
gate = pwmio.PWMOut(board.GP2, frequency=1000, duty_cycle=0)
ntc = analogio.AnalogIn(board.GP26)
duty_pct = 0.0
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


def set_duty(raw):
    global duty_pct
    try:
        value = float(raw)
    except ValueError:
        print("# invalid duty")
        return
    if ((not math.isfinite(value)) or (value < 0.0) or (value > 100.0)):
        print("# invalid duty")
        return
    duty_pct = value
    gate.duty_cycle = int(((duty_pct / 100.0) * 65535))
    print("# set duty", round(duty_pct, 3))


def poll_serial():
    global pending
    while supervisor.runtime.serial_bytes_available:
        char = sys.stdin.read(1)
        if (char in ("\n", "\r")):
            line = pending.strip().lower()
            pending = ""
            if (line.startswith("duty=")):
                set_duty(line[5:])
            elif (line):
                print("# invalid command")
        elif (len(pending) < 32):
            pending += char
        else:
            pending = "#"


t_start = time.monotonic()
t_next = t_start
print("# time_s temperature_c duty_pct adc_raw")

while True:
    poll_serial()
    (temperature, adc) = read_celsius()
    if (((temperature is None) or (temperature >= MAX_SAFE_C))
            and (duty_pct > 0.0)):
        set_duty("0")
        print("# safety cutoff")
    temperature_text = ("nan" if (temperature is None) else round(temperature, 4))
    print(round((time.monotonic() - t_start), 3),
          temperature_text,
          round(duty_pct, 3),
          round(adc, 1))
    now = time.monotonic()
    t_next = max((t_next + SAMPLE_S), now)
    time.sleep(max(0.0, (t_next - now)))
