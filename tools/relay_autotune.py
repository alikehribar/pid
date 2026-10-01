import glob
import math
import statistics
import time

import serial

TARGET_C = 70.0
BAND_C = 0.25
SPAN = 15.0
CYCLES = 7
MIN_PHASE_S = 5.0
AMBIENT_C = 23.15
GAIN_C_PER_PCT = 0.825
START_BELOW_C = (TARGET_C - 2.0)
SAFETY_C = min(100.0, (TARGET_C + 20.0))


def read_temperature(link):
    fields = link.readline().decode("utf-8", "replace").split()
    if ((len(fields) != 4) or fields[0].startswith("#")):
        return None
    try:
        return float(fields[1])
    except ValueError:
        return None


def send_duty(link, percent):
    link.write(("duty=%.4f\n" % percent).encode("ascii"))


def new_test(now, temperature):
    return {"heating": True, "cycles": 0, "bias": 50.0, "d": 50.0,
            "command": 100.0, "done": "", "high_start": now,
            "low_start": 0.0, "t_high": 0.0, "t_low": 0.0,
            "max": temperature, "min": temperature, "results": []}


def relay_step(s, now, temperature):
    s["max"] = max(s["max"], temperature)
    s["min"] = min(s["min"], temperature)
    if (temperature >= SAFETY_C):
        s["done"] = "safety limit"
    elif (s["heating"] and (temperature > (TARGET_C + BAND_C))
            and ((now - s["high_start"]) >= MIN_PHASE_S)):
        s["heating"] = False
        s["t_high"] = (now - s["high_start"])
        s["low_start"] = now
        s["max"] = temperature
        s["command"] = max(0.0, (s["bias"] - s["d"]))
    elif ((not s["heating"]) and (temperature < (TARGET_C - BAND_C))
            and ((now - s["low_start"]) >= MIN_PHASE_S)):
        s["heating"] = True
        s["t_low"] = (now - s["low_start"])
        s["high_start"] = now
        if (s["cycles"] == 0):
            s["bias"] = ((TARGET_C - AMBIENT_C) / GAIN_C_PER_PCT)
        else:
            s["bias"] += ((s["d"] * (s["t_high"] - s["t_low"]))
                          / (s["t_high"] + s["t_low"]))
        s["bias"] = min(80.0, max(20.0, s["bias"]))
        s["d"] = min(SPAN, s["bias"], (99.0 - s["bias"]))
        if ((s["cycles"] > 2) and (s["max"] > s["min"])):
            amplitude = ((s["max"] - s["min"]) / 2.0)
            ku = ((4.0 * s["d"]) / (math.pi * amplitude))
            s["results"].append((ku, (s["t_high"] + s["t_low"])))
        s["cycles"] += 1
        s["min"] = temperature
        s["command"] = min(100.0, (s["bias"] + s["d"]))
        if (s["cycles"] >= CYCLES):
            s["done"] = "completed"
    if s["done"]:
        s["command"] = 0.0
    return s["command"]


def gains(results):
    last = results[-3:]
    ku = statistics.median(r[0] for r in last)
    tu = statistics.median(r[1] for r in last)
    kp = (0.6 * ku)
    return (ku, tu, kp, ((2.0 * kp) / tu), ((kp * tu) / 8.0))


def main():
    port = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
    link = serial.Serial(port, 115200, timeout=1)
    log = open(time.strftime("relay_autotune_%Y%m%d_%H%M%S.csv"), "w")
    log.write("time_s,temperature_c,command_pct,cycle,bias_pct\n")
    print("waiting until the heater is below %.1f C" % START_BELOW_C)
    t0 = time.monotonic()
    s = None
    try:
        while ((s is None) or (not s["done"])):
            temperature = read_temperature(link)
            if (temperature is None):
                continue
            now = (time.monotonic() - t0)
            if (s is None):
                if (temperature > START_BELOW_C):
                    continue
                s = new_test(now, temperature)
                send_duty(link, s["command"])
                print("test started at %.2f C" % temperature)
            else:
                old = s["command"]
                if (relay_step(s, now, temperature) != old):
                    send_duty(link, s["command"])
            log.write("%.3f,%.4f,%.3f,%d,%.4f\n" % (
                now, temperature, s["command"], s["cycles"], s["bias"]))
    finally:
        send_duty(link, 0.0)
        log.close()
    print("stopped: " + s["done"])
    if s["results"]:
        print("Ku %.6f  Tu %.3f s  Kp %.6f  Ki %.6f  Kd %.6f" % gains(s["results"]))


if (__name__ == "__main__"):
    main()
