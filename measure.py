import glob
import math
import time

import matplotlib.pyplot as plt
import serial

R_FIXED = 10000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15


def to_celsius(voltage):
    ratio = (voltage / 3.3)
    if ((ratio <= 0.001) or (ratio >= 0.999)):
        return None
    resistance = (R_FIXED * (ratio / (1.0 - ratio)))
    kelvin = (1.0 / ((1.0 / T0) + (math.log((resistance / R0)) / BETA)))
    return (kelvin - 273.15)


port = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
link = serial.Serial(port, 115200, timeout=0)
time.sleep(2.0)
link.reset_input_buffer()
path = time.strftime("measure_%Y%m%d_%H%M%S.txt")
log = open(path, "w")
log.write("# time_s  celsius  duty_pct\n")
print("port %s, logging to %s" % (port, path))

plt.ion()
(figure, axis) = plt.subplots(figsize=(11, 6))
axis.set_xlabel("time (s)")
axis.set_ylabel("temperature (C)")
axis.grid(True, alpha=0.3)
twin = axis.twinx()
twin.set_ylabel("duty (%)")
twin.set_ylim(-5.0, 105.0)
(dots,) = axis.plot([], [], ".", markersize=4, color="crimson")
(line,) = twin.plot([], [], "-", linewidth=1.5, color="steelblue")

(times, temps, duties, buffer, t_zero) = ([], [], [], "", None)
try:
    while plt.fignum_exists(figure.number):
        buffer += link.read(4096).decode("utf-8", "replace")
        while ("\n" in buffer):
            (raw, buffer) = buffer.split("\n", 1)
            parts = raw.split()
            if (len(parts) < 3):
                continue
            try:
                (board_time, celsius) = (float(parts[0]), to_celsius(float(parts[1])))
                applied = float(parts[2])
            except ValueError:
                continue
            if (celsius is None):
                continue
            if (t_zero is None):
                t_zero = time.monotonic()
            stamp = (time.monotonic() - t_zero)
            times.append(stamp)
            temps.append(celsius)
            duties.append(applied)
            log.write("%.3f %.4f %.1f\n" % (stamp, celsius, applied))
        log.flush()
        if (len(temps) > 0):
            axis.set_title("t = %.0f s   T = %.2f C   duty = %.1f %%"
                           % (times[-1], temps[-1], duties[-1]))
        dots.set_data(times, temps)
        line.set_data(times, duties)
        axis.relim()
        axis.autoscale_view()
        twin.set_xlim(axis.get_xlim())
        plt.pause(0.25)
except KeyboardInterrupt:
    print("stopped by user")
finally:
    link.close()
    log.close()
    print("saved %s, %d samples" % (path, len(temps)))
