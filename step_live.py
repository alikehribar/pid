import math
import time

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import serial

PORT = "/dev/cu.usbmodem11301"
SCHEDULE = ([(60.0, 0.0)]
            + [(300.0, ((n + 1) / 10.0)) for n in range(10)]
            + [(600.0, 0.0)])
T_MAX_INIT = 90.0
MARGIN_C = 15.0
COMMAND_EVERY = 5.0
R_FIXED = 10000.0
R0 = 10000.0
BETA = 3950.0
T0 = 298.15


def to_celsius(voltage):
    ratio = (voltage / 3.3)
    if ((ratio <= 0.001) or (ratio >= 0.999)):
        return None
    resistance = (R_FIXED * ((ratio / (1.0 - ratio))))
    kelvin = (1.0 / (((1.0 / T0) + ((math.log((resistance / R0)) / BETA)))))
    return (kelvin - 273.15)


def duty_at(elapsed):
    marker = 0.0
    for (span, duty) in SCHEDULE:
        marker += span
        if (elapsed < marker):
            return duty
    return None


link = serial.Serial(PORT, 115200, timeout=0)
time.sleep(2.0)
link.reset_input_buffer()
path = time.strftime("step_%Y%m%d_%H%M%S.txt")
log = open(path, "w")
log.write("# time_s  celsius  duty_pct\n")
print("logging to %s, close the window or press Ctrl-C to stop" % (path,))

plt.ion()
(figure, axis) = plt.subplots(figsize=(11, 7))
figure.subplots_adjust(bottom=0.26)
axis.set_xlabel("time (s)")
axis.set_ylabel("temperature (C)")
axis.grid(True, alpha=0.3)
twin = axis.twinx()
twin.set_ylabel("duty (%)")
twin.set_ylim(-5.0, 105.0)
(dots,) = axis.plot([], [], ".", markersize=4, color="crimson")
(line,) = twin.plot([], [], "-", linewidth=1.5, color="steelblue")
ceiling = axis.axhline(T_MAX_INIT, color="darkorange", linestyle="--", linewidth=1.2)

slider = Slider(figure.add_axes([0.12, 0.13, 0.58, 0.04]), "T max (C)",
                40.0, 150.0, valinit=T_MAX_INIT, valstep=1.0)
button_start = Button(figure.add_axes([0.76, 0.115, 0.10, 0.06]), "Start")
button_pause = Button(figure.add_axes([0.87, 0.115, 0.10, 0.06]), "Pause")
state = {"running": False}


def on_start(event):
    state["running"] = True


def on_pause(event):
    state["running"] = False


button_start.on_clicked(on_start)
button_pause.on_clicked(on_pause)


(times, temps, duties, buffer, t_zero) = ([], [], [], "", None)
(run_time, last_tick, last_command) = (0.0, time.monotonic(), 0.0)
try:
    while plt.fignum_exists(figure.number):
        now = time.monotonic()
        if state["running"]:
            run_time += (now - last_tick)
        last_tick = now
        duty = (duty_at(run_time) if state["running"] else 0.0)
        if (duty is None):
            print("schedule finished")
            break
        if (((now - last_command)) >= COMMAND_EVERY):
            link.write(("%.5f %.1f\n" % (duty, slider.val)).encode())
            link.flush()
            last_command = now
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
                t_zero = board_time
            times.append((board_time - t_zero))
            temps.append(celsius)
            duties.append(applied)
            log.write("%.3f %.4f %.1f\n" % ((board_time - t_zero), celsius, applied))
        log.flush()
        if ((len(temps) > 0) and (temps[-1] > (slider.val + MARGIN_C))):
            print("ABORT at %.2f C, limit was %.1f C" % (temps[-1], slider.val))
            break
        ceiling.set_ydata([slider.val, slider.val])
        axis.set_title("%s   t = %.0f s   T = %s   duty = %s"
                       % (("RUNNING" if state["running"] else "PAUSED"), run_time,
                          (("%.2f C" % (temps[-1],)) if (len(temps) > 0) else "-"),
                          (("%.1f %%" % (duties[-1],)) if (len(duties) > 0) else "-")))
        dots.set_data(times, temps)
        line.set_data(times, duties)
        axis.relim()
        axis.autoscale_view()
        twin.set_xlim(axis.get_xlim())
        plt.pause(0.25)
except KeyboardInterrupt:
    print("stopped by user")
finally:
    link.write(b"0.0\n")
    link.flush()
    link.close()
    log.close()
    print("saved %s, %d samples" % (path, len(temps)))
