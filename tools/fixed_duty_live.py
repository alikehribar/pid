import argparse
import glob
import math
import statistics
import time
from bisect import bisect_left

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox

BAUD = 115200
REFRESH_MS = 500
FOLLOW_S = 900.0
MAX_PLOT_POINTS = 6000
MEDIAN_SAMPLES = 5


def filtered_temperature(values):
    current = values[-1]
    if (not math.isfinite(current)):
        return current
    recent = [value for value in values[-MEDIAN_SAMPLES:] if math.isfinite(value)]
    return statistics.median(recent)


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if (not ports):
        raise SystemExit("No /dev/cu.usbmodem* serial port found.")
    return ports[0]


def parse_line(text):
    fields = text.split()
    if (len(fields) != 4):
        return None
    try:
        row = tuple(float(field) for field in fields)
    except ValueError:
        return None
    if ((not math.isfinite(row[0])) or (not math.isfinite(row[2]))
            or (not math.isfinite(row[3])) or (row[0] < 0.0)
            or (row[2] < 0.0) or (row[2] > 100.0)):
        return None
    return row


def main():
    parser = argparse.ArgumentParser(description="Fixed-duty heater measurement plot.")
    parser.add_argument("--port", default=None)
    arguments = parser.parse_args()
    port = (arguments.port if (arguments.port is not None) else find_port())
    link = serial.Serial(port, BAUD, timeout=0)
    link.reset_input_buffer()
    log_path = time.strftime("fixed_duty_%Y%m%d_%H%M%S.csv")
    log = open(log_path, "w", buffering=1)
    log.write("host_elapsed_s,board_elapsed_s,temperature_c,duty_pct,adc_raw\n")
    started = time.monotonic()
    times = []
    temperatures = []
    display_temperatures = []
    duties = []
    state = {"buffer": "", "whole": False}
    (figure, (temperature_ax, duty_ax)) = plt.subplots(
        2, 1, sharex=True, figsize=(11, 7.5))
    figure.canvas.manager.set_window_title("Fixed duty | %s" % port)
    figure.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.22, hspace=0.30)
    (temperature_line,) = temperature_ax.plot([], [], color="tab:blue", label="NTC median (5)")
    (duty_line,) = duty_ax.plot([], [], color="tab:red", drawstyle="steps-post",
                                label="PWM duty")
    temperature_ax.set_ylabel("NTC temperature (°C)")
    duty_ax.set_ylabel("Duty (%)")
    duty_ax.set_xlabel("Computer elapsed time (min)")
    duty_ax.set_ylim(-1.0, 20.0)
    temperature_ax.grid(alpha=0.3)
    duty_ax.grid(alpha=0.3)
    temperature_ax.legend(loc="upper left")
    duty_ax.legend(loc="upper left")
    temperature_ax.set_title("Waiting for measurements ...")
    status = figure.text(0.09, 0.025, "Enter a duty percentage and press Return.", fontsize=9)

    def send_duty(raw):
        try:
            value = float(raw.strip())
        except ValueError:
            value = float("nan")
        if ((not math.isfinite(value)) or (value < 0.0) or (value > 100.0)):
            status.set_text("Invalid duty: enter 0 to 100%.")
            figure.canvas.draw_idle()
            return
        link.write(("duty=%s\n" % raw.strip()).encode("ascii"))
        link.flush()
        status.set_text("Duty command sent; waiting for board confirmation.")
        figure.canvas.draw_idle()

    duty_box = TextBox(figure.add_axes([0.17, 0.11, 0.13, 0.055]),
                       "Duty % ", initial="0")
    duty_box.on_submit(send_duty)
    zero_button = Button(figure.add_axes([0.36, 0.11, 0.14, 0.055]), "0% / Off")
    zero_button.on_clicked(lambda _event: send_duty("0"))
    live_button = Button(figure.add_axes([0.55, 0.11, 0.16, 0.055]), "Live 15 min")
    live_button.on_clicked(lambda _event: state.__setitem__("whole", False))
    whole_button = Button(figure.add_axes([0.76, 0.11, 0.15, 0.055]), "Full run")
    whole_button.on_clicked(lambda _event: state.__setitem__("whole", True))

    def update(_frame):
        state["buffer"] += link.read(4096).decode("utf-8", "replace")
        if (len(state["buffer"]) > 8192):
            state["buffer"] = ""
        while ("\n" in state["buffer"]):
            (raw, state["buffer"]) = state["buffer"].split("\n", 1)
            text = raw.strip()
            if (text.startswith("#")):
                if (text.startswith("# set duty ")):
                    status.set_text(("Board confirmed: " + text[2:]))
                elif (text.startswith("# invalid")):
                    status.set_text(("Board rejected: " + text[2:]))
                continue
            row = parse_line(text)
            if (row is None):
                continue
            elapsed = (time.monotonic() - started)
            times.append(elapsed)
            temperatures.append(row[1])
            display_temperatures.append(filtered_temperature(temperatures))
            duties.append(row[2])
            log.write("%.3f,%.3f,%.4f,%.3f,%.1f\n" %
                      (elapsed, row[0], row[1], row[2], row[3]))
        if (not times):
            return (temperature_line, duty_line)
        first = (0 if state["whole"] else bisect_left(times, (times[-1] - FOLLOW_S)))
        count = (len(times) - first)
        stride = max(1, (((count + MAX_PLOT_POINTS) - 1) // MAX_PLOT_POINTS))
        indices = list(range(first, len(times), stride))
        if (indices[-1] != (len(times) - 1)):
            indices.append((len(times) - 1))
        x = [(times[index] / 60.0) for index in indices]
        y_temperature = [display_temperatures[index] for index in indices]
        y_duty = [duties[index] for index in indices]
        temperature_line.set_data(x, y_temperature)
        duty_line.set_data(x, y_duty)
        span = max((x[-1] - x[0]), 1.0)
        duty_ax.set_xlim(max(0.0, (x[0] - (span * 0.02))),
                         max((x[-1] + (span * 0.02)), (x[0] + 1.0)))
        finite = [value for value in display_temperatures[first:] if math.isfinite(value)]
        if (finite):
            (low, high) = (min(finite), max(finite))
            pad = max(0.5, ((high - low) * 0.08))
            temperature_ax.set_ylim((low - pad), (high + pad))
        duty_ax.set_ylim(-1.0, max(20.0, (max(duties[first:]) + 2.0)))
        temperature_ax.set_title(
            "NTC %.2f °C   |   duty %.3f %%   |   elapsed %.1f min   |   %d rows" %
            (display_temperatures[-1], duties[-1], (times[-1] / 60.0), len(times)))
        return (temperature_line, duty_line)

    animation = FuncAnimation(figure, update, interval=REFRESH_MS,
                              cache_frame_data=False)
    print("Reading %s; saving all rows to %s" % (port, log_path))
    try:
        plt.show()
    finally:
        link.close()
        log.close()
    return (animation, duty_box, zero_button, live_button, whole_button)


if (__name__ == "__main__"):
    main()
