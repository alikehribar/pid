"""Run a Marlin-style Astrom-Hagglund relay autotune."""

import argparse
import glob
import math
import statistics
import time

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

BAUD = 115200
REFRESH_MS = 200
MIN_PHASE_S = 5.0
TEST_TIMEOUT_S = 3600.0
START_MARGIN_C = 2.0
AMBIENT_C = 23.15
GAIN_C_PER_PCT = 0.825
RELAY_SPAN_MAX = 15.0
HYSTERESIS_C = 0.25


def clamp(value, low, high):
    return min(high, max(low, value))


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
    if ((not math.isfinite(row[0])) or (not math.isfinite(row[1]))
            or (not math.isfinite(row[2])) or (row[2] < 0.0)
            or (row[2] > 100.0)):
        return None
    return row


class RelayAutotuner:
    def __init__(self, target, required_cycles):
        self.target = target
        self.required_cycles = required_cycles
        self.safety_limit = min(100.0, (target + 20.0))
        self.running = False
        self.finished = False
        self.heating = True
        self.cycles = 0
        self.bias = 50.0
        self.relay_d = 50.0
        self.command = 0.0
        self.started_at = 0.0
        self.t_high_start = 0.0
        self.t_low_start = 0.0
        self.t_high = 0.0
        self.t_low = 0.0
        self.maximum = float("-inf")
        self.minimum = float("inf")
        self.results = []
        self.reason = ""
        self.initial_temperature = 0.0

    def start(self, now, temperature):
        self.running = True
        self.started_at = now
        self.t_high_start = now
        self.maximum = temperature
        self.minimum = temperature
        self.initial_temperature = temperature
        self.command = 100.0
        return self.command

    def stop(self, reason):
        self.running = False
        self.finished = True
        self.command = 0.0
        self.reason = reason
        return self.command

    def calculate_cycle(self):
        amplitude = ((self.maximum - self.minimum) / 2.0)
        period = (self.t_high + self.t_low)
        if ((amplitude <= 0.0) or (period <= 0.0)):
            return None
        ultimate_gain = ((4.0 * self.relay_d) / (math.pi * amplitude))
        result = (ultimate_gain, period, amplitude, self.minimum, self.maximum,
                  self.bias, self.relay_d)
        self.results.append(result)
        return result

    def step(self, now, temperature):
        self.maximum = max(self.maximum, temperature)
        self.minimum = min(self.minimum, temperature)
        if (temperature >= self.safety_limit):
            return (self.stop("temperature safety limit"), "safety")
        if ((now - self.started_at) >= TEST_TIMEOUT_S):
            return (self.stop("test timeout"), "timeout")
        if (self.heating and (temperature > (self.target + HYSTERESIS_C))
                and ((now - self.t_high_start) >= MIN_PHASE_S)):
            self.heating = False
            self.t_low_start = now
            self.t_high = (now - self.t_high_start)
            self.maximum = temperature
            self.command = max(0.0, (self.bias - self.relay_d))
            return (self.command, "high_to_low")
        if ((not self.heating) and (temperature < (self.target - HYSTERESIS_C))
                and ((now - self.t_low_start) >= MIN_PHASE_S)):
            self.heating = True
            self.t_high_start = now
            self.t_low = (now - self.t_low_start)
            if (self.cycles == 0):
                estimate = ((self.target - AMBIENT_C) / GAIN_C_PER_PCT)
                self.bias = clamp(estimate, 20.0, 80.0)
                self.relay_d = min(RELAY_SPAN_MAX, self.bias,
                                   (99.0 - self.bias))
            elif (self.cycles > 0):
                correction = ((self.relay_d * (self.t_high - self.t_low))
                              / (self.t_low + self.t_high))
                self.bias = clamp((self.bias + correction), 20.0, 80.0)
                self.relay_d = min(RELAY_SPAN_MAX, self.bias,
                                   (99.0 - self.bias))
            if (self.cycles > 2):
                self.calculate_cycle()
            self.cycles += 1
            self.minimum = temperature
            if (self.cycles >= self.required_cycles):
                return (self.stop("completed"), "completed")
            self.command = min(100.0, (self.bias + self.relay_d))
            return (self.command, "low_to_high")
        return (None, None)


def summarize(results):
    if (not results):
        return None
    selected = results[-min(3, len(results)):]
    ultimate_gain = statistics.median(item[0] for item in selected)
    ultimate_period = statistics.median(item[1] for item in selected)
    kp = (0.6 * ultimate_gain)
    ki = ((2.0 * kp) / ultimate_period)
    kd = ((kp * ultimate_period) / 8.0)
    return (ultimate_gain, ultimate_period, kp, ki, kd)


def main():
    parser = argparse.ArgumentParser(description="Marlin-style relay PID autotune.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--target", type=float, default=70.0)
    parser.add_argument("--cycles", type=int, default=7)
    arguments = parser.parse_args()
    if ((arguments.target < 40.0) or (arguments.target > 80.0)):
        raise SystemExit("Target must be between 40 and 80 C.")
    if ((arguments.cycles < 5) or (arguments.cycles > 12)):
        raise SystemExit("Cycles must be between 5 and 12.")
    port = (arguments.port if (arguments.port is not None) else find_port())
    link = serial.Serial(port, BAUD, timeout=0)
    link.reset_input_buffer()
    tuner = RelayAutotuner(arguments.target, arguments.cycles)
    log_path = time.strftime("relay_autotune_%Y%m%d_%H%M%S.csv")
    log = open(log_path, "w", buffering=1)
    log.write("host_s,board_s,temperature_c,board_duty_pct,command_pct,"
              "heating,cycle,bias_pct,relay_d_pct\n")
    started = time.monotonic()
    state = {"buffer": "", "last_valid": started, "reported": False,
             "link": link}
    times = []
    temperatures = []
    commands = []
    (figure, (temperature_ax, duty_ax)) = plt.subplots(
        2, 1, sharex=True, figsize=(11, 7.5))
    figure.canvas.manager.set_window_title("Relay autotune | %s" % port)
    (temperature_line,) = temperature_ax.plot([], [], color="tab:blue",
                                               label="NTC temperature")
    (target_line,) = temperature_ax.plot([], [], color="0.4", linestyle="--",
                                         label="tune target")
    (duty_line,) = duty_ax.plot([], [], color="tab:red", drawstyle="steps-post",
                                label="relay command")
    temperature_ax.set_ylabel("Temperature (C)")
    duty_ax.set_ylabel("Duty (%)")
    duty_ax.set_xlabel("Elapsed time (min)")
    temperature_ax.grid(alpha=0.3)
    duty_ax.grid(alpha=0.3)
    temperature_ax.legend(loc="upper left")
    duty_ax.legend(loc="upper left")
    figure.subplots_adjust(bottom=0.13, hspace=0.28)
    status = figure.text(0.08, 0.025, "Waiting for the first measurement.", fontsize=9)

    def send_duty(value):
        active_link = state["link"]
        if (active_link is None):
            return
        try:
            active_link.write(("duty=%.4f\n" % value).encode("ascii"))
            active_link.flush()
        except serial.SerialException:
            state["link"] = None

    def report_result():
        if (state["reported"]):
            return
        state["reported"] = True
        result = summarize(tuner.results)
        if (result is None):
            message = ("Autotune stopped without enough stable cycles: " + tuner.reason)
        else:
            (ku, tu, kp, ki, kd) = result
            message = ("Ku %.6f  Tu %.3f s  Kp %.6f  Ki %.6f  Kd %.6f"
                       % (ku, tu, kp, ki, kd))
            result_path = time.strftime("relay_autotune_result_%Y%m%d_%H%M%S.txt")
            with open(result_path, "w") as result_file:
                result_file.write((message + "\n"))
                result_file.write(("source_csv %s\n" % log_path))
            print(("\n" + message))
            print(("Saved result to " + result_path))
        status.set_text(message)

    def stop_test(reason):
        if (not tuner.finished):
            tuner.stop(reason)
        send_duty(0.0)
        report_result()
        figure.canvas.draw_idle()

    stop_button = Button(figure.add_axes([0.78, 0.02, 0.16, 0.055]),
                         "STOP / OUTPUT OFF")
    stop_button.on_clicked(lambda _event: stop_test("stopped by user"))

    def update(_frame):
        now = time.monotonic()
        active_link = state["link"]
        if (active_link is None):
            if (tuner.running and ((now - state["last_valid"]) > 2.0)):
                tuner.stop("serial connection lost")
            ports = sorted(glob.glob("/dev/cu.usbmodem*"))
            if ports:
                try:
                    state["link"] = serial.Serial(ports[0], BAUD, timeout=0)
                    send_duty(tuner.command)
                except serial.SerialException:
                    state["link"] = None
            return (temperature_line, target_line, duty_line)
        try:
            received = active_link.read(4096).decode("utf-8", "replace")
        except serial.SerialException:
            state["link"] = None
            status.set_text("Serial connection lost; waiting to reconnect.")
            return (temperature_line, target_line, duty_line)
        state["buffer"] += received
        while ("\n" in state["buffer"]):
            (raw, state["buffer"]) = state["buffer"].split("\n", 1)
            text = raw.strip()
            if (text.startswith("#")):
                continue
            row = parse_line(text)
            if (row is None):
                continue
            elapsed = (time.monotonic() - started)
            temperature = row[1]
            state["last_valid"] = time.monotonic()
            if ((not tuner.running) and (not tuner.finished)):
                start_limit = (arguments.target - START_MARGIN_C)
                if (temperature > start_limit):
                    status.set_text("Cooling: %.2f C; start requires <= %.2f C."
                                    % (temperature, start_limit))
                else:
                    command = tuner.start(elapsed, temperature)
                    send_duty(command)
                    status.set_text("Autotune started at %.2f C." % temperature)
            elif tuner.running:
                (command, event) = tuner.step(elapsed, temperature)
                if (command is not None):
                    send_duty(command)
                if (event is not None):
                    status.set_text(("%s | cycle %d | bias %.2f | d %.2f"
                                     % (event, tuner.cycles, tuner.bias,
                                        tuner.relay_d)))
                if tuner.finished:
                    report_result()
            times.append(elapsed)
            temperatures.append(temperature)
            commands.append(tuner.command)
            log.write("%.3f,%.3f,%.4f,%.3f,%.3f,%d,%d,%.4f,%.4f\n" %
                      (elapsed, row[0], temperature, row[2], tuner.command,
                       (1 if tuner.heating else 0), tuner.cycles,
                       tuner.bias, tuner.relay_d))
        if (tuner.running and ((now - state["last_valid"]) > 2.0)):
            stop_test("measurement timeout")
        if (not times):
            return (temperature_line, target_line, duty_line)
        x = [(value / 60.0) for value in times]
        temperature_line.set_data(x, temperatures)
        target_line.set_data([x[0], x[-1]], [arguments.target, arguments.target])
        duty_line.set_data(x, commands)
        temperature_ax.set_xlim(x[0], max((x[-1] + 0.2), 1.0))
        temperature_ax.set_ylim((min(temperatures) - 2.0),
                                max((max(temperatures) + 2.0),
                                    (arguments.target + 4.0)))
        duty_ax.set_ylim(-2.0, 102.0)
        temperature_ax.set_title(
            "T %.2f C | target %.2f C | duty %.1f %% | cycle %d/%d" %
            (temperatures[-1], arguments.target, tuner.command,
             tuner.cycles, arguments.cycles))
        return (temperature_line, target_line, duty_line)

    animation = FuncAnimation(figure, update, interval=REFRESH_MS,
                              cache_frame_data=False)
    print("Reading %s; saving to %s" % (port, log_path))
    try:
        plt.show()
    finally:
        try:
            send_duty(0.0)
        finally:
            if (state["link"] is not None):
                state["link"].close()
            log.close()
    return (animation, stop_button)


if (__name__ == "__main__"):
    main()
