import argparse
import bisect
import glob
import sys
import time

import matplotlib.pyplot as plt
import serial

from ntc import resistance_to_celsius, voltage_to_resistance
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox

BAUD = 115200
DEFAULT_WINDOW_S = 120.0
REFRESH_MS = 200
DOT_SIZE = 3.5
LINE_STYLE = "-"
MARKER = "none"
LINE_WIDTH = 1.4
H_PREDICT = 5.0
TAU_D = 5.0
NO_SETPOINT = float("nan")


def find_port():
  candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
  if (len(candidates) == 0):
    raise SystemExit("No /dev/cu.usbmodem* port found.")
  return candidates[0]


def parse_line(text):
  try:
    numbers = [float(part) for part in text.split()]
  except ValueError:
    return None
  if (len(numbers) == 4):
    return (numbers[0], numbers[1], numbers[2], numbers[3])
  if (len(numbers) == 3):
    resistance = voltage_to_resistance(numbers[1])
    if (resistance is None):
      return None
    return (numbers[0], NO_SETPOINT, resistance_to_celsius(resistance), numbers[2])
  return None


def errors_from(state, row):
  (moment, setpoint, measured, _duty) = row
  span = ((moment - state["t_prev"]) if (state["t_prev"] is not None) else 0.0)
  real = (setpoint - measured)
  if (span <= 0.0):
    (state["t_prev"], state["pv_prev"], state["sp_prev"]) = (moment, measured, setpoint)
    return (real, real)
  raw_rate = ((measured - state["pv_prev"]) / span)
  state["rate_filt"] += (((raw_rate - state["rate_filt"]) * span) / TAU_D)
  sp_rate = ((setpoint - state["sp_prev"]) / span)
  target = (setpoint + (H_PREDICT * sp_rate))
  predicted = (measured + (H_PREDICT * state["rate_filt"]))
  (state["t_prev"], state["pv_prev"], state["sp_prev"]) = (moment, measured, setpoint)
  return (real, (target - predicted))


def rescale(axes, values, floor_span):
  clean = [value for value in values if (value == value)]
  if (len(clean) == 0):
    return
  (low, high) = (min(clean), max(clean))
  pad = max(((high - low) * 0.08), floor_span)
  axes.set_ylim((low - pad), (high + pad))


def main():
  parser = argparse.ArgumentParser(description="Live dot plot of the PID loop.")
  parser.add_argument("--port", default=None)
  parser.add_argument("--window", type=float, default=DEFAULT_WINDOW_S)
  arguments = parser.parse_args()

  port = (arguments.port if (arguments.port is not None) else find_port())
  link = serial.Serial(port, BAUD, timeout=0)
  link.reset_input_buffer()

  log_path = time.strftime("live_%Y%m%d_%H%M%S.txt")
  log = open(log_path, "w")
  log.write("# time_s  celsius  duty_pct\n")

  state = {"buffer": "", "following": True, "whole": False,
           "t_prev": None, "pv_prev": 0.0, "sp_prev": 0.0, "rate_filt": 0.0}
  time_s = []
  setpoint_c = []
  measured_c = []
  power_pct = []
  error_c = []
  predicted_c = []

  (figure, (top, bottom, errors)) = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
  figure.canvas.manager.set_window_title("PID loop, port %s" % (port,))
  (dots_setpoint,) = top.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="0.6", label="setpoint")
  (dots_measured,) = top.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:blue", label="measured")
  (dots_power,) = bottom.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:red")
  top.set_ylabel("Temperature (°C)")
  top.legend(loc="upper left")
  top.grid(alpha=0.3)
  top.set_title("Waiting for the board ...")
  bottom.set_ylabel("Power (%)")
  bottom.grid(alpha=0.3)
  (line_error,) = errors.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:green", label="real error")
  (line_predicted,) = errors.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:purple", label="predicted error (%.0f s ahead)" % (H_PREDICT,))
  errors.axhline(0.0, color="0.7", linewidth=0.8)
  errors.set_ylabel("Error (\u00b0C)")
  errors.set_xlabel("Time (s)")
  errors.legend(loc="upper left")
  errors.grid(alpha=0.3)

  def draw_title():
    if (len(time_s) == 0):
      top.set_title("Waiting for the board ...")
      return
    tail = ("" if state["following"] else "   ·   frozen (r: live, a: whole run)")
    top.set_title(
      "SP %.2f °C   ·   T %.2f °C   ·   P %.1f %%   ·   %d readings   ·   %.1f s%s"
      % (setpoint_c[-1], measured_c[-1], power_pct[-1], len(time_s), time_s[-1], tail)
    )

  def on_key(event):
    if (event.key == " "):
      state["following"] = False
    elif (event.key == "r"):
      state["following"] = True
      state["whole"] = False
    elif (event.key == "a"):
      state["following"] = True
      state["whole"] = True
    draw_title()
    figure.canvas.draw_idle()

  figure.canvas.mpl_connect("key_press_event", on_key)

  def update(_frame):
    state["buffer"] += link.read(4096).decode("utf-8", "replace")
    while ("\n" in state["buffer"]):
      (raw, state["buffer"]) = state["buffer"].split("\n", 1)
      row = parse_line(raw.strip())
      if (row is None):
        continue
      if ((len(time_s) > 0) and (row[0] < time_s[-1])):
        for series in (time_s, setpoint_c, measured_c, power_pct, error_c, predicted_c):
          series.clear()
      time_s.append(row[0])
      setpoint_c.append(row[1])
      measured_c.append(row[2])
      power_pct.append(row[3])
      (real, ahead) = errors_from(state, row)
      error_c.append(real)
      predicted_c.append(ahead)
      log.write("%.3f %.4f %.1f\n" % (row[0], row[2], row[3]))
      sys.stdout.write("\r%8.1f s   %7.3f C   %5.1f %%" % (row[0], row[2], row[3]))
    log.flush()
    sys.stdout.flush()
    if (len(time_s) == 0):
      return (dots_setpoint, dots_measured, dots_power, line_error, line_predicted)
    dots_setpoint.set_data(time_s, setpoint_c)
    dots_measured.set_data(time_s, measured_c)
    dots_power.set_data(time_s, power_pct)
    line_error.set_data(time_s, error_c)
    line_predicted.set_data(time_s, predicted_c)
    draw_title()
    if (not state["following"]):
      return (dots_setpoint, dots_measured, dots_power, line_error, line_predicted)
    if (state["whole"]):
      start = time_s[0]
    else:
      start = max(time_s[0], (time_s[-1] - arguments.window))
    first = bisect.bisect_left(time_s, start)
    span = max((time_s[-1] - start), 1.0)
    bottom.set_xlim((start - (span * 0.02)), (time_s[-1] + (span * 0.02)))
    rescale(top, (setpoint_c[first:] + measured_c[first:]), 0.25)
    rescale(bottom, power_pct[first:], 2.0)
    rescale(errors, (error_c[first:] + predicted_c[first:]), 0.5)
    return (dots_setpoint, dots_measured, dots_power, line_error, line_predicted)

  print("Reading %s. Close the window to stop." % (port,))
  print("Keys: space freezes the axes, r goes back to live, a shows the whole run.")
  def send_target(text):
    try:
      value = float(text)
    except ValueError:
      return
    link.write(("%.3f\n" % (value,)).encode())
    link.flush()
    print("\nsent target %.2f C" % (value,))

  animation = FuncAnimation(figure, update, interval=REFRESH_MS, cache_frame_data=False)
  plt.tight_layout(rect=(0.0, 0.07, 1.0, 1.0))
  target_box = TextBox(figure.add_axes([0.16, 0.015, 0.12, 0.038]), "target (\u00b0C)  ")
  target_box.on_submit(send_target)
  plt.show()
  link.close()
  log.close()
  print("\nsaved %s, %d rows" % (log_path, len(time_s)))
  return animation


if (__name__ == "__main__"):
  main()
