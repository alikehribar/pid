import argparse
import bisect
import glob
import sys
import time

import matplotlib.pyplot as plt
import serial

from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox

BAUD = 115200
DEFAULT_WINDOW_S = 120.0
REFRESH_MS = 200
DOT_SIZE = 3.5
LINE_STYLE = "-"
MARKER = "none"
LINE_WIDTH = 1.4


def find_port():
  candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
  if (len(candidates) == 0):
    raise SystemExit("No /dev/cu.usbmodem* port found.")
  return candidates[0]


def parse_line(text):
  """Read six fields, optionally followed by the integral output percentage."""
  try:
    numbers = [float(part) for part in text.split()]
  except ValueError:
    return None
  if (len(numbers) not in (6, 7, 8, 9)):
    return None
  return tuple(numbers)


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
  log.write("# time_s  setpoint_c  celsius  duty_pct  err_real  err_pred"
            "  predicted_c  rate_c_s\n")

  state = {"buffer": "", "following": True, "whole": False, "horizon": 10.0,
           "trail": False}
  time_s = []
  setpoint_c = []
  measured_c = []
  power_pct = []
  error_c = []
  predicted_c = []
  forecast_c = []
  rate_c_s = []

  (figure, (top, bottom, errors)) = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
  figure.canvas.manager.set_window_title("PID loop, port %s" % (port,))
  (dots_setpoint,) = top.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="0.6", label="setpoint")
  (dots_measured,) = top.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:blue", label="measured")
  (line_forecast,) = top.plot(
    [], [], linestyle=":", color="tab:orange", linewidth=1.0, alpha=0.45,
    label="past predictions (p)")
  line_forecast.set_visible(False)
  (line_tangent,) = top.plot(
    [], [], linestyle="--", marker="o", markersize=5.0,
    color="tab:orange", linewidth=2.0, label="prediction now")
  forecast_text = top.annotate(
    "", xy=(0.0, 0.0), xytext=(-6, 9), textcoords="offset points",
    color="tab:orange", fontsize=9, ha="right")
  (dots_power,) = bottom.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:red")
  top.set_ylabel("Temperature (°C)")
  top.legend(loc="lower right", fontsize=8, ncol=2, framealpha=0.85)
  top.grid(alpha=0.3)
  top.set_title("Waiting for the board ...")
  bottom.set_ylabel("Power (%)")
  bottom.grid(alpha=0.3)
  (line_error,) = errors.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:green", label="real error")
  (line_predicted,) = errors.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:purple", label="controller error (board)")
  errors.axhline(0.0, color="0.7", linewidth=0.8)
  errors.set_ylabel("Error (\u00b0C)")
  errors.set_xlabel("Time (s)")
  errors.legend(loc="upper right", fontsize=8, framealpha=0.85)
  errors.grid(alpha=0.3)

  def draw_title():
    if (len(time_s) == 0):
      top.set_title("Waiting for the board ...")
      return
    tail = ("" if state["following"] else "   ·   frozen (r: live, a: whole run)")
    top.set_title(
      ("SP %.2f °C   ·   T %.2f °C   ·   P %.1f %%   ·   %.1f s%s\n"
       "slope %+.3f °C/s   ->   %.2f °C in %.0f s   ·   gap to SP %+.2f °C")
      % (setpoint_c[-1], measured_c[-1], power_pct[-1], time_s[-1], tail,
         rate_c_s[-1], forecast_c[-1], state["horizon"],
         (forecast_c[-1] - setpoint_c[-1])),
      fontsize=10)

  def on_key(event):
    if (event.key == " "):
      state["following"] = False
    elif (event.key == "r"):
      state["following"] = True
      state["whole"] = False
    elif (event.key == "p"):
      state["trail"] = (not state["trail"])
      line_forecast.set_visible(state["trail"])
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
        for series in (time_s, setpoint_c, measured_c, power_pct, error_c,
                       predicted_c, forecast_c, rate_c_s):
          series.clear()
      time_s.append(row[0])
      setpoint_c.append(row[1])
      measured_c.append(row[2])
      power_pct.append(row[3])
      error_c.append(row[4])
      predicted_c.append(row[5])
      if (len(row) == 9):
        (forecast, rate) = (row[7], row[8])
      elif (len(row) == 8):
        (forecast, rate) = (row[6], row[7])
      else:
        (forecast, rate) = (float("nan"), float("nan"))
      forecast_c.append(forecast)
      rate_c_s.append(rate)
      if (abs(rate) > 0.002):
        state["horizon"] = ((forecast - row[2]) / rate)
      log.write("%.3f %.2f %.4f %.1f %.3f %.3f %.2f %.4f\n"
                % (row[:6] + (forecast, rate)))
      integral_text = (("%+7.2f %%" % row[6]) if (len(row) in (7, 9)) else "     n/a")
      sys.stdout.write("\r%8.1f s   %7.3f C   %5.1f %%   I %s   "
                       % (row[0], row[2], row[3], integral_text))
    log.flush()
    sys.stdout.flush()
    if (len(time_s) == 0):
      return (dots_setpoint, dots_measured, dots_power, line_error, line_predicted)
    dots_setpoint.set_data(time_s, setpoint_c)
    dots_measured.set_data(time_s, measured_c)
    dots_power.set_data(time_s, power_pct)
    line_error.set_data(time_s, error_c)
    line_predicted.set_data(time_s, predicted_c)
    horizon = state["horizon"]
    line_forecast.set_data([(moment + horizon) for moment in time_s], forecast_c)
    line_tangent.set_data([time_s[-1], (time_s[-1] + horizon)],
                          [measured_c[-1], forecast_c[-1]])
    forecast_text.xy = ((time_s[-1] + horizon), forecast_c[-1])
    forecast_text.set_text("%.2f °C" % (forecast_c[-1],))
    draw_title()
    if (not state["following"]):
      return (dots_setpoint, dots_measured, dots_power, line_error, line_predicted)
    if (state["whole"]):
      start = time_s[0]
    else:
      start = max(time_s[0], (time_s[-1] - arguments.window))
    first = bisect.bisect_left(time_s, start)
    span = max((time_s[-1] - start), 1.0)
    bottom.set_xlim((start - (span * 0.02)),
                    ((time_s[-1] + horizon) + (span * 0.02)))
    visible = (forecast_c[first:] if state["trail"] else forecast_c[-1:])
    rescale(top, ((setpoint_c[first:] + measured_c[first:]) + visible), 0.25)
    rescale(bottom, power_pct[first:], 2.0)
    rescale(errors, (error_c[first:] + predicted_c[first:]), 0.5)
    return (dots_setpoint, dots_measured, dots_power, line_error, line_predicted)

  print("Reading %s. Close the window to stop." % (port,))
  print("Keys: space freezes the axes, r goes back to live, a shows the whole run.")
  def send_target(text):
    """Any line goes through: a bare number is the target, 'kp 0.05' is a gain."""
    line = text.strip()
    if (not line):
      return
    link.write((line + "\n").encode())
    link.flush()
    print("\nsent: %s" % (line,))

  animation = FuncAnimation(figure, update, interval=REFRESH_MS, cache_frame_data=False)
  plt.tight_layout(rect=(0.0, 0.07, 1.0, 1.0))
  target_box = TextBox(figure.add_axes([0.16, 0.015, 0.12, 0.038]), "cmd  ")
  target_box.on_submit(send_target)
  plt.show()
  link.close()
  log.close()
  print("\nsaved %s, %d rows" % (log_path, len(time_s)))
  return animation


if (__name__ == "__main__"):
  main()
