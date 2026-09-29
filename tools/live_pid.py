import argparse
import bisect
import glob
import math
import sys
import time

import matplotlib.pyplot as plt
import serial

from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox

BAUD = 115200
DEFAULT_WINDOW_S = 120.0
REFRESH_MS = 200
SLOPE_WINDOW_S = 5.0   # the title slope is fitted over this much history
DOT_SIZE = 3.5
LINE_STYLE = "-"
MARKER = "none"

# (name, initial value) for the Tune window; must match the firmware defaults.
TUNABLES = (("sp_target", "70"), ("kp", "13.321502"),
            ("ki", "0.149848"), ("kd", "296.071540"),
            ("derivative_filter_s", "3.0"), ("ramp_rate", "0.2"))
TUNE_BOUNDS = {"sp_target": (30, 100), "kp": (0, 100), "ki": (0, 5),
               "kd": (0, 1000), "derivative_filter_s": (0.2, 30),
               "ramp_rate": (0.01, 2.0)}


def find_port():
  candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
  if (len(candidates) == 0):
    raise SystemExit("No /dev/cu.usbmodem* port found.")
  return candidates[0]


MAX_STEP_C = 5.0     # the plant cannot really move this far inside one second
RESYNC_AFTER = 5     # accept anyway after this many rejects, so we never stall

_previous = [None, None]   # last (time, temperature), used for the slope
_rejected = [0]            # consecutive lines dropped by the jump guard


def parse_line(text):
  fields = text.split()
  if (len(fields) != 9):
    return None                 # a short line is a truncated line, not old firmware
  try:
    values = [float(item) for item in fields]
  except ValueError:
    return None
  (moment, temperature) = (values[0], values[2])
  if (_previous[0] is not None):
    gap = (moment - _previous[0])
    if (((0.0 < gap) and (gap < 1.0))
        and (abs((temperature - _previous[1])) > MAX_STEP_C)
        and (_rejected[0] < RESYNC_AFTER)):
      _rejected[0] += 1
      return None
  _rejected[0] = 0
  rate = 0.0
  if ((_previous[0] is not None) and ((moment - _previous[0]) > 0.0)):
    rate = ((temperature - _previous[1]) / (moment - _previous[0]))
  (_previous[0], _previous[1]) = (moment, temperature)
  return tuple(values + [rate])


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
  log.write("# time_s target_c celsius duty_pct error_c filtered_rate_c_s"
            " p_pct i_pct d_pct rate_c_s\n")

  state = {"buffer": "", "following": True, "whole": False,
           "link": link, "next_connect": 0.0}
  time_s = []
  setpoint_c = []
  measured_c = []
  power_pct = []
  error_c = []
  filtered_rate_c_s = []
  p_pct = []
  i_pct = []
  d_pct = []

  (figure, (top, bottom, errors)) = plt.subplots(3, 1, sharex=True, figsize=(11, 9.5))
  figure.canvas.manager.set_window_title("Classic PID live | %s" % port)
  (dots_setpoint,) = top.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="0.35", label="ramped setpoint")
  (dots_measured,) = top.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:blue", label="NTC temperature")
  (dots_power,) = bottom.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:red", label="actual duty")
  bottom.axhline(0.0, color="0.7", linewidth=0.8)
  top.set_ylabel("Temperature (°C)")
  top.legend(loc="lower right", fontsize=8, ncol=2, framealpha=0.85)
  top.grid(alpha=0.3)
  top.set_title("Waiting for the board ...")
  bottom.set_ylabel("Actual PWM duty (%)")
  bottom.set_ylim(-2.0, 102.0)
  bottom.legend(loc="upper right", fontsize=8, framealpha=0.85)
  bottom.grid(alpha=0.3)
  (line_error,) = errors.plot(
    [], [], linestyle=LINE_STYLE, marker=MARKER, markersize=DOT_SIZE,
    color="tab:green", label="tracking error")
  errors.axhline(0.0, color="0.7", linewidth=0.8)
  errors.set_ylabel("Error (\u00b0C)")
  errors.set_xlabel("Time (s)")
  errors.legend(loc="upper right", fontsize=8, framealpha=0.85)
  errors.grid(alpha=0.3)

  def recent_slope(seconds):
    """Least squares slope of the measurement over the last few seconds."""
    if (len(time_s) < 2):
      return 0.0
    start = bisect.bisect_left(time_s, (time_s[-1] - seconds))
    (xs, ys) = (time_s[start:], measured_c[start:])
    count = len(xs)
    if (count < 2):
      return 0.0
    mean_x = (sum(xs) / count)
    mean_y = (sum(ys) / count)
    spread = sum((((x - mean_x) ** 2) for x in xs))
    if (spread <= 0.0):
      return 0.0
    return (sum((((xs[i] - mean_x) * (ys[i] - mean_y)) for i in range(count))) / spread)

  def draw_title():
    if (len(time_s) == 0):
      top.set_title("Waiting for the board ...")
      return
    tail = ("" if state["following"] else "   ·   frozen (r: live, a: whole run)")
    top.set_title(
      ("SP %.2f °C   ·   T %.2f °C   ·   PWM %.1f %%   ·   %.1f s%s\n"
       "P %+.1f   I %+.1f   D %+.1f %%   ·   slope %+.3f °C/s")
      % (setpoint_c[-1], measured_c[-1], power_pct[-1], time_s[-1], tail,
         p_pct[-1], i_pct[-1], d_pct[-1], recent_slope(SLOPE_WINDOW_S)),
      fontsize=10)

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
    active = state["link"]
    if (active is None):
      if (time.monotonic() >= state["next_connect"]):
        state["next_connect"] = (time.monotonic() + 2.0)
        try:
          state["link"] = serial.Serial(port, BAUD, timeout=0)
          state["buffer"] = ""
          status.set_text("Board reconnected; recording resumed.")
        except serial.SerialException:
          pass
      return ()
    try:
      state["buffer"] += active.read(4096).decode("utf-8", "replace")
    except serial.SerialException:
      active.close()
      state["link"] = None
      status.set_text("USB connection lost; waiting to reconnect.")
      return ()
    while ("\n" in state["buffer"]):
      (raw, state["buffer"]) = state["buffer"].split("\n", 1)
      text = raw.strip()
      row = parse_line(text)
      if (row is None):
        if text.startswith("#"):
          sys.stdout.write(("\n[board] " + text + "\n"))   # acknowledgement
          if text.startswith("# set "):
            status.set_text(("Board confirmed: " + text[6:]))
        continue
      if ((len(time_s) > 0) and (row[0] < time_s[-1])):
        for series in (time_s, setpoint_c, measured_c, power_pct, error_c,
                       filtered_rate_c_s, p_pct, i_pct, d_pct):
          series.clear()
      time_s.append(row[0])
      setpoint_c.append(row[1])
      measured_c.append(row[2])
      power_pct.append(row[3])
      error_c.append(row[4])
      filtered_rate_c_s.append(row[5])
      p_pct.append(row[6])
      i_pct.append(row[7])
      d_pct.append(row[8])
      log.write("%.3f %.2f %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.4f\n" % row)
      sys.stdout.write("\r%8.1f s   %7.3f C   %5.1f %%   P %+8.3f %%   I %+8.3f %%   "
                       % (row[0], row[2], row[3], row[6], row[7]))
    log.flush()
    sys.stdout.flush()
    if (len(time_s) == 0):
      return (dots_setpoint, dots_measured, dots_power, line_error)
    dots_setpoint.set_data(time_s, setpoint_c)
    dots_measured.set_data(time_s, measured_c)
    dots_power.set_data(time_s, power_pct)
    line_error.set_data(time_s, error_c)
    draw_title()
    if (not state["following"]):
      return (dots_setpoint, dots_measured, dots_power, line_error)
    if (state["whole"]):
      start = time_s[0]
    else:
      start = max(time_s[0], (time_s[-1] - arguments.window))
    first = bisect.bisect_left(time_s, start)
    span = max((time_s[-1] - start), 1.0)
    bottom.set_xlim((start - (span * 0.02)),
                    (time_s[-1] + (span * 0.02)))
    rescale(top, (setpoint_c[first:] + measured_c[first:]), 0.25)
    rescale(errors, error_c[first:], 0.5)
    return (dots_setpoint, dots_measured, dots_power, line_error)

  print("Reading %s. Close the window to stop." % (port,))
  print("Keys: space freezes the axes, r goes back to live, a shows the whole run.")
  status = figure.text(0.08, 0.015, "Enter a value, then press Return.", fontsize=9)
  def send_target(text):
    line = text.strip()
    if (not line):
      return
    active = state["link"]
    if (active is None):
      status.set_text("Board disconnected; wait for reconnection.")
      return
    try:
      active.write((line + "\n").encode())
      active.flush()
    except serial.SerialException:
      state["link"] = None
      status.set_text("USB connection lost; waiting to reconnect.")
      return
    sys.stdout.write(("\n[sent ] " + line + "\n"))
    sys.stdout.flush()

  def send_setting(name, raw):
    try:
      value = float(raw.strip())
    except ValueError:
      value = float("nan")
    (low, high) = TUNE_BOUNDS[name]
    if ((not math.isfinite(value)) or (value < low) or (value > high)):
      status.set_text(("Invalid %s; allowed range: %s to %s" % (name, low, high)))
    else:
      send_target(("%s=%s" % (name, raw.strip())))
      status.set_text(("Sent %s=%s; waiting for board reply" % (name, raw.strip())))
    figure.canvas.draw_idle()

  figure.subplots_adjust(left=0.09, right=0.97, top=0.90, bottom=0.27, hspace=0.35)
  figure.text(0.08, 0.225, "TUNE  •  press Return to send a precise value", fontsize=10)
  boxes = []
  for (index, (name, initial)) in enumerate(TUNABLES):
    (row, column) = (index // 4, index % 4)
    axes = figure.add_axes([(0.17 + (column * 0.225)), (0.16 - (row * 0.075)), 0.115, 0.04])
    box = TextBox(axes, (name + "  "), initial=initial)
    box.on_submit(lambda raw, key=name: send_setting(key, raw))
    boxes.append(box)
  off_button = Button(figure.add_axes([0.82, 0.012, 0.13, 0.04]), "OUTPUT OFF")
  off_button.on_clicked(lambda _event: send_target("off"))
  animation = FuncAnimation(figure, update, interval=REFRESH_MS, cache_frame_data=False)
  plt.show()
  if (state["link"] is not None):
    state["link"].close()
  log.close()
  print("\nsaved %s, %d rows" % (log_path, len(time_s)))
  return (animation, boxes, off_button)


if (__name__ == "__main__"):
  main()
