import sys

import numpy as np
import matplotlib.pyplot as plt

SP_RATE = 0.2
SP_TAU = 5.0
SP_TARGET = 90.0
KP = 0.08175
H_PREDICT = 18.0
TAU_D = 8.0
T_SPIKE = 3.0
U_SPIKE = 25.0


def drop_spikes(time_s, celsius, duty):
  """Remove single-sample glitches by comparing each point to its 5-point median."""
  keep = np.ones(len(time_s), dtype=bool)
  for (values, limit) in ((celsius, T_SPIKE), (duty, U_SPIKE)):
    median = np.array([np.median(values[max(0, (i - 2)):(i + 3)]) for i in range(len(values))])
    keep &= (np.abs((values - median)) <= limit)
  return (keep, int((~keep).sum()))


def rebuild_setpoint(time_s, celsius):
  """The log stores no setpoint column, so replay the firmware reference filter."""
  (sp_cmd, sp_filt) = (celsius[0], celsius[0])
  out = np.zeros(len(time_s))
  for i in range(len(time_s)):
    dt = ((time_s[i] - time_s[(i - 1)]) if (i > 0) else 0.0)
    step = (SP_RATE * dt)
    sp_cmd += max((-step), min(step, (SP_TARGET - sp_cmd)))
    sp_filt += (((sp_cmd - sp_filt) * dt) / SP_TAU)
    out[i] = sp_filt
  return out


def filtered_rate(time_s, values):
  """Same first-order rate filter the controller runs (TAU_D)."""
  out = np.zeros(len(values))
  rate = 0.0
  for i in range(1, len(values)):
    dt = (time_s[i] - time_s[(i - 1)])
    if (dt <= 0.0):
      continue
    rate += ((((values[i] - values[(i - 1)]) / dt) - rate) * dt / TAU_D)
    out[i] = rate
  return out


def main():
  path = (sys.argv[1] if (len(sys.argv) > 1) else "live_20260909_200327.txt")
  data = np.loadtxt(path)
  (time_s, celsius, duty) = (data[:, 0], data[:, 1], data[:, 2])
  (keep, dropped) = drop_spikes(time_s, celsius, duty)
  (time_s, celsius, duty) = (time_s[keep], celsius[keep], duty[keep])

  setpoint = rebuild_setpoint(time_s, celsius)
  pv_rate = filtered_rate(time_s, celsius)
  sp_rate = filtered_rate(time_s, setpoint)
  error = (setpoint - celsius)
  ctrl_error = (error + (H_PREDICT * (sp_rate - pv_rate)))

  peak = int(np.argmax(celsius))
  above = np.where((celsius >= SP_TARGET))[0]
  cross = (int(above[0]) if (len(above) > 0) else None)

  (figure, axes) = plt.subplots(4, 1, sharex=True, figsize=(13, 12))
  figure.suptitle(
    "%s   ·   KP %.5f   ·   H %.0f s   ·   TAU_D %.0f s   ·   %d samples, %.0f s   ·   %d spikes removed"
    % (path, KP, H_PREDICT, TAU_D, len(time_s), time_s[-1], dropped), fontsize=11)

  top = axes[0]
  top.plot(time_s, setpoint, ".", ms=3, color="0.6", label="setpoint (rebuilt)")
  top.plot(time_s, celsius, ".", ms=3, color="tab:blue", label="measured")
  top.axhline(SP_TARGET, color="tab:orange", lw=0.9, ls="--", label="target %.0f °C" % (SP_TARGET,))
  top.annotate("peak %.2f °C\novershoot %+.2f °C" % (celsius[peak], (celsius[peak] - SP_TARGET)),
               (time_s[peak], celsius[peak]), textcoords="offset points", xytext=(10, -28),
               fontsize=9, arrowprops={"arrowstyle": "->", "color": "tab:red"})
  if (cross is not None):
    top.axvline(time_s[cross], color="tab:red", lw=0.8, alpha=0.5)
  top.set_ylabel("Temperature (°C)")
  top.legend(loc="lower right", fontsize=9)
  top.grid(alpha=0.3)

  power = axes[1]
  power.plot(time_s, duty, ".", ms=3, color="tab:red", label="duty")
  power.plot(time_s, (KP * ctrl_error * 100.0), ".", ms=2.5, color="tab:orange",
             label="P+D part = KP × ctrl error")
  power.plot(time_s, (duty - (KP * ctrl_error * 100.0)), ".", ms=2.5, color="tab:purple",
             label="integral part (remainder)")
  power.axhline(0.0, color="0.7", lw=0.8)
  if (cross is not None):
    power.axvline(time_s[cross], color="tab:red", lw=0.8, alpha=0.5)
  power.set_ylabel("Power (%)")
  power.legend(loc="upper left", fontsize=9, ncol=3)
  power.grid(alpha=0.3)

  err = axes[2]
  err.plot(time_s, error, ".", ms=3, color="tab:green", label="real error (sp − T)")
  err.plot(time_s, ctrl_error, ".", ms=3, color="tab:purple",
           label="controller error (+ %.0f s prediction)" % (H_PREDICT,))
  err.fill_between(time_s, error, ctrl_error, color="tab:purple", alpha=0.15,
                   label="D term = H × (sp rate − T rate)")
  err.axhline(0.0, color="0.7", lw=0.8)
  err.set_ylabel("Error (°C)")
  err.legend(loc="lower left", fontsize=9)
  err.grid(alpha=0.3)

  spd = axes[3]
  spd.plot(time_s, pv_rate, ".", ms=3, color="tab:blue", label="T rate (filtered)")
  spd.plot(time_s, sp_rate, ".", ms=3, color="0.6", label="setpoint rate")
  spd.axhline(0.0, color="0.7", lw=0.8)
  spd.set_ylabel("Rate (°C/s)")
  spd.set_xlabel("Time (s)")
  spd.legend(loc="upper right", fontsize=9)
  spd.grid(alpha=0.3)

  plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
  out_path = (path.rsplit(".", 1)[0] + ".png")
  plt.savefig(out_path, dpi=140)
  print("peak %.2f °C, overshoot %+.2f °C at %.1f s" % (celsius[peak], (celsius[peak] - SP_TARGET), time_s[peak]))
  if (cross is not None):
    print("crossed %.0f °C at %.1f s" % (SP_TARGET, time_s[cross]))
  print("removed %d spike samples, saved %s" % (dropped, out_path))


if (__name__ == "__main__"):
  main()
