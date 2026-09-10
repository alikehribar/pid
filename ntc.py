import glob
import math

V_SUPPLY = 3.3
R_FIXED = 10000.0
R0 = 10000.0
BETA = 3950.0
T0_KELVIN = 298.15


def find_port():
  candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
  if (len(candidates) == 0):
    raise SystemExit("No /dev/cu.usbmodem* port found. Plug the Pico in, or pass the port explicitly.")
  return candidates[0]


def parse_line(text):
  parts = text.split()
  if (len(parts) != 2):
    return None
  try:
    return (float(parts[0]), float(parts[1]))
  except ValueError:
    return None


def voltage_to_resistance(voltage):
  ratio = (voltage / V_SUPPLY)
  if ((ratio <= 0.0) or (ratio >= 1.0)):
    return None
  return (R_FIXED * (ratio / (1.0 - ratio)))


def resistance_to_celsius(resistance):
  kelvin = (1.0 / ((1.0 / T0_KELVIN) + (math.log((resistance / R0)) / BETA)))
  return (kelvin - 273.15)
