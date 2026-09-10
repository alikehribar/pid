import sys
import time

import serial

from ntc import find_port, resistance_to_celsius, voltage_to_resistance

link = serial.Serial(find_port(), 115200, timeout=1)
link.reset_input_buffer()
path = time.strftime("step_%Y%m%d_%H%M%S.txt")
buffer = ""

with open(path, "w") as log:
    log.write("# time_s  celsius  duty_pct\n")
    print("Logging to %s. Ctrl-C to stop." % (path,))
    while True:
        buffer += link.read(256).decode("utf-8", "replace")
        while ("\n" in buffer):
            (raw, buffer) = buffer.split("\n", 1)
            parts = raw.split()
            if (len(parts) != 3):
                continue
            try:
                (seconds, voltage, duty) = (float(parts[0]), float(parts[1]), float(parts[2]))
            except ValueError:
                continue
            resistance = voltage_to_resistance(voltage)
            if (resistance is None):
                continue
            celsius = resistance_to_celsius(resistance)
            log.write("%.3f %.4f %.1f\n" % (seconds, celsius, duty))
            log.flush()
            sys.stdout.write("\r%8.1f s   %7.3f C   %5.1f %%" % (seconds, celsius, duty))
            sys.stdout.flush()
