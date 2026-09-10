"""Drive the plate through a target sequence and log the whole run."""
import glob
import sys
import time

import serial

PORT = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
STAGES = [(90.0, "up"), (60.0, "down"), (120.0, "up")]
TOLERANCE_C = 1.5
HOLD_S = 30.0
ABORT_C = 130.0
STAGE_LIMIT_S = 2400.0
VALID_RANGE = (5.0, 200.0)


def reached(celsius, target, way):
    if (way == "up"):
        return (celsius >= (target - TOLERANCE_C))
    return (celsius <= (target + TOLERANCE_C))


def note(text):
    stamp = time.strftime("%H:%M:%S")
    line = ("%s  %s" % (stamp, text))
    print(line)
    sys.stdout.flush()
    sink.write(("# " + line + "\n"))
    sink.flush()


path = time.strftime("sequence_%Y%m%d_%H%M%S.txt")
sink = open(path, "w")
sink.write("# time_s  setpoint_c  celsius  duty_pct\n")

link = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(2.0)
link.reset_input_buffer()
note("started, log %s" % (path,))

buffer = ""
index = 0
(target, way) = STAGES[0]
link.write(("%.3f\n" % (target,)).encode())
link.flush()
note("stage 1/%d: target %.1f C (%s)" % (len(STAGES), target, way))
(stage_start, inside_since, last_report) = (time.monotonic(), None, 0.0)
latest = None

try:
    while True:
        buffer += link.read(4096).decode("utf-8", "replace")
        while ("\n" in buffer):
            (raw, buffer) = buffer.split("\n", 1)
            parts = raw.split()
            if (len(parts) != 4):
                continue
            try:
                values = [float(part) for part in parts]
            except ValueError:
                continue
            celsius = values[2]
            if (not (VALID_RANGE[0] <= celsius <= VALID_RANGE[1])):
                continue
            latest = celsius
            sink.write("%.3f %.2f %.2f %.1f\n" % tuple(values))
            sink.flush()

        now = time.monotonic()
        if (latest is None):
            continue
        if (latest > ABORT_C):
            link.write(b"0.0\n")
            link.flush()
            note("ABORT at %.2f C, limit %.1f C" % (latest, ABORT_C))
            break
        if ((now - last_report) >= 60.0):
            note("stage %d, target %.1f C, now %.2f C, %.0f s in stage"
                 % ((index + 1), target, latest, (now - stage_start)))
            last_report = now
        if reached(latest, target, way):
            if (inside_since is None):
                inside_since = now
            elif ((now - inside_since) >= HOLD_S):
                note("stage %d done at %.2f C after %.0f s"
                     % ((index + 1), latest, (now - stage_start)))
                index += 1
                if (index >= len(STAGES)):
                    note("sequence complete")
                    break
                (target, way) = STAGES[index]
                link.write(("%.3f\n" % (target,)).encode())
                link.flush()
                note("stage %d/%d: target %.1f C (%s)"
                     % ((index + 1), len(STAGES), target, way))
                (stage_start, inside_since) = (now, None)
        else:
            inside_since = None
        if ((now - stage_start) > STAGE_LIMIT_S):
            note("stage %d timed out at %.2f C" % ((index + 1), latest))
            break
finally:
    link.close()
    sink.close()
    note("closed")
