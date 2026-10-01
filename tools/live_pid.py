import glob
import threading
import time

import matplotlib.pyplot as plt
import serial


def send_typed_commands(link):
    while True:
        link.write((input() + "\n").encode())


def main():
    port = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
    link = serial.Serial(port, 115200, timeout=0.1)
    log = open(time.strftime("live_%Y%m%d_%H%M%S.txt"), "w")
    log.write("# time_s target_c celsius duty_pct error_c filtered_rate_c_s"
              " p_pct i_pct d_pct\n")
    threading.Thread(target=send_typed_commands, args=(link,), daemon=True).start()
    print("type a target like 60, or kp=..., ki=..., kd=..., off, then press Return")
    (times, targets, temperatures, duties) = ([], [], [], [])
    plt.ion()
    (figure, (top, bottom)) = plt.subplots(2, 1, sharex=True)
    try:
        while plt.fignum_exists(figure.number):
            line = link.readline().decode("utf-8", "replace").strip()
            if line.startswith("#"):
                print(line)
                continue
            try:
                values = [float(field) for field in line.split()]
            except ValueError:
                continue
            if (len(values) != 9):
                continue
            log.write(line + "\n")
            times.append(values[0])
            targets.append(values[1])
            temperatures.append(values[2])
            duties.append(values[3])
            if ((len(times) % 5) == 0):
                top.clear()
                bottom.clear()
                top.plot(times, targets, "--", times, temperatures)
                bottom.plot(times, duties)
                top.set_ylabel("Temperature (C)")
                bottom.set_ylabel("PWM (%)")
                bottom.set_xlabel("Time (s)")
                plt.pause(0.01)
    finally:
        link.write(b"off\n")
        log.close()


if (__name__ == "__main__"):
    main()
