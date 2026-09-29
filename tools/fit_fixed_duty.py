"""Fit a two-lag thermal response and validate it on the held-out cooldown."""

import numpy as np
from scipy.optimize import least_squares


def response(seconds, heater, sensor, duty, ambient, gain, tau_heater, tau_sensor):
    heater_decay = np.exp(((-seconds) / tau_heater))
    sensor_decay = np.exp(((-seconds) / tau_sensor))
    equilibrium = (ambient + (gain * duty))
    next_heater = (equilibrium + ((heater - equilibrium) * heater_decay))
    coupling = ((heater - equilibrium) * (tau_heater / (tau_heater - tau_sensor)))
    next_sensor = (equilibrium + ((sensor - equilibrium) * sensor_decay)
                   + (coupling * (heater_decay - sensor_decay)))
    return (next_heater, next_sensor)


def load_runs(path, stride=5):
    data = np.genfromtxt(path, delimiter=",", names=True)
    times = data["board_elapsed_s"]
    duty = data["duty_pct"]
    temperature = data["temperature_c"]
    edges = np.r_[0, (np.flatnonzero((np.diff(duty) != 0.0)) + 1), len(data)]
    runs = []
    for lower, upper in zip(edges[:-1], edges[1:]):
        selected = np.arange(lower, upper, stride)
        if (selected[-1] != (upper - 1)):
            selected = np.r_[selected, (upper - 1)]
        runs.append((duty[lower], times[lower], times[(upper - 1)],
                     (times[selected] - times[lower]), temperature[selected], temperature[lower]))
    return runs


def predict_runs(runs, heater, sensor, ambient, gain, tau_heater, tau_sensor):
    predictions = []
    last_end = runs[0][1]
    last_duty = runs[0][0]
    for duty, start, end, relative, _observed, _first in runs:
        gap = (start - last_end)
        (heater, sensor) = response(gap, heater, sensor, last_duty, ambient,
                                    gain, tau_heater, tau_sensor)
        (_estimated_heater, predicted) = response(relative, heater, sensor, duty,
                                                  ambient, gain, tau_heater, tau_sensor)
        predictions.append(predicted)
        (heater, sensor) = response((end - start), heater, sensor, duty, ambient,
                                    gain, tau_heater, tau_sensor)
        (last_end, last_duty) = (end, duty)
    return (predictions, heater, sensor)


def fit_residual(parameters, first, second, ambient_first):
    (gain, tau_heater, tau_sensor, ambient_second, heater_second) = parameters
    initial_first = first[0][5]
    (predicted_first, _, _) = predict_runs(first, initial_first, initial_first,
                                           ambient_first, gain, tau_heater, tau_sensor)
    initial_second = second[0][5]
    (predicted_second, _, _) = predict_runs(second, heater_second, initial_second,
                                            ambient_second, gain, tau_heater, tau_sensor)
    errors = [(predicted - run[4]) for predicted, run in
              zip((predicted_first + predicted_second), (first + second))]
    return np.concatenate(errors)


def main():
    first = load_runs("data/2026-09-14_fixed_duty_1x10R/1459_0-5-10pct.csv")
    second = load_runs("data/2026-09-14_fixed_duty_1x10R/1538_10-15-0pct.csv")
    training_second = second[:-1]
    ambient_first = float(np.median(first[0][4]))
    initial = [5.0, 350.0, 20.0, 27.0, 76.0]
    lower = [2.0, 120.0, 1.0, 23.0, 70.0]
    upper = [8.0, 1500.0, 100.0, 31.0, 85.0]
    fitted = least_squares(fit_residual, initial, bounds=(lower, upper),
                           args=(first, training_second, ambient_first), max_nfev=200)
    (gain, tau_heater, tau_sensor, ambient_second, heater_second) = fitted.x
    print("gain_C_per_pct, tau_heater_s, tau_sensor_s, ambient_second_C, heater_start_C")
    print(*[round(float(value), 4) for value in fitted.x])
    print("training_RMSE_C", round(float(np.sqrt(np.mean((fitted.fun ** 2)))), 4))
    training_runs = (first + training_second)
    offset = 0
    for index, run in enumerate(training_runs):
        length = len(run[4])
        portion = fitted.fun[offset:(offset + length)]
        print("heating_run", index, "duty_pct", run[0], "RMSE_C",
              round(float(np.sqrt(np.mean((portion ** 2)))), 4), "end_error_C",
              round(float(portion[-1]), 4))
        offset = (offset + length)
    (_, heater_at_end, sensor_at_end) = predict_runs(training_second, heater_second,
        training_second[0][5], ambient_second, gain, tau_heater, tau_sensor)
    cooldown = second[-1]
    gap = (cooldown[1] - training_second[-1][2])
    (heater_at_cut, sensor_at_cut) = response(gap, heater_at_end, sensor_at_end,
        training_second[-1][0], ambient_second, gain, tau_heater, tau_sensor)
    (_, forecast) = response(cooldown[3], heater_at_cut, sensor_at_cut, 0.0,
                             ambient_second, gain, tau_heater, tau_sensor)
    predicted_peak = int(np.argmax(forecast))
    print("predicted_cooldown_peak_C_at_s", round(float(forecast[predicted_peak]), 4),
          round(float(cooldown[3][predicted_peak]), 3))
    print("cooldown_start_observed_predicted_C", round(float(cooldown[4][0]), 4),
          round(float(forecast[0]), 4))
    for horizon in (30, 60, 300, 600, 1800):
        mask = (cooldown[3] <= horizon)
        error = (forecast[mask] - cooldown[4][mask])
        print("cooldown_RMSE_C_at_s", horizon, round(float(np.sqrt(np.mean((error ** 2)))), 4))
    (_, anchored) = response(cooldown[3], heater_at_cut, cooldown[4][0], 0.0,
                             ambient_second, gain, tau_heater, tau_sensor)
    for horizon in (60, 300, 600):
        mask = (cooldown[3] <= horizon)
        error = (anchored[mask] - cooldown[4][mask])
        print("anchored_cooldown_RMSE_C_at_s", horizon,
              round(float(np.sqrt(np.mean((error ** 2)))), 4))


if (__name__ == "__main__"):
    main()
