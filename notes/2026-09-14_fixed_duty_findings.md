# Fixed-duty characterization, 2026-09-14

The reported supply voltage was 12.4 V. With a nominal 10-ohm resistor, the
estimated average electrical input is 0.769 W at 5% PWM, 1.538 W at 10%, and
2.306 W at 15%. These values assume the voltage stayed constant and the load
was switched cleanly. The resistor's 5 W rating depends on its mounting and
ambient conditions.

## Recorded transitions

| PWM transition | Board time (s) | NTC at transition (C) | Observation |
| --- | ---: | ---: | --- |
| 0% to 5% | 24.211 | 25.6624 | Heating begins. |
| 5% to 10% | 986.484 | 49.9280 | The 5% segment was still rising slowly. |
| 10% to 15% | 3389.789 | 76.0265 | A separate CSV contains the later 10% segment. |
| 15% to 0% | 5434.704 | 98.7742 | Cooling accelerates; no post-cut temperature rise was observed. |

The 15% segment reached 99.6789 C at board time 5193.274 s. This was
241.430 s before the 0% command; temperature fell 0.9047 C before cutoff.
After the 0% command the NTC was 98.6436 C at 10 s, 92.7723 C at 60 s, and
54.6037 C at 300 s. The first 0% sample was the maximum of the cooldown.
No 20% segment appears in these files.

## Data quality and interpretation

The long CSV files contain 10,457 and 26,796 complete rows. Within each file,
board timestamps are strictly increasing, sample spacing is about 0.2 s, and
no NTC values are missing. The files have a 208.783 s recording gap between
board times 2092.871 s and 2301.654 s. Input and temperature during that gap
are unknown, so the files were modeled as separate initial conditions.

The final-minute temperature slopes were +0.103 C/min at 5%, -0.080 C/min
at the later 10% segment, and -0.386 C/min at 15%. Consequently, these
terminal temperatures are useful operating points, not exact equilibrium
temperatures. The 5% terminal value alone does not prove a 49.94 C setpoint.

After removing the linear trend in each final minute, raw temperature residual
standard deviations were about 0.022 C at 5%, 0.014 C at the later 10%, and
0.029 C at 15%. The plot's five-sample median can appear quieter than the
raw CSV. Display variation does not establish absolute NTC accuracy.

## First predictive model and validation

`tools/fit_fixed_duty.py` fits a two-lag effective heater/sensor model to the
heating segments and reserves the 15% to 0% cooldown for validation. The
fitted gain was 4.852 C per duty percentage point; the two fitted time
constants were 281.5 s and 30.4 s. These are effective model values, not
direct measurements of the ceramic resistor's internal temperature.

Heating fit RMSE was 0.670 C. On the unseen cooldown, the first five minutes
had 1.046 C RMSE. The model predicted 99.397 C at cutoff whereas the NTC read
98.774 C. This error is substantially larger than a 0.05 C overshoot target.
The model should not be used yet as a precision cutoff controller.

Run the repeatable fit from the repository root with
`python3 tools/fit_fixed_duty.py`. The measured trace is in
`figures/fixed_duty_trace_2026-09-14.png`.

## Later rising-cutoff measurement

`data/2026-09-14_fixed_duty_1x10R/1719_40-15-0pct_early_cut.csv` records about 82.5 s at 40% PWM, followed
by 53.2 s at 15% PWM, then 0%. The NTC read 74.9189 C at the first 0% row
(board time 8542.666 s). It reached 79.0090 C at 8580.850 s: an additional
4.0901 C after 38.184 s. A five-sample median gives a 4.0871 C rise, so the
peak is not an isolated sample glitch. The last 10 s before cutoff rose at
about 0.245 C/s. The record ends 59.514 s after cutoff, after temperature
had already fallen 0.806 C from the peak. It captures the peak but not the
full cooldown.

At 12.4 V and nominal 10 ohms, 40% PWM implies about 6.15 W average input,
above the stated 5 W resistor rating. The earlier model was fitted only on
0-15% input. This mixed 40%/15% test demonstrates stored-heat overshoot, but
it is not a clean validation of a 15%-only approach or a safe basis for
extrapolating the model to higher duty. See `figures/early_cut_2026-09-14.png`.

## Next validation measurement

For a predictive braking test, start from a cooled assembly, hold 15% while
the NTC is clearly rising, then command 0% at a chosen temperature below the
desired peak. Keep the board and logger powered while recording the resulting
peak and cooldown. This measures stored-heat overshoot during approach, which
the first cooldown did not test because the NTC was already cooling. The
later rising-cutoff test confirms overshoot but has a 40% preheat stage, so it
does not isolate the response to 15% alone.
Repeat at a second cutoff temperature to check whether the behavior is
reproducible. A later partial reduction such as 15% to 10% would test braking
without fully removing power. Do not extrapolate this dataset to 120-140 C.
