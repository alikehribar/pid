"""Checks for the filtered classic PID controller."""

import math
import unittest

import sys
from pathlib import Path

sys.path.insert(0, str((Path(__file__).resolve().parents[1] / "firmware")))
from classic_pid import Controller, MAX_DUTY


class ClassicPidTests(unittest.TestCase):
    def test_boot_target_and_fault_safety(self):
        control = Controller()
        self.assertFalse(control.set_target(70.0))
        self.assertEqual(control.step(27.0, 0.2)[2], 0.0)
        self.assertTrue(control.set_target(70.0))
        self.assertIsNone(control.step(float("nan"), 0.2))
        self.assertEqual(control.duty, 0.0)
        self.assertIsNone(control.target)

    def test_median_filter_rejects_one_sample_spike(self):
        control = Controller()
        for _ in range(5):
            control.step(40.0, 0.2)
        row = control.step(60.0, 0.2)
        self.assertEqual(row[1], 40.0)
        self.assertEqual(control.rate, 0.0)

    def test_integral_does_not_wind_up_at_full_output(self):
        control = Controller()
        control.ramp_rate = 1000.0
        control.step(27.0, 0.2)
        self.assertTrue(control.set_target(100.0))
        for _ in range(100):
            row = control.step(27.0, 0.2)
        self.assertEqual(row[2], MAX_DUTY)
        self.assertEqual(control.integral, 0.0)

    def test_setpoint_ramps_at_configured_rate(self):
        control = Controller()
        control.step(27.0, 0.2)
        self.assertTrue(control.set_target(60.0))
        row = control.step(27.0, 1.0)
        self.assertAlmostEqual(row[0], 27.2)

    def test_characterized_plant_reaches_target(self):
        control = Controller()
        (heater, sensor, peak) = (27.0, 27.0, 27.0)
        dt = 0.2
        control.step(sensor, dt)
        self.assertTrue(control.set_target(70.0))
        for _ in range(int((3600.0 / dt))):
            duty = control.step(sensor, dt)[2]
            equilibrium = (27.0 + (0.825 * duty))
            heater += ((equilibrium - heater)
                       * (1.0 - math.exp(((-dt) / 275.94))))
            sensor += ((heater - sensor)
                       * (1.0 - math.exp(((-dt) / 46.92))))
            peak = max(peak, sensor)
        self.assertLess(peak, 72.0)
        self.assertLess(abs((sensor - 70.0)), 0.1)


if (__name__ == "__main__"):
    unittest.main()
