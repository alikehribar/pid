"""Checks for the Marlin-style relay autotuner."""

import math
import unittest

import sys
from pathlib import Path

sys.path.insert(0, str((Path(__file__).resolve().parents[1] / "tools")))
from relay_autotune import RelayAutotuner, summarize


class RelayAutotunerTests(unittest.TestCase):
    def test_completes_on_characterized_plant(self):
        tuner = RelayAutotuner(70.0, 7)
        (heater, sensor, dt) = (27.0, 27.0, 0.2)
        tuner.start(0.0, sensor)
        for index in range(int((3600.0 / dt))):
            equilibrium = (27.0 + (0.825 * tuner.command))
            heater += ((equilibrium - heater)
                       * (1.0 - math.exp(((-dt) / 275.94))))
            sensor += ((heater - sensor)
                       * (1.0 - math.exp(((-dt) / 46.92))))
            tuner.step(((index + 1) * dt), sensor)
            if tuner.finished:
                break
        self.assertEqual(tuner.reason, "completed")
        self.assertGreaterEqual(len(tuner.results), 3)
        result = summarize(tuner.results)
        self.assertIsNotNone(result)
        self.assertGreater(result[0], 0.0)
        self.assertGreater(result[1], 0.0)

    def test_safety_cutoff_sets_zero_output(self):
        tuner = RelayAutotuner(70.0, 7)
        tuner.start(0.0, 27.0)
        (command, event) = tuner.step(10.0, 90.0)
        self.assertEqual(event, "safety")
        self.assertEqual(command, 0.0)
        self.assertTrue(tuner.finished)


if (__name__ == "__main__"):
    unittest.main()
