#!/usr/bin/env python3
"""Host-side contract checks for the low-side shunt correction."""

import math
import unittest


R_SHUNT_OHM = 10.0


def correct_s(s_raw_v, current_ma):
    if not math.isfinite(s_raw_v) or not math.isfinite(current_ma) or current_ma < 0:
        return s_raw_v, 0.0, False
    drop_v = current_ma * R_SHUNT_OHM / 1000.0
    s_board_v = s_raw_v - drop_v
    if not math.isfinite(drop_v) or not math.isfinite(s_board_v):
        return s_raw_v, 0.0, False
    return s_board_v, drop_v, True


class ShuntCorrectionTest(unittest.TestCase):
    def test_zero_current(self):
        self.assertEqual(correct_s(1.234, 0.0), (1.234, 0.0, True))

    def test_29_ma_through_10_ohm(self):
        board, drop, valid = correct_s(0.726, 29.0)
        self.assertTrue(valid)
        self.assertAlmostEqual(drop, 0.290)
        self.assertAlmostEqual(board, 0.436)

    def test_real_wiring_sign_from_supply_dataset(self):
        board, drop, valid = correct_s(0.726, 29.37)
        self.assertTrue(valid)
        self.assertAlmostEqual(drop, 0.2937)
        self.assertAlmostEqual(board, 0.4323)

    def test_invalid_current_falls_back_to_raw(self):
        for current in (float("nan"), float("inf"), -1.0):
            board, drop, valid = correct_s(1.0, current)
            self.assertFalse(valid)
            self.assertEqual(board, 1.0)
            self.assertEqual(drop, 0.0)

    def test_invalid_raw_is_flagged(self):
        board, drop, valid = correct_s(float("nan"), 10.0)
        self.assertFalse(valid)
        self.assertTrue(math.isnan(board))
        self.assertEqual(drop, 0.0)


if __name__ == "__main__":
    unittest.main()
