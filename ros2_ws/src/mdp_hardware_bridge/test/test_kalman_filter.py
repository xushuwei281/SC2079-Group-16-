import math
import unittest

from mdp_hardware_bridge.kalman_filter import KalmanFilter1D, PoseKalmanFilter


class TestKalmanFilter1D(unittest.TestCase):
    def test_initialization(self):
        kf = KalmanFilter1D(q=1e-3, r=1e-2, min_val=0.02, max_val=4.0)
        self.assertFalse(kf.is_initialized)
        self.assertEqual(kf.value, float("inf"))

        val = kf.update(0.50)
        self.assertTrue(kf.is_initialized)
        self.assertAlmostEqual(val, 0.50, places=3)
        self.assertAlmostEqual(kf.value, 0.50, places=3)

    def test_smoothing_noise(self):
        kf = KalmanFilter1D(q=1e-3, r=1e-2)
        kf.update(1.0)
        # Alternate around 1.0 with noise: 1.02, 0.98, 1.03, 0.97
        readings = [1.02, 0.98, 1.03, 0.97, 1.01, 0.99]
        for z in readings:
            filtered = kf.update(z)
            # Filtered value should stay close to 1.0 and vary less than raw noise
            self.assertTrue(0.985 <= filtered <= 1.015)

    def test_single_spike_rejected(self):
        kf = KalmanFilter1D(q=1e-3, r=1e-2, outlier_threshold=0.30, max_consecutive_outliers=2)
        kf.update(0.80)
        kf.update(0.80)

        # Single spike: drops suddenly to 0.25 (55cm jump, like our Sharp IR probe)
        val_spike = kf.update(0.25)
        # Should reject the spike and remain near 0.80
        self.assertTrue(val_spike >= 0.75, f"Expected filtered >= 0.75, got {val_spike}")

        # Returns to 0.80 on next reading
        val_normal = kf.update(0.80)
        self.assertAlmostEqual(val_normal, 0.80, places=2)

    def test_sustained_change_adopted(self):
        kf = KalmanFilter1D(q=1e-3, r=1e-2, outlier_threshold=0.30, max_consecutive_outliers=2)
        kf.update(0.80)

        # First outlier: rejected
        kf.update(0.25)
        # Second consecutive outlier: adopted as genuine obstacle
        val2 = kf.update(0.25)
        self.assertAlmostEqual(val2, 0.25, places=2)

    def test_invalid_readings_and_coasting(self):
        kf = KalmanFilter1D(q=1e-3, r=1e-2, min_val=0.02, max_val=3.0)
        kf.update(1.50)

        # 1 invalid reading coasts on last state
        coasted = kf.update(float("inf"))
        self.assertAlmostEqual(coasted, 1.50, places=2)

        # 2nd invalid reading drops to inf
        invalid = kf.update(float("inf"))
        self.assertEqual(invalid, float("inf"))
        self.assertFalse(kf.is_initialized)


class TestPoseKalmanFilter(unittest.TestCase):
    def test_initialization(self):
        kf = PoseKalmanFilter(initial_x=0.20, initial_y=0.20, initial_yaw=math.pi / 2.0)
        self.assertTrue(kf.is_initialized)
        self.assertAlmostEqual(kf.x, 0.20, places=3)
        self.assertAlmostEqual(kf.y, 0.20, places=3)
        self.assertAlmostEqual(kf.yaw, math.pi / 2.0, places=3)

    def test_lazy_initialization(self):
        kf = PoseKalmanFilter()
        self.assertFalse(kf.is_initialized)
        x, y, yaw = kf.update(0.35, 0.45, 1.0)
        self.assertTrue(kf.is_initialized)
        self.assertAlmostEqual(x, 0.35, places=3)
        self.assertAlmostEqual(y, 0.45, places=3)
        self.assertAlmostEqual(yaw, 1.0, places=3)

    def test_position_smoothing(self):
        kf = PoseKalmanFilter(
            q_pos=5e-4, r_pos=2e-3, initial_x=0.50, initial_y=0.50, initial_yaw=0.0
        )
        # Jitter around 0.50
        readings = [(0.52, 0.48), (0.49, 0.51), (0.51, 0.49)]
        for rx, ry in readings:
            fx, fy, _ = kf.update(rx, ry, 0.0)
            self.assertTrue(0.485 <= fx <= 0.515)
            self.assertTrue(0.485 <= fy <= 0.515)

    def test_yaw_circular_wrapping(self):
        # Robot heading near +pi (+179 deg = +3.124 rad)
        # Next measurement wraps to -pi (-179 deg = -3.124 rad)
        initial_yaw = math.radians(179.0)
        kf = PoseKalmanFilter(
            q_yaw=1e-3, r_yaw=1e-2, initial_x=0.0, initial_y=0.0, initial_yaw=initial_yaw
        )

        # Measure 181 deg = -179 deg (-3.124 rad)
        meas_yaw = math.radians(-179.0)
        _, _, fyaw = kf.update(0.0, 0.0, meas_yaw)

        # True angular difference is only 2 degrees. The filter must NOT jump through 0 deg!
        # Filtered yaw should be near 180 deg / -180 deg (magnitude close to pi)
        self.assertAlmostEqual(abs(fyaw), math.pi, delta=0.05)

    def test_outlier_jump_rejection(self):
        kf = PoseKalmanFilter(
            outlier_dist_threshold=0.50,
            max_consecutive_outliers=2,
            initial_x=0.20,
            initial_y=0.20,
            initial_yaw=0.0,
        )
        # Sudden teleop/noise glitch: 10m jump in 1 frame
        fx, fy, _ = kf.update(10.20, 10.20, 0.0)
        self.assertAlmostEqual(fx, 0.20, places=2)
        self.assertAlmostEqual(fy, 0.20, places=2)

    def test_reset_and_set_state(self):
        kf = PoseKalmanFilter(initial_x=0.0, initial_y=0.0, initial_yaw=0.0)
        kf.update(0.10, 0.10, 0.10)

        kf.reset(0.20, 0.20, math.pi / 2.0)
        self.assertAlmostEqual(kf.x, 0.20, places=3)
        self.assertAlmostEqual(kf.y, 0.20, places=3)
        self.assertAlmostEqual(kf.yaw, math.pi / 2.0, places=3)


if __name__ == "__main__":
    unittest.main()
