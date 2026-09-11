"""Kalman filter implementations for telemetry smoothing on the Raspberry Pi.

Provides:
- KalmanFilter1D: 1D linear Kalman filter with outlier/spike rejection for
  rangefinders (ultrasonic HC-SR04, Sharp IR sensors).
- PoseKalmanFilter: 3-DoF Kalman filter for 2D robot pose (x, y, yaw) with
  continuous circular angle wrapping in (-pi, pi] and position gating.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


class KalmanFilter1D:
    """1D linear Kalman filter with innovation-based outlier/glitch rejection.

    Suitable for distance sensors like HC-SR04 ultrasonic and Sharp analog IR.
    Single-sample transient reflections or spikes are gated unless confirmed by
    consecutive readings, preventing false collision stops.
    """

    def __init__(
        self,
        q: float = 1e-3,
        r: float = 1e-2,
        outlier_threshold: float = 0.30,
        max_consecutive_outliers: int = 2,
        min_val: float = 0.02,
        max_val: float = 4.0,
    ) -> None:
        """Initialize the 1D Kalman filter.

        Args:
            q: Process noise covariance (higher = tracks faster, more jitter).
            r: Measurement noise covariance (higher = smoother, more lag).
            outlier_threshold: Minimum innovation distance (m) to flag an outlier.
            max_consecutive_outliers: Consecutive outliers before adopting the new value.
            min_val: Minimum valid sensor measurement in meters.
            max_val: Maximum valid sensor measurement in meters.
        """
        self._q = float(q)
        self._r = float(r)
        self._outlier_threshold = float(outlier_threshold)
        self._max_consecutive_outliers = int(max_consecutive_outliers)
        self._min_val = float(min_val)
        self._max_val = float(max_val)

        self._x: float = 0.0
        self._p: float = 1.0
        self._initialized: bool = False
        self._consecutive_outliers: int = 0
        self._consecutive_invalids: int = 0
        self._last_raw: float = float("inf")

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def value(self) -> float:
        return self._x if self._initialized else float("inf")

    def reset(self, val: Optional[float] = None) -> None:
        """Reset filter state."""
        if val is not None and self._min_val <= val <= self._max_val:
            self._x = float(val)
            self._p = self._r
            self._initialized = True
        else:
            self._x = 0.0
            self._p = 1.0
            self._initialized = False
        self._consecutive_outliers = 0
        self._consecutive_invalids = 0
        self._last_raw = val if val is not None else float("inf")

    def update(self, z: float) -> float:
        """Process incoming raw measurement and return filtered estimate.

        If z is out of valid range (<= 0 or > max_val or inf), handles coasting
        for 1-2 frames before reporting inf.
        """
        self._last_raw = float(z)

        # Check validity
        if math.isinf(z) or math.isnan(z) or z < self._min_val or z > self._max_val:
            self._consecutive_invalids += 1
            # If invalid for 2 or more consecutive cycles, invalidate state
            if self._consecutive_invalids >= 2 or not self._initialized:
                self._initialized = False
                return float("inf")
            # Coast on last estimate with increased covariance
            self._p += self._q * 2.0
            return self._x

        self._consecutive_invalids = 0

        # First valid measurement initializes immediately
        if not self._initialized:
            self._x = float(z)
            self._p = self._r
            self._initialized = True
            self._consecutive_outliers = 0
            return self._x

        # Prediction step
        p_pred = self._p + self._q

        # Innovation (residual)
        y = z - self._x

        # Outlier gating: single-sample large jumps are rejected unless repeated
        if abs(y) > self._outlier_threshold:
            self._consecutive_outliers += 1
            if self._consecutive_outliers < self._max_consecutive_outliers:
                # Reject spike, coast on predicted state
                self._p = p_pred
                return self._x
            # Sustained change (e.g. object genuinely appeared) -> adopt new value
            self._x = float(z)
            self._p = self._r
            self._consecutive_outliers = 0
            return self._x

        # Normal measurement: reset outlier counter and update
        self._consecutive_outliers = 0
        k = p_pred / (p_pred + self._r)
        self._x = self._x + k * y
        self._p = (1.0 - k) * p_pred

        return self._x


class PoseKalmanFilter:
    """3-DoF Kalman filter for 2D robot pose (x, y, yaw).

    Features:
    - Independent position and heading tracking.
    - Continuous angular innovation wrapping across (-pi, pi].
    - Single-frame teleoperation / telemetry glitch rejection.
    - Explicit state overriding on verified move batch completion.
    """

    def __init__(
        self,
        q_pos: float = 5e-4,
        r_pos: float = 2e-3,
        q_yaw: float = 5e-4,
        r_yaw: float = 5e-3,
        outlier_dist_threshold: float = 0.60,
        max_consecutive_outliers: int = 2,
        initial_x: Optional[float] = None,
        initial_y: Optional[float] = None,
        initial_yaw: Optional[float] = None,
    ) -> None:
        """Initialize the Pose Kalman filter."""
        self._q_pos = float(q_pos)
        self._r_pos = float(r_pos)
        self._q_yaw = float(q_yaw)
        self._r_yaw = float(r_yaw)
        self._outlier_dist_sq = float(outlier_dist_threshold) ** 2
        self._max_consecutive_outliers = int(max_consecutive_outliers)

        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        self._px: float = self._r_pos
        self._py: float = self._r_pos
        self._pyaw: float = self._r_yaw

        self._initialized: bool = False
        self._consecutive_outliers: int = 0

        if initial_x is not None and initial_y is not None and initial_yaw is not None:
            self.set_state(initial_x, initial_y, initial_yaw)

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def yaw(self) -> float:
        return self._yaw

    def set_state(self, x: float, y: float, yaw: float) -> None:
        """Explicitly set filter state (e.g. from authoritative FIN feedback or reset)."""
        self._x = float(x)
        self._y = float(y)
        # Normalize yaw to (-pi, pi]
        self._yaw = (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi
        self._px = self._r_pos
        self._py = self._r_pos
        self._pyaw = self._r_yaw
        self._initialized = True
        self._consecutive_outliers = 0

    def reset(self, x: float, y: float, yaw: float) -> None:
        """Reset pose to initial configuration."""
        self.set_state(x, y, yaw)

    def update(self, z_x: float, z_y: float, z_yaw: float) -> Tuple[float, float, float]:
        """Update filter with incoming telemetry measurement.

        Returns:
            Tuple of (filtered_x, filtered_y, filtered_yaw).
        """
        # First reading initializes state immediately
        if not self._initialized:
            self.set_state(z_x, z_y, z_yaw)
            return self._x, self._y, self._yaw

        # Outlier gating for position
        dx = z_x - self._x
        dy = z_y - self._y
        dist_sq = dx * dx + dy * dy

        if dist_sq > self._outlier_dist_sq:
            self._consecutive_outliers += 1
            if self._consecutive_outliers < self._max_consecutive_outliers:
                # Reject transient jump
                self._px += self._q_pos
                self._py += self._q_pos
                self._pyaw += self._q_yaw
                return self._x, self._y, self._yaw
            # Confirmed step change
            self.set_state(z_x, z_y, z_yaw)
            return self._x, self._y, self._yaw

        self._consecutive_outliers = 0

        # Predict step
        px_pred = self._px + self._q_pos
        py_pred = self._py + self._q_pos
        pyaw_pred = self._pyaw + self._q_yaw

        # Kalman update for X
        kx = px_pred / (px_pred + self._r_pos)
        self._x = self._x + kx * (z_x - self._x)
        self._px = (1.0 - kx) * px_pred

        # Kalman update for Y
        ky = py_pred / (py_pred + self._r_pos)
        self._y = self._y + ky * (z_y - self._y)
        self._py = (1.0 - ky) * py_pred

        # Kalman update for Yaw with circular wrapping
        dyaw = (z_yaw - self._yaw + math.pi) % (2.0 * math.pi) - math.pi
        kyaw = pyaw_pred / (pyaw_pred + self._r_yaw)
        self._yaw = self._yaw + kyaw * dyaw
        self._yaw = (self._yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._pyaw = (1.0 - kyaw) * pyaw_pred

        return self._x, self._y, self._yaw
