#!/usr/bin/env python3

import math


class BatteryDistanceSimulator:
    """Track simulated battery drain from traveled 2D distance."""

    def __init__(
        self,
        initial_percentage: float = 100.0,
        meters_per_percent: float = 100.0,
        min_delta_m: float = 0.0,
    ):
        if meters_per_percent <= 0.0:
            raise ValueError('meters_per_percent must be positive')
        if min_delta_m < 0.0:
            raise ValueError('min_delta_m must be non-negative')

        self.initial_percentage = self._clamp_percentage(initial_percentage)
        self.meters_per_percent = float(meters_per_percent)
        self.min_delta_m = float(min_delta_m)
        self.distance_m = 0.0
        self.last_position: tuple[float, float] | None = None

    @staticmethod
    def _clamp_percentage(value: float) -> float:
        return min(100.0, max(0.0, float(value)))

    @property
    def percentage(self) -> float:
        used_percentage = self.distance_m / self.meters_per_percent
        return self._clamp_percentage(self.initial_percentage - used_percentage)

    @property
    def percentage_fraction(self) -> float:
        return self.percentage / 100.0

    def reset(self):
        self.distance_m = 0.0
        self.last_position = None

    def update_position(self, x: float, y: float) -> float:
        if not math.isfinite(x) or not math.isfinite(y):
            return self.percentage

        position = (float(x), float(y))
        if self.last_position is None:
            self.last_position = position
            return self.percentage

        delta = math.hypot(
            position[0] - self.last_position[0],
            position[1] - self.last_position[1],
        )
        self.last_position = position

        if delta >= self.min_delta_m:
            self.distance_m += delta
        return self.percentage
