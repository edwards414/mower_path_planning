"""Blade teleoperation logic shared by the ROS node and tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp *value* to the inclusive ``[minimum, maximum]`` range."""
    return max(minimum, min(value, maximum))


def approach(current: float, target: float, max_delta: float) -> float:
    """Move *current* toward *target* by at most *max_delta*."""
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


def button_is_pressed(buttons: Sequence[int], index: int) -> bool:
    """Return ``True`` when the indexed button exists and is pressed."""
    if index < 0 or index >= len(buttons):
        return False
    return buttons[index] != 0


def map_axis_to_command(
    axis_value: float,
    released_value: float,
    pressed_value: float,
    max_command: float,
) -> float:
    """Map a trigger axis value to a ``0..max_command`` blade command."""
    if (
        not all(math.isfinite(float(value)) for value in (
            axis_value,
            released_value,
            pressed_value,
            max_command,
        ))
        or max_command <= 0.0
        or pressed_value == released_value
    ):
        return 0.0

    normalized = (axis_value - released_value) / (pressed_value - released_value)
    return clamp(normalized, 0.0, 1.0) * max_command


@dataclass(frozen=True)
class BladeTeleopConfig:
    """Configuration for blade joystick teleoperation."""

    blade_axis_index: int = 5
    axis_released_value: float = 1.0
    axis_pressed_value: float = -1.0
    enable_button_index: int = 4
    estop_button_index: int = 0
    max_blade_command: float = 100.0
    joy_timeout: float = 0.3
    command_ramp_per_sec: float = 200.0
    axis_release_tolerance: float = 0.1


class BladeTeleopController:
    """Stateful blade teleoperation controller."""

    def __init__(self, config: BladeTeleopConfig) -> None:
        self.config = config
        self.desired_command = 0.0
        self.current_command = 0.0
        self.last_joy_time: float | None = None
        self._prev_estop_pressed = False
        self._timed_out = False
        self._release_observed = False
        self._deadman_pressed = False
        self._warned_unarmed = False

    def axis_command(self, axes: Sequence[float]) -> float:
        """Translate the configured axis from the latest joy message."""
        axis_value = self.config.axis_released_value
        if 0 <= self.config.blade_axis_index < len(axes):
            axis_value = axes[self.config.blade_axis_index]

        return map_axis_to_command(
            axis_value=axis_value,
            released_value=self.config.axis_released_value,
            pressed_value=self.config.axis_pressed_value,
            max_command=max(0.0, self.config.max_blade_command),
        )

    def _axis_value(self, axes: Sequence[float]) -> float | None:
        if not 0 <= self.config.blade_axis_index < len(axes):
            return None
        value = float(axes[self.config.blade_axis_index])
        return value if math.isfinite(value) else None

    def _force_zero(self) -> None:
        self.desired_command = 0.0
        self.current_command = 0.0

    def handle_joy(
        self,
        axes: Sequence[float],
        buttons: Sequence[int],
        now: float,
    ) -> str | None:
        """Consume a joy sample and update the desired blade command."""
        self.last_joy_time = now
        self._timed_out = False

        deadman_pressed = button_is_pressed(
            buttons,
            self.config.enable_button_index,
        )
        estop_pressed = button_is_pressed(buttons, self.config.estop_button_index)
        axis_value = self._axis_value(axes)
        event = None

        if estop_pressed:
            self._force_zero()
            self._release_observed = False
            self._warned_unarmed = False
            if not self._prev_estop_pressed:
                event = 'estop'
        elif axis_value is None:
            self._force_zero()
            self._release_observed = False
            self._warned_unarmed = False
            event = 'invalid_axis'
        elif not deadman_pressed:
            # Hold-to-run: releasing the hardware button is an immediate stop,
            # not a ramp. Observe a genuinely released trigger before arming so
            # the common reconnect default axis value 0 cannot mean 50% blade.
            was_pressed = self._deadman_pressed
            self._force_zero()
            tolerance = max(0.0, self.config.axis_release_tolerance)
            if abs(axis_value - self.config.axis_released_value) <= tolerance:
                self._release_observed = True
                self._warned_unarmed = False
            if was_pressed:
                event = 'deadman_released'
        elif not self._release_observed:
            self._force_zero()
            if not self._warned_unarmed:
                event = 'not_armed'
                self._warned_unarmed = True
        else:
            self.desired_command = self.axis_command(axes)

        self._deadman_pressed = deadman_pressed
        self._prev_estop_pressed = estop_pressed
        return event

    def update(self, now: float, dt: float) -> tuple[float, str | None]:
        """Advance the controller and return the command to publish."""
        if self.last_joy_time is None:
            self.desired_command = 0.0
            self.current_command = 0.0
            return self.current_command, None

        timeout = max(0.0, self.config.joy_timeout)
        if now - self.last_joy_time > timeout:
            event = None if self._timed_out else 'timeout'
            self._timed_out = True
            self._force_zero()
            self._release_observed = False
            self._deadman_pressed = False
            self._warned_unarmed = False
            return self.current_command, event

        max_delta = max(0.0, self.config.command_ramp_per_sec) * max(dt, 0.0)
        self.current_command = clamp(
            approach(self.current_command, self.desired_command, max_delta),
            0.0,
            max(0.0, self.config.max_blade_command),
        )
        return self.current_command, None
