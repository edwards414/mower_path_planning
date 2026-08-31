from pytest import approx
import math

from mower_teleop.blade_teleop_logic import BladeTeleopConfig
from mower_teleop.blade_teleop_logic import BladeTeleopController
from mower_teleop.blade_teleop_logic import map_axis_to_command


def make_controller(max_blade_command=100.0, joy_timeout=0.3):
    return BladeTeleopController(
        BladeTeleopConfig(
            blade_axis_index=0,
            axis_released_value=1.0,
            axis_pressed_value=-1.0,
            enable_button_index=1,
            estop_button_index=2,
            max_blade_command=max_blade_command,
            joy_timeout=joy_timeout,
            command_ramp_per_sec=5000.0,
            axis_release_tolerance=0.05,
        )
    )


def test_rt_axis_mapping():
    assert map_axis_to_command(1.0, 1.0, -1.0, 100.0) == approx(0.0)
    assert map_axis_to_command(0.0, 1.0, -1.0, 100.0) == approx(50.0)
    assert map_axis_to_command(-1.0, 1.0, -1.0, 100.0) == approx(100.0)
    assert map_axis_to_command(math.nan, 1.0, -1.0, 100.0) == approx(0.0)


def test_initial_trigger_default_cannot_start_blade():
    controller = make_controller()

    assert controller.handle_joy([0.0], [0, 1, 0], 0.0) == 'not_armed'
    command, event = controller.update(0.05, 0.05)
    assert event is None
    assert command == approx(0.0)


def test_released_trigger_then_held_deadman_arms_blade():
    controller = make_controller()

    assert controller.handle_joy([1.0], [0, 0, 0], 0.0) is None
    assert controller.handle_joy([-1.0], [0, 1, 0], 0.05) is None
    command, event = controller.update(0.10, 0.05)
    assert event is None
    assert command == approx(100.0)

    assert (
        controller.handle_joy([-1.0], [0, 0, 0], 0.11)
        == 'deadman_released'
    )
    command, _ = controller.update(0.12, 0.01)
    assert command == approx(0.0)


def test_estop_clears_lock_and_zeroes_command():
    controller = make_controller()

    controller.handle_joy([1.0], [0, 0, 0], 0.0)
    controller.handle_joy([-1.0], [0, 1, 0], 0.01)
    command, event = controller.update(0.05, 0.05)
    assert event is None
    assert command == approx(100.0)

    assert controller.handle_joy([-1.0], [0, 1, 1], 0.06) == 'estop'
    assert controller.current_command == approx(0.0)
    assert controller.desired_command == approx(0.0)

    # Level-triggered stop: a held button can never re-enter the armed branch.
    assert controller.handle_joy([1.0], [0, 0, 1], 0.07) is None
    assert controller.handle_joy([-1.0], [0, 1, 1], 0.08) is None
    assert controller.update(0.09, 0.01)[0] == approx(0.0)


def test_joy_timeout_zeroes_command_and_requires_rearming():
    controller = make_controller(joy_timeout=0.3)

    controller.handle_joy([1.0], [0, 0, 0], 0.0)
    controller.handle_joy([-1.0], [0, 1, 0], 0.01)
    controller.update(0.05, 0.05)

    command, event = controller.update(0.40, 0.35)
    assert event == 'timeout'
    assert command == approx(0.0)
    assert controller.handle_joy([-1.0], [0, 1, 0], 0.41) == 'not_armed'
    command, _ = controller.update(0.42, 0.01)
    assert command == approx(0.0)


def test_commands_are_clamped_to_configured_maximum():
    controller = make_controller(max_blade_command=60.0)

    controller.handle_joy([1.0], [0, 0, 0], 0.0)
    controller.handle_joy([-2.0], [0, 1, 0], 0.01)
    command, event = controller.update(0.05, 0.05)
    assert event is None
    assert command == approx(60.0)


def test_missing_or_non_finite_axis_forces_zero_and_disarms():
    controller = make_controller()
    controller.handle_joy([1.0], [0, 0, 0], 0.0)
    controller.handle_joy([-1.0], [0, 1, 0], 0.01)
    assert controller.update(0.02, 0.01)[0] > 0.0

    assert controller.handle_joy([math.nan], [0, 1, 0], 0.03) == 'invalid_axis'
    assert controller.update(0.04, 0.01)[0] == approx(0.0)
