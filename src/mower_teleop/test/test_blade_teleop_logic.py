from pytest import approx

from mower_teleop.blade_teleop_logic import BladeTeleopConfig
from mower_teleop.blade_teleop_logic import BladeTeleopController
from mower_teleop.blade_teleop_logic import map_axis_to_command


def make_controller(max_blade_command=100.0, joy_timeout=0.3):
    return BladeTeleopController(
        BladeTeleopConfig(
            blade_axis_index=0,
            axis_released_value=1.0,
            axis_pressed_value=-1.0,
            lock_button_index=1,
            estop_button_index=2,
            max_blade_command=max_blade_command,
            joy_timeout=joy_timeout,
            command_ramp_per_sec=5000.0,
        )
    )


def test_rt_axis_mapping():
    assert map_axis_to_command(1.0, 1.0, -1.0, 100.0) == approx(0.0)
    assert map_axis_to_command(0.0, 1.0, -1.0, 100.0) == approx(50.0)
    assert map_axis_to_command(-1.0, 1.0, -1.0, 100.0) == approx(100.0)


def test_lock_holds_current_command():
    controller = make_controller()

    assert controller.handle_joy([0.0], [0, 0, 0], 0.0) is None
    command, event = controller.update(0.05, 0.05)
    assert event is None
    assert command == approx(50.0)

    assert controller.handle_joy([0.0], [0, 1, 0], 0.05) == 'lock'
    assert controller.handle_joy([0.0], [0, 0, 0], 0.10) is None

    assert controller.handle_joy([-1.0], [0, 0, 0], 0.15) is None
    command, event = controller.update(0.20, 0.05)
    assert event is None
    assert command == approx(50.0)


def test_unlock_returns_to_live_trigger_value():
    controller = make_controller()

    controller.handle_joy([0.0], [0, 0, 0], 0.0)
    controller.update(0.05, 0.05)
    controller.handle_joy([0.0], [0, 1, 0], 0.05)
    controller.handle_joy([0.0], [0, 0, 0], 0.10)

    assert controller.handle_joy([-1.0], [0, 1, 0], 0.15) == 'unlock'
    command, event = controller.update(0.20, 0.05)
    assert event is None
    assert command == approx(100.0)


def test_estop_clears_lock_and_zeroes_command():
    controller = make_controller()

    controller.handle_joy([-1.0], [0, 0, 0], 0.0)
    command, event = controller.update(0.05, 0.05)
    assert event is None
    assert command == approx(100.0)

    assert controller.handle_joy([-1.0], [0, 0, 1], 0.06) == 'estop'
    assert controller.current_command == approx(0.0)
    assert controller.desired_command == approx(0.0)
    assert not controller.locked


def test_joy_timeout_zeroes_command_and_unlocks():
    controller = make_controller(joy_timeout=0.3)

    controller.handle_joy([-1.0], [0, 0, 0], 0.0)
    controller.update(0.05, 0.05)
    controller.handle_joy([-1.0], [0, 1, 0], 0.05)

    command, event = controller.update(0.40, 0.35)
    assert event == 'timeout'
    assert command == approx(0.0)
    assert not controller.locked


def test_commands_are_clamped_to_configured_maximum():
    controller = make_controller(max_blade_command=60.0)

    controller.handle_joy([-2.0], [0, 0, 0], 0.0)
    command, event = controller.update(0.05, 0.05)
    assert event is None
    assert command == approx(60.0)
