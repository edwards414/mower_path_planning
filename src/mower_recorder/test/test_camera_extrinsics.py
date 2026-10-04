"""Extrinsics math: optical-frame axes, base_link chain, URDF parsing."""
import math
import os

import pytest
import yaml

from mower_recorder import camera_extrinsics as ce

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
URDF = os.path.join(os.path.dirname(PKG), 'mower_description', 'mower_robot',
                    'robot.urdf.xacro')


def _placeholder():
    with open(os.path.join(PKG, 'config', 'camera_front_extrinsics.yaml')) as f:
        return yaml.safe_load(f)


def _close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_shipped_placeholder_is_valid_but_flagged():
    ext = _placeholder()
    errors, warnings = ce.validate(ext)
    assert errors == []
    assert any('placeholder' in w for w in warnings)


def test_optical_axes_for_downward_pitch():
    ext = _placeholder()
    ext['camera'].update(pitch_deg=20.0, roll_deg=0.0, yaw_deg=0.0)
    res = ce.compute(ext)
    t = res['base_footprint_to_optical']
    th = math.radians(20.0)
    # optical z = forward and 20 deg down, x = right, y = down(ish)
    assert _close(t.rotate((0, 0, 1)), (math.cos(th), 0.0, -math.sin(th)))
    assert _close(t.rotate((1, 0, 0)), (0.0, -1.0, 0.0))
    assert _close(t.rotate((0, 1, 0)), (-math.sin(th), 0.0, -math.cos(th)))
    assert res['camera_height_m'] == 0.62
    assert res['camera_pitch_deg'] == 20.0


def test_height_uses_axle_reference():
    ext = _placeholder()
    ref = ce.default_reference()
    res = ce.compute(ext, ref)
    axle = ref['axle_center_in_base_footprint']
    x, y, z = res['base_footprint_to_camera_link'].translation
    assert x == pytest.approx(axle[0] + 0.35)
    assert y == pytest.approx(axle[1])
    # z in base_footprint = height above ground - base_footprint height
    assert z == pytest.approx(0.62 - (0.09 - axle[2]))
    assert res['base_footprint_height_m'] == pytest.approx(0.09 - axle[2])


def test_base_link_chain_composes_back():
    ext = _placeholder()
    ext['camera'].update(roll_deg=2.0, yaw_deg=-3.0)
    ref = ce.default_reference()
    res = ce.compute(ext, ref)
    chained = ref['base_link_in_base_footprint'] @ res['base_link_to_camera_link']
    direct = res['base_footprint_to_camera_link']
    assert _close(chained.translation, direct.translation)
    for i in range(3):
        assert _close(chained.rotation[i], direct.rotation[i])


def test_real_urdf_reference():
    with open(URDF) as f:
        ref = ce.urdf_reference(f.read())
    bl = ref['base_link_in_base_footprint']
    assert _close(bl.rpy, (0.0, 0.0, 1.5708), 1e-6)
    axle = ref['axle_center_in_base_footprint']
    # drive axle ~0.166 m behind base_footprint, ~3 mm below it (P-098)
    assert axle[0] == pytest.approx(-0.165534, abs=1e-5)
    assert axle[1] == pytest.approx(0.01453, abs=1e-5)
    assert axle[2] == pytest.approx(-0.00295275, abs=1e-6)
    default = ce.default_reference()['axle_center_in_base_footprint']
    assert _close(axle, default, 1e-5)


def test_explicit_axle_overrides_urdf():
    ext = _placeholder()
    ext['reference']['axle_center_in_base_footprint_m'] = [0.0, 0.0, 0.0]
    res = ce.compute(ext)
    assert res['base_footprint_to_camera_link'].translation[0] == pytest.approx(0.35)


def test_rpy_quaternion_round_trip():
    for rpy in ((0.1, -0.4, 2.0), (-1.2, 0.3, -2.9), (0.0, 1.2, 0.5)):
        m = ce.rpy_to_matrix(*rpy)
        assert _close(ce.matrix_to_rpy(m), rpy, 1e-9)
        q = ce.matrix_to_quaternion(m)
        m2 = ce.quaternion_to_matrix(q)
        for i in range(3):
            assert _close(m[i], m2[i], 1e-9)


def test_validation_errors():
    ext = _placeholder()
    ext['camera']['pitch_deg'] = 95
    ext['source'] = 'guess'
    del ext['camera']['height_m']
    errors, _ = ce.validate(ext)
    assert len(errors) >= 2
    with pytest.raises(ValueError):
        ce.compute(ext)


def test_static_tf_arguments_and_derived_block():
    res = ce.compute(_placeholder())
    args = ce.static_tf_arguments(res['camera_link_to_optical'], ce.CAMERA_LINK,
                                  ce.OPTICAL_FRAME)
    assert args[-4:] == ['--frame-id', ce.CAMERA_LINK, '--child-frame-id',
                         ce.OPTICAL_FRAME]
    qx, qy, qz, qw = (float(args[i]) for i in (7, 9, 11, 13))
    assert _close((qx, qy, qz, qw), (-0.5, 0.5, -0.5, 0.5), 1e-9)
    block = ce.derived_block(res)
    opt = block['base_footprint_to_camera_front_optical_frame']
    assert opt['parent'] == 'base_footprint'
    assert [t['parent'] for t in block['published_tf']] == ['base_link',
                                                           'camera_front_link']
    yaml.safe_dump(block)   # plain types only
