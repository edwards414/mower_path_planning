import numpy as np

from mower_mission.map_safety import erode_free_space_grid


def test_free_space_erosion_treats_map_exterior_as_occupied():
    result = erode_free_space_grid(
        np.zeros((41, 41), dtype=np.int8),
        resolution_m=0.1,
        inflate_radius_m=0.75,
    )

    assert np.all(result[:8, :] == 100)
    assert np.all(result[-8:, :] == 100)
    assert np.all(result[:, :8] == 100)
    assert np.all(result[:, -8:] == 100)
    assert result[20, 20] == 0


def test_free_space_narrower_than_safety_diameter_is_rejected():
    result = erode_free_space_grid(
        np.zeros((10, 10), dtype=np.int8),
        resolution_m=0.1,
        inflate_radius_m=0.75,
    )

    assert not np.any(result == 0)
