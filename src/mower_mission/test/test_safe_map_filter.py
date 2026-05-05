import numpy as np

from mower_mission.coverage.safe_map_filter import filter_safe_components


def test_filter_safe_components_removes_tiny_island():
    safe = np.zeros((6, 6), dtype=bool)
    safe[0:3, 0:3] = True
    safe[5, 5] = True

    filtered, sizes, kept_sizes = filter_safe_components(
        safe,
        resolution=0.1,
        min_area_m2=0.05,
    )

    assert sizes == [9, 1]
    assert kept_sizes == [9]
    assert filtered[0:3, 0:3].all()
    assert not filtered[5, 5]


def test_filter_safe_components_keeps_largest_only():
    safe = np.zeros((8, 8), dtype=bool)
    safe[0:3, 0:3] = True
    safe[5:7, 5:7] = True

    filtered, sizes, kept_sizes = filter_safe_components(
        safe,
        resolution=0.1,
        min_area_m2=0.01,
        keep_largest_only=True,
    )

    assert sizes == [9, 4]
    assert kept_sizes == [9]
    assert int(np.count_nonzero(filtered)) == 9
