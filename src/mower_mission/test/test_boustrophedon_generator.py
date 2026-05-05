import numpy as np

from mower_mission.coverage.path_validator import SafeMap, validate_path
from mower_mission.path_generators.boustrophedon import (
    _generate_coverage_boustrophedon_path,
)


def test_segment_end_stops_before_first_unsafe_cell():
    safe_map = np.array(
        [
            [False],
            [True],
            [True],
            [False],
        ],
        dtype=np.uint8,
    )

    points, _split_points, invalid_segments = (
        _generate_coverage_boustrophedon_path(
            safe_map=safe_map,
            strip_width_m=1.0,
            waypoint_spacing_m=1.0,
            res=1.0,
            H=4,
            W=1,
            origin_x=0.0,
            origin_y=0.0,
            angle_deg=0.0,
        )
    )

    assert points == [(0.5, 1.5), (0.5, 2.5)]
    assert invalid_segments == []

    safe_map_struct = SafeMap(
        grid=safe_map.astype(bool),
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )
    assert validate_path(points, safe_map_struct).valid
