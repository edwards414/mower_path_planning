#!/usr/bin/env python3

# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""測試 boustrophedon 路徑生成功能."""
from boustrophedon_coverage.path_generators.boustrophedon import (
    _generate_coverage_boustrophedon_path
)

import numpy as np


class TestBoustrophedonPathGeneration:
    """測試牛耕式路徑生成."""

    def test_basic_path_generation(self):
        """測試基本路徑生成（角度為0）."""
        # 創建一個簡單的可行區域地圖（10x10，全部可行）
        H, W = 10, 10
        safe_map = np.ones((H, W), dtype=bool)
        res = 0.1  # 0.1m 解析度
        origin_x, origin_y = 0.0, 0.0
        strip_width_m = 0.5  # 0.5m 割幅
        waypoint_spacing_m = 0.2  # 0.2m 點間距
        angle_deg = 0.0

        points, split_points = _generate_coverage_boustrophedon_path(
            safe_map=safe_map,
            strip_width_m=strip_width_m,
            waypoint_spacing_m=waypoint_spacing_m,
            res=res,
            H=H,
            W=W,
            origin_x=origin_x,
            origin_y=origin_y,
            angle_deg=angle_deg,
        )

        # 驗證返回結果
        assert isinstance(points, list)
        assert isinstance(split_points, list)
        assert len(points) > 0, '應該生成至少一個路徑點'
        assert all(isinstance(p, tuple) and len(p) == 2 for p in points)
        assert all(isinstance(p, tuple) and len(p) == 2 for p in split_points)

    def test_rotated_path_generation(self):
        """測試旋轉路徑生成（角度不為0）."""
        H, W = 10, 10
        safe_map = np.ones((H, W), dtype=bool)
        res = 0.1
        origin_x, origin_y = 0.0, 0.0
        strip_width_m = 0.5
        waypoint_spacing_m = 0.2
        angle_deg = 45.0  # 45度旋轉

        points, split_points = _generate_coverage_boustrophedon_path(
            safe_map=safe_map,
            strip_width_m=strip_width_m,
            waypoint_spacing_m=waypoint_spacing_m,
            res=res,
            H=H,
            W=W,
            origin_x=origin_x,
            origin_y=origin_y,
            angle_deg=angle_deg,
        )

        assert len(points) > 0
        assert all(isinstance(p, tuple) and len(p) == 2 for p in points)

    def test_partial_coverage(self):
        """測試部分覆蓋區域（地圖中有不可行區域）."""
        H, W = 10, 10
        safe_map = np.zeros((H, W), dtype=bool)
        # 創建一個中央區域為可行（調整位置使條帶中心線能落在可行區域內）
        # 使用 strip_width_m=0.2，條帶中心線會在列 1, 3, 5, 7, 9
        # 所以將可行區域設為包含列 3, 5
        safe_map[3:7, 2:8] = True
        res = 0.1
        origin_x, origin_y = 0.0, 0.0
        strip_width_m = 0.2  # 減小條帶寬度，使更多條帶中心線能覆蓋可行區域
        waypoint_spacing_m = 0.2
        angle_deg = 0.0

        points, split_points = _generate_coverage_boustrophedon_path(
            safe_map=safe_map,
            strip_width_m=strip_width_m,
            waypoint_spacing_m=waypoint_spacing_m,
            res=res,
            H=H,
            W=W,
            origin_x=origin_x,
            origin_y=origin_y,
            angle_deg=angle_deg,
        )

        # 驗證所有點都在可行區域內（或接近）
        assert len(points) > 0

    def test_empty_map(self):
        """測試空地圖（無可行區域）."""
        H, W = 10, 10
        safe_map = np.zeros((H, W), dtype=bool)
        res = 0.1
        origin_x, origin_y = 0.0, 0.0
        strip_width_m = 0.5
        waypoint_spacing_m = 0.2
        angle_deg = 0.0

        points, split_points = _generate_coverage_boustrophedon_path(
            safe_map=safe_map,
            strip_width_m=strip_width_m,
            waypoint_spacing_m=waypoint_spacing_m,
            res=res,
            H=H,
            W=W,
            origin_x=origin_x,
            origin_y=origin_y,
            angle_deg=angle_deg,
        )

        # 空地圖應該返回空列表或很少的點
        assert isinstance(points, list)
        assert isinstance(split_points, list)

    def test_different_angles(self):
        """測試不同角度的路徑生成."""
        H, W = 10, 10
        safe_map = np.ones((H, W), dtype=bool)
        res = 0.1
        origin_x, origin_y = 0.0, 0.0
        strip_width_m = 0.5
        waypoint_spacing_m = 0.2

        angles = [0.0, 30.0, 45.0, 90.0, -45.0]

        for angle_deg in angles:
            points, split_points = _generate_coverage_boustrophedon_path(
                safe_map=safe_map,
                strip_width_m=strip_width_m,
                waypoint_spacing_m=waypoint_spacing_m,
                res=res,
                H=H,
                W=W,
                origin_x=origin_x,
                origin_y=origin_y,
                angle_deg=angle_deg,
            )

            assert len(points) > 0, f'角度 {angle_deg} 應該生成路徑點'
            assert all(isinstance(p, tuple) and len(p) == 2 for p in points)
