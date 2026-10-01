#!/usr/bin/env python3
"""Record the Python coverage reference outputs as a JSON regression oracle.

The Python coverage implementation (src/mower_mission/mower_mission/coverage/
and path_generators/) was removed after commit dff480b; this script records its
outputs, for fixed inputs, so the Rust crate can be checked against them:
  filter_safe_components, validate_path, plan_connector (A*),
  zigzag (angle 0 / 30 / 90 and the parity 45), spiral, cell_decomposition.decompose.

It needs the Python sources of that commit, e.g.

  git worktree add /tmp/coverage-ref dff480b
  python3 gen_python_reference_oracle.py --ref /tmp/coverage-ref/src/mower_mission
  git worktree remove /tmp/coverage-ref

(numpy is the only third-party dependency). The output is deterministic: the
same commit and numpy version reproduce python_reference_oracle.json byte for
byte. Default output: python_reference_oracle.json next to this script.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

import argparse  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
_ARGS = argparse.ArgumentParser(description=__doc__.splitlines()[0])
_ARGS.add_argument('--ref', required=True,
                   help='src/mower_mission of a checkout of commit dff480b')
_ARGS.add_argument('--out', default=os.path.join(
    HERE, 'python_reference_oracle.json'))
ARGS = _ARGS.parse_args()
sys.path.insert(0, os.path.abspath(ARGS.ref))

from mower_mission.coverage.cell_decomposition import decompose  # noqa: E402
from mower_mission.coverage.connector_planner import plan_connector  # noqa: E402
from mower_mission.coverage.path_validator import SafeMap, validate_path  # noqa: E402
from mower_mission.coverage.safe_map_filter import filter_safe_components  # noqa: E402
from mower_mission.path_generators.spiral import (  # noqa: E402
    _generate_coverage_spiral_path,
)
from mower_mission.path_generators.zigzag import (  # noqa: E402
    _generate_coverage_zigzag_path,
)


# ── Maps ───────────────────────────────────────────────────────────────────────

MAPS: dict[str, dict] = {}


def add_map(name: str, grid: np.ndarray, res: float = 0.1,
            ox: float = 0.0, oy: float = 0.0, source: str = '') -> str:
    assert name not in MAPS, name
    grid = np.asarray(grid, dtype=bool)
    MAPS[name] = {
        'grid': grid, 'res': float(res), 'ox': float(ox), 'oy': float(oy),
        'source': source,
    }
    return name


def sm_of(name: str) -> SafeMap:
    m = MAPS[name]
    return SafeMap(grid=m['grid'], resolution=m['res'],
                   origin_x=m['ox'], origin_y=m['oy'])


def rows_of(grid: np.ndarray) -> list[str]:
    return [''.join('1' if v else '0' for v in row) for row in grid]


# conftest.py fixtures (res 0.1, origin 0,0)
def _conftest_maps() -> list[str]:
    names = []
    g = np.ones((20, 20), dtype=bool)
    names.append(add_map('fixture_open', g, source='conftest small_open_grid'))
    g = np.zeros((30, 20), dtype=bool)
    g[0:26, 2:8] = True
    g[0:26, 12:18] = True
    g[26:30, 2:18] = True
    names.append(add_map('fixture_u_shape', g, source='conftest u_shape_grid'))
    g = np.zeros((20, 30), dtype=bool)
    g[2:8, 2:10] = True
    g[12:18, 20:28] = True
    names.append(add_map('fixture_two_islands', g, source='conftest two_island_grid'))
    g = np.ones((20, 20), dtype=bool)
    g[8:12, 8:12] = False
    names.append(add_map('fixture_rectangle_obstacle', g,
                         source='conftest rectangle_obstacle_grid'))
    g = np.ones((20, 20), dtype=bool)
    g[:18, 10] = False
    names.append(add_map('fixture_narrow_passage', g,
                         source='conftest narrow_passage_grid'))
    return names


def _random_map(seed: int) -> str:
    """Seeded random obstacle map; kind, size, resolution and origin vary."""
    rng = np.random.RandomState(seed)
    kind = ['blocks', 'blocks_discs', 'noise', 'islands'][seed % 4]
    H = int(rng.randint(12, 29))
    W = int(rng.randint(12, 29))
    res = float([0.1, 0.05, 0.2, 0.1, 0.25][seed % 5])
    ox = float(np.round(rng.uniform(-6.0, 6.0), 3))
    oy = float(np.round(rng.uniform(-6.0, 6.0), 3))
    rr, cc = np.mgrid[0:H, 0:W]
    if kind in ('blocks', 'blocks_discs'):
        g = np.ones((H, W), dtype=bool)
        # optional unsafe border
        if rng.rand() < 0.5:
            g[0, :] = g[-1, :] = False
            g[:, 0] = g[:, -1] = False
        for _ in range(int(rng.randint(1, 6))):
            h = int(rng.randint(1, max(2, H // 3)))
            w = int(rng.randint(1, max(2, W // 3)))
            r0 = int(rng.randint(0, H - h + 1))
            c0 = int(rng.randint(0, W - w + 1))
            g[r0:r0 + h, c0:c0 + w] = False
        if kind == 'blocks_discs':
            for _ in range(int(rng.randint(1, 4))):
                cr = rng.uniform(0, H)
                ccen = rng.uniform(0, W)
                rad = rng.uniform(1.0, min(H, W) / 4.0)
                g[(rr - cr) ** 2 + (cc - ccen) ** 2 <= rad * rad] = False
    elif kind == 'noise':
        g = rng.rand(H, W) < 0.82
    else:  # islands: several random safe rectangles on an unsafe background
        g = np.zeros((H, W), dtype=bool)
        for _ in range(int(rng.randint(2, 6))):
            h = int(rng.randint(2, max(3, H // 2)))
            w = int(rng.randint(2, max(3, W // 2)))
            r0 = int(rng.randint(0, H - h + 1))
            c0 = int(rng.randint(0, W - w + 1))
            g[r0:r0 + h, c0:c0 + w] = True
    return add_map(f'random_{seed:02d}_{kind}', g, res=res, ox=ox, oy=oy,
                   source=f'np.random.RandomState({seed}) kind={kind}')


# ── Helpers ────────────────────────────────────────────────────────────────────

def ftuple(p) -> list[float]:
    return [float(p[0]), float(p[1])]


def fpoints(pts) -> list[list[float]]:
    return [ftuple(p) for p in pts]


def ipairs(pairs) -> list[list[int]]:
    return [[int(a), int(b)] for a, b in pairs]


def check_round_ratio(x: float, what: str) -> None:
    """Python round() is half-to-even, Rust f64::round is half-away-from-zero.

    They disagree only for an exact .5 ratio; such parameters would record a
    known, intentional divergence instead of the shared behaviour, so refuse.
    """
    frac = x - math.floor(x)
    if frac == 0.5:
        raise SystemExit(f'{what}: ratio {x!r} is an exact .5; pick other params')


def cell_centre(name: str, r: int, c: int) -> tuple[float, float]:
    m = MAPS[name]
    return (m['ox'] + (c + 0.5) * m['res'], m['oy'] + (r + 0.5) * m['res'])


# ── Case builders ──────────────────────────────────────────────────────────────

CASES: dict[str, list] = {
    'filter': [], 'validate': [], 'connector': [], 'zigzag': [],
    'spiral': [], 'decompose': [],
}
IDS: set[str] = set()


def _new_id(case_id: str) -> str:
    assert case_id not in IDS, case_id
    IDS.add(case_id)
    return case_id


def rec_filter(case_id, name, min_area_m2, keep_largest_only):
    m = MAPS[name]
    filtered, sizes, kept = filter_safe_components(
        m['grid'], resolution=m['res'], min_area_m2=min_area_m2,
        keep_largest_only=keep_largest_only,
    )
    CASES['filter'].append({
        'id': _new_id(case_id), 'map': name,
        'min_area_m2': float(min_area_m2),
        'keep_largest_only': bool(keep_largest_only),
        'filtered': rows_of(filtered),
        'all_sizes': [int(s) for s in sizes],
        'kept_sizes': [int(s) for s in kept],
    })


def rec_validate(case_id, name, points):
    r = validate_path(points, sm_of(name))
    CASES['validate'].append({
        'id': _new_id(case_id), 'map': name,
        'points': fpoints(points),
        'valid': bool(r.valid),
        'invalid_points': [int(i) for i in r.invalid_points],
        'invalid_segments': ipairs(r.invalid_segments),
        'message': r.message,
    })


def rec_connector(case_id, name, start, end, boundary_weight=None):
    sm = sm_of(name)
    if boundary_weight is None:
        path = plan_connector(start, end, sm)
        bw = 0.2  # Python default
    else:
        path = plan_connector(start, end, sm, boundary_weight)
        bw = boundary_weight
    CASES['connector'].append({
        'id': _new_id(case_id), 'map': name,
        'start': ftuple(start), 'end': ftuple(end),
        'boundary_weight': float(bw),
        'path': None if path is None else fpoints(path),
    })


def rec_zigzag(case_id, name, strip, spacing, angle):
    m = MAPS[name]
    check_round_ratio(strip / m['res'], f'{case_id} strip/res')
    H, W = m['grid'].shape
    pts, split, inv = _generate_coverage_zigzag_path(
        safe_map=m['grid'], strip_width_m=strip, waypoint_spacing_m=spacing,
        res=m['res'], H=H, W=W, origin_x=m['ox'], origin_y=m['oy'],
        angle_deg=angle,
    )
    CASES['zigzag'].append({
        'id': _new_id(case_id), 'map': name,
        'strip_width_m': float(strip), 'waypoint_spacing_m': float(spacing),
        'angle_deg': float(angle),
        'points': fpoints(pts), 'split_points': fpoints(split),
        'invalid_segments': ipairs(inv),
    })


def rec_spiral(case_id, name, strip, spacing):
    m = MAPS[name]
    check_round_ratio(strip / m['res'], f'{case_id} strip/res')
    check_round_ratio(spacing / m['res'], f'{case_id} spacing/res')
    H, W = m['grid'].shape
    pts, split, inv = _generate_coverage_spiral_path(
        safe_map=m['grid'], strip_width_m=strip, waypoint_spacing_m=spacing,
        res=m['res'], H=H, W=W, origin_x=m['ox'], origin_y=m['oy'],
    )
    CASES['spiral'].append({
        'id': _new_id(case_id), 'map': name,
        'strip_width_m': float(strip), 'waypoint_spacing_m': float(spacing),
        'points': fpoints(pts), 'split_points': fpoints(split),
        'invalid_segments': ipairs(inv),
    })


def _mask_runs(mask: np.ndarray) -> list[list[int]]:
    """Encode a bool mask as [row, col_start, col_end] inclusive runs."""
    runs = []
    for r in range(mask.shape[0]):
        row = mask[r]
        c = 0
        W = row.shape[0]
        while c < W:
            if row[c]:
                s = c
                while c + 1 < W and row[c + 1]:
                    c += 1
                runs.append([r, s, c])
            c += 1
    return runs


def rec_decompose(case_id, name):
    cells, graph = decompose(sm_of(name))
    out_cells = []
    for cell in cells:
        out_cells.append({
            'cell_id': int(cell.cell_id),
            'mask_runs': _mask_runs(cell.mask),
            'bbox': [int(v) for v in cell.bbox],
            'area_m2': float(cell.area_m2),
            'centroid_xy': ftuple(cell.centroid_xy),
            'entry_candidates': fpoints(cell.entry_candidates),
            'exit_candidates': fpoints(cell.exit_candidates),
            'valid': bool(cell.valid),
            'invalid_reason': cell.invalid_reason,
        })
    CASES['decompose'].append({
        'id': _new_id(case_id), 'map': name,
        'cells': out_cells,
        'graph': [[int(k), [int(v) for v in vs]] for k, vs in graph.items()],
    })


# ── Input generation ───────────────────────────────────────────────────────────

def random_points(name: str, rng: np.random.RandomState, n: int):
    """Mix of cell centres, random in-cell offsets and out-of-grid points."""
    m = MAPS[name]
    H, W = m['grid'].shape
    res, ox, oy = m['res'], m['ox'], m['oy']
    pts = []
    for _ in range(n):
        u = rng.rand()
        if u < 0.45:
            r, c = int(rng.randint(0, H)), int(rng.randint(0, W))
            pts.append(cell_centre(name, r, c))
        elif u < 0.9:
            pts.append((ox + rng.uniform(0, W * res), oy + rng.uniform(0, H * res)))
        else:  # slightly outside the grid, incl. the (-1, 0) cell truncation zone
            pts.append((ox + rng.uniform(-1.5 * res, (W + 1.5) * res),
                        oy + rng.uniform(-1.5 * res, (H + 1.5) * res)))
    return pts


def safe_cells(name):
    return [tuple(int(v) for v in rc) for rc in np.argwhere(MAPS[name]['grid'])]


def _dump_one_entry_per_line(doc: dict) -> str:
    """Compact JSON with one map / one case per line, so diffs stay readable."""
    def c(v):
        return json.dumps(v, separators=(',', ':'))

    parts = []
    for key, value in doc.items():
        if isinstance(value, dict) and key == 'maps':
            inner = ',\n'.join(f' {c(k)}:{c(v)}' for k, v in value.items())
            parts.append(f'{c(key)}:{{\n{inner}\n}}')
        elif isinstance(value, list):
            inner = ',\n'.join(f' {c(v)}' for v in value)
            parts.append(f'{c(key)}:[\n{inner}\n]')
        else:
            parts.append(f'{c(key)}:{c(value)}')
    return '{\n' + ',\n'.join(parts) + '\n}\n'


def main(out_path: str) -> None:
    fixtures = _conftest_maps()
    randoms = [_random_map(seed) for seed in range(16)]

    # Maps used by the ported Python unit / parity tests (named by their test).
    g = np.ones((10, 10), dtype=bool)
    add_map('open10', g, source='test_backend_parity open 10x10')
    g = np.ones((10, 10), dtype=bool); g[5, 5] = False
    add_map('open10_hole55', g, source='test_backend_parity g[5,5]=False')
    g = np.ones((20, 20), dtype=bool); g[:, 10] = False
    add_map('wall20', g, source='full-height wall at col 10')
    g = np.ones((10, 10), dtype=bool)
    add_map('open10_neg_origin', g, res=0.1, ox=-0.5, oy=-0.5,
            source='test_backend_parity test_negative_origin')
    g = np.zeros((6, 6), dtype=bool); g[0:3, 0:3] = True; g[5, 5] = True
    add_map('tiny_island6', g, source='test_safe_map_filter removes_tiny_island')
    g = np.zeros((8, 8), dtype=bool); g[0:3, 0:3] = True; g[5:7, 5:7] = True
    add_map('two_blocks8', g, source='test_safe_map_filter keeps_largest_only')
    add_map('empty5', np.zeros((5, 5), dtype=bool), source='empty 5x5')
    add_map('empty10', np.zeros((10, 10), dtype=bool), source='empty 10x10')
    g = np.ones((10, 10), dtype=bool); g[0, 0] = False
    add_map('open10_unsafe00', g, source='test_backend_parity unsafe start')
    g = np.ones((5, 5), dtype=bool); g[1, 2] = False; g[2, 1] = False
    add_map('corner5', g, source='no corner cutting 5x5')
    add_map('open30', np.ones((30, 30), dtype=bool), source='open 30x30')

    all_maps = fixtures + randoms

    # filter
    for name in fixtures + randoms:
        rec_filter(f'filter/{name}/default', name, 0.05, True)
        rec_filter(f'filter/{name}/all_kept', name, 0.02, False)
        rec_filter(f'filter/{name}/big_min', name, 0.5, False)
    rec_filter('parity/filter/single_component', 'open10', 0.05, True)
    rec_filter('parity/filter/removes_tiny_island', 'tiny_island6', 0.05, True)
    rec_filter('parity/filter/keeps_largest_only', 'two_blocks8', 0.01, True)
    rec_filter('parity/filter/empty_grid', 'empty5', 0.05, True)

    # validate
    for i, name in enumerate(all_maps):
        rng = np.random.RandomState(1000 + i)
        rec_validate(f'validate/{name}/random', name, random_points(name, rng, 40))
        # a path along safe cell centres (mostly valid) with a few long jumps
        cells = safe_cells(name)
        if cells:
            idx = rng.randint(0, len(cells), size=12)
            rec_validate(f'validate/{name}/safe_cells', name,
                         [cell_centre(name, *cells[j]) for j in idx])
    rec_validate('validate/open10/empty_path', 'open10', [])
    for x in (0.05, 0.55, 0.95):
        for y in (0.05, 0.55, 0.95):
            rec_validate(f'parity/validate/point_open/{x}_{y}', 'open10', [(x, y)])
    rec_validate('parity/validate/point_unsafe', 'open10_hole55', [(0.55, 0.55)])
    rec_validate('parity/validate/segment_crossing_wall', 'wall20',
                 [(0.55, 1.05), (1.45, 1.05)])
    rec_validate('parity/validate/full_path_open_grid', 'fixture_open',
                 [(0.05, 0.05), (0.95, 0.05), (0.95, 0.95), (0.05, 0.95)])
    rec_validate('parity/validate/out_of_bounds_point', 'open10', [(-0.5, 0.05)])
    rec_validate('parity/validate/negative_origin', 'open10_neg_origin',
                 [(-0.45, -0.45), (-0.05, -0.05)])

    # connector
    for i, name in enumerate(all_maps):
        rng = np.random.RandomState(2000 + i)
        cells = safe_cells(name)
        m = MAPS[name]
        H, W = m['grid'].shape
        for k in range(6):
            bw = [None, 0.0, 0.5][k % 3]
            if cells and k < 5:
                a = cells[rng.randint(0, len(cells))]
                b = cells[rng.randint(0, len(cells))]
                # jitter inside the cell so world_to_grid truncation is exercised
                start = (cell_centre(name, *a)[0] + rng.uniform(-0.45, 0.45) * m['res'],
                         cell_centre(name, *a)[1] + rng.uniform(-0.45, 0.45) * m['res'])
                end = cell_centre(name, *b)
            else:  # arbitrary (possibly unsafe / out-of-grid) endpoints
                start, end = random_points(name, rng, 2)
            rec_connector(f'connector/{name}/{k}', name, start, end, bw)
    rec_connector('connector/fixture_open/same_cell', 'fixture_open',
                  (0.55, 0.55), (0.55, 0.55))
    rec_connector('parity/connector/open_grid', 'fixture_open',
                  (0.05, 0.05), (1.95, 1.95))
    rec_connector('parity/connector/disconnected', 'wall20',
                  (0.55, 1.05), (1.55, 1.05))
    rec_connector('parity/connector/wall_with_gap', 'fixture_narrow_passage',
                  (0.55, 0.55), (1.55, 0.55))
    rec_connector('parity/connector/unsafe_start', 'open10_unsafe00',
                  (0.05, 0.05), (0.95, 0.95))
    rec_connector('parity/connector/no_corner_cutting', 'corner5',
                  (0.15, 0.15), (0.35, 0.35))

    # zigzag: (strip, spacing) per map resolution, angle 0 / 30 / 90
    def zz_params(res):
        return {0.1: [(0.2, 0.1), (0.8, 0.3)],
                0.05: [(0.3, 0.1), (0.8, 0.25)],
                0.2: [(0.6, 0.4), (1.0, 0.5)],
                0.25: [(0.8, 0.3), (1.2, 0.5)]}[res]

    for name in all_maps:
        res = MAPS[name]['res']
        for pi, (strip, spacing) in enumerate(zz_params(res)):
            # p0 (narrow strips): axis-aligned + both rotated angles.
            # p1 (wide strips, U-turns): axis-aligned; rotated only on the
            # conftest fixtures to keep the file small.
            if pi == 0 or name.startswith('fixture_'):
                angles = (0.0, 30.0, 90.0) if pi == 0 else (0.0, 30.0)
            else:
                angles = (0.0,)
            for angle in angles:
                rec_zigzag(f'zigzag/{name}/p{pi}/a{int(angle)}', name,
                           strip, spacing, angle)
    rec_zigzag('parity/zigzag/empty_map', 'empty10', 0.2, 0.1, 0.0)
    rec_zigzag('parity/zigzag/open_grid', 'fixture_open', 0.2, 0.1, 0.0)
    rec_zigzag('parity/zigzag/rotated_open_grid', 'fixture_open', 0.2, 0.1, 45.0)
    rec_zigzag('parity/zigzag/obstacle', 'fixture_rectangle_obstacle', 0.2, 0.1, 0.0)
    rec_zigzag('parity/zigzag/wall', 'wall20', 0.2, 0.1, 0.0)
    rec_zigzag('zigzag/open30/wide_u_turns', 'open30', 0.8, 0.2, 0.0)

    # spiral
    def sp_params(res):
        return {0.1: [(0.2, 0.1), (0.4, 0.2)],
                0.05: [(0.3, 0.1), (0.4, 0.2)],
                0.2: [(0.4, 0.2), (0.8, 0.4)],
                0.25: [(0.8, 0.3), (1.2, 0.5)]}[res]

    for name in all_maps:
        res = MAPS[name]['res']
        for pi, (strip, spacing) in enumerate(sp_params(res)):
            rec_spiral(f'spiral/{name}/p{pi}', name, strip, spacing)
    rec_spiral('parity/spiral/empty_map', 'empty10', 0.4, 0.2)
    rec_spiral('parity/spiral/open_grid', 'fixture_open', 0.4, 0.2)
    rec_spiral('parity/spiral/two_islands', 'fixture_two_islands', 0.4, 0.2)

    # decompose
    for name in all_maps:
        rec_decompose(f'decompose/{name}', name)
    rec_decompose('decompose/empty10', 'empty10')

    try:
        numpy_version = np.__version__
        py_version = sys.version.split()[0]
    except Exception:  # pragma: no cover
        numpy_version = py_version = '?'

    doc = {
        'provenance': {
            'generator': 'tests/data/gen_python_reference_oracle.py',
            'python_reference': 'mower_mission coverage/ + path_generators/ '
                                'at commit dff480b (removed afterwards)',
            'python': py_version,
            'numpy': numpy_version,
            'notes': [
                'grids are row strings, row 0 first; "1" = safe',
                'zigzag/spiral use the map res/origin and H, W = grid shape',
                'connector boundary_weight 0.2 is the Python default',
                'decompose masks are [row, col_start, col_end] inclusive runs',
                'the rotated zigzag cases pin CURRENT (known-poor) behaviour',
            ],
        },
        'maps': {
            name: {
                'rows': rows_of(m['grid']),
                'res': m['res'], 'origin_x': m['ox'], 'origin_y': m['oy'],
                'source': m['source'],
            }
            for name, m in MAPS.items()
        },
        **CASES,
    }
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        f.write(_dump_one_entry_per_line(doc))
    size = os.path.getsize(out_path)
    counts = {k: len(v) for k, v in CASES.items()}
    print(f'wrote {out_path} ({size} bytes) maps={len(MAPS)} cases={counts}')


if __name__ == '__main__':
    main(ARGS.out)
