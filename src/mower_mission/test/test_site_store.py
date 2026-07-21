"""Unit tests for utils/site_store.py — named zone-set sites.

Pure-python module: geo conversion, site build/reproject round-trips, and
file operations, all testable without rclpy.
"""

import json
import math

import pytest

from mower_mission.utils import site_store


DATUM_A = {'lat': 23.6939508, 'lon': 120.5376539,
           'bearing_rad': 0.0, 'source': 'navsat'}
# Same field, different boot: origin ~35 m away, map frame rotated 30°.
DATUM_B = {'lat': 23.6942, 'lon': 120.5379,
           'bearing_rad': math.radians(30.0), 'source': 'navsat'}


# ── geo conversion ─────────────────────────────────────────────────────────

@pytest.mark.parametrize('datum', [DATUM_A, DATUM_B])
@pytest.mark.parametrize('xy', [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0),
                                (-7.3, 42.1), (123.4, -56.7)])
def test_ll_xy_round_trip(datum, xy):
    lat, lon = site_store.ll_from_xy(xy[0], xy[1], datum)
    x, y = site_store.xy_from_ll(lat, lon, datum)
    assert x == pytest.approx(xy[0], abs=1e-6)
    assert y == pytest.approx(xy[1], abs=1e-6)


def test_bearing_convention_matches_app_geo_anchor():
    # bearing = map +X clockwise from true north; with bearing 90° map +X
    # points due east, so (10, 0) must move longitude only.
    datum = dict(DATUM_A, bearing_rad=math.radians(90.0))
    lat, lon = site_store.ll_from_xy(10.0, 0.0, datum)
    assert lat == pytest.approx(datum['lat'], abs=1e-9)
    assert lon > datum['lon']
    # With bearing 0 map +X points north: latitude only.
    lat, lon = site_store.ll_from_xy(10.0, 0.0, DATUM_A)
    assert lon == pytest.approx(DATUM_A['lon'], abs=1e-9)
    assert (lat - DATUM_A['lat']) * site_store.M_PER_DEG_LAT == \
        pytest.approx(10.0, abs=1e-6)


def test_cross_session_reprojection_is_earth_fixed():
    """The core guarantee: save under datum A, load under datum B, and every
    vertex still sits at the same spot on Earth."""
    zone_xy_a = [[0.0, 0.0], [8.0, 0.5], [7.5, 6.0], [-0.5, 5.5], [0.0, 0.0]]
    site = site_store.build_site(
        'field', DATUM_A,
        zones=[{'id': 1, 'ns': 'zones', 'points': zone_xy_a}],
        risk_zones=[], channels=[])

    zones_b, _, _ = site_store.site_to_xy(site, DATUM_B)
    for (x_a, y_a), (x_b, y_b) in zip(zone_xy_a, zones_b[0]['points']):
        ll_a = site_store.ll_from_xy(x_a, y_a, DATUM_A)
        ll_b = site_store.ll_from_xy(x_b, y_b, DATUM_B)
        assert ll_b[0] == pytest.approx(ll_a[0], abs=1e-9)
        assert ll_b[1] == pytest.approx(ll_a[1], abs=1e-9)
    # And the loaded XY must differ from the saved XY (frame really moved).
    assert zones_b[0]['points'][1] != pytest.approx(zone_xy_a[1], abs=0.1)


def test_polygon_area():
    square = [[0, 0], [4, 0], [4, 4], [0, 4]]
    assert site_store.polygon_area_m2(square) == pytest.approx(16.0)
    closed = square + [[0, 0]]
    assert site_store.polygon_area_m2(closed) == pytest.approx(16.0)
    assert site_store.polygon_area_m2([[0, 0], [1, 1]]) == 0.0


# ── names ──────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('bad', ['', '  ', '.', '..', 'a/b', 'a\\b', 'x\x00y'])
def test_invalid_names_rejected(bad):
    assert site_store.valid_name(bad) is None


def test_unicode_names_accepted():
    assert site_store.valid_name(' 後院草皮 ') == '後院草皮'


# ── site build + files ─────────────────────────────────────────────────────

def _sample_site(name='後院'):
    return site_store.build_site(
        name, DATUM_A,
        zones=[{'id': 1, 'ns': 'zones',
                'points': [[0, 0], [4, 0], [4, 4], [0, 4], [0, 0]]}],
        risk_zones=[{'id': 2, 'ns': 'risk_zones',
                     'points': [[1, 1], [2, 1], [2, 2], [1, 1]]}],
        channels=[{'id': 1, 'ns': 'chennal_path',
                   'points': [[5, 5], [9, 9]],
                   'color': {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 0.8},
                   'scale': 0.1}])


def test_build_site_meta_and_channel_extras():
    site = _sample_site()
    assert site['meta'] == {
        'zone_count': 1, 'risk_count': 1, 'channel_count': 1,
        'area_m2': 16.0, 'datum_source': 'navsat',
    }
    assert site['channels'][0]['color']['g'] == 1.0
    assert site['channels'][0]['scale'] == 0.1
    assert site['datum'] == DATUM_A


def test_build_site_preserves_created_at():
    site = site_store.build_site(
        's', DATUM_A, zones=[], risk_zones=[], channels=[],
        created_at='2026-01-01T00:00:00+00:00')
    assert site['created_at'] == '2026-01-01T00:00:00+00:00'
    assert site['updated_at'] != site['created_at']


def test_write_read_rename_delete_and_list(tmp_path):
    sites_dir = str(tmp_path / 'sites')
    site_store.write_site(sites_dir, _sample_site('後院'))
    site_store.write_site(sites_dir, _sample_site('前院'))

    listing = site_store.list_sites(sites_dir, active='後院')
    assert listing['active'] == '後院'
    assert sorted(s['name'] for s in listing['sites']) == ['前院', '後院']
    assert listing['sites'][0]['zone_count'] == 1
    assert listing['sites'][0]['area_m2'] == 16.0

    loaded = site_store.read_site(sites_dir, '後院')
    assert loaded['name'] == '後院'

    site_store.rename_site(sites_dir, '後院', '大後院')
    names = {s['name'] for s in site_store.list_sites(sites_dir)['sites']}
    assert names == {'前院', '大後院'}
    assert site_store.read_site(sites_dir, '大後院')['name'] == '大後院'

    site_store.delete_site(sites_dir, '前院')
    names = {s['name'] for s in site_store.list_sites(sites_dir)['sites']}
    assert names == {'大後院'}


def test_list_skips_corrupt_files(tmp_path):
    sites_dir = str(tmp_path)
    site_store.write_site(sites_dir, _sample_site('好的'))
    (tmp_path / '壞的.json').write_text('not json{', encoding='utf-8')
    listing = site_store.list_sites(sites_dir)
    assert [s['name'] for s in listing['sites']] == ['好的']


def test_list_missing_dir_is_empty():
    assert site_store.list_sites('/nonexistent/dir') == \
        {'active': None, 'sites': []}


def test_site_file_is_utf8_readable(tmp_path):
    sites_dir = str(tmp_path)
    site_store.write_site(sites_dir, _sample_site('後院'))
    raw = (tmp_path / '後院.json').read_text(encoding='utf-8')
    assert '後院' in raw  # ensure_ascii=False keeps the name human-readable
    assert json.loads(raw)['version'] == 1


def test_write_site_is_atomic(tmp_path):
    sites_dir = str(tmp_path)
    site_store.write_site(sites_dir, _sample_site('後院'))
    # No .tmp left behind, and list_sites never picks tmp files up.
    assert sorted(p.name for p in tmp_path.iterdir()) == ['後院.json']
    (tmp_path / '殘留.json.tmp').write_text('{', encoding='utf-8')
    names = [s['name'] for s in site_store.list_sites(sites_dir)['sites']]
    assert names == ['後院']


def test_antimeridian_wrap():
    # Site saved just west of the antimeridian, reloaded under a datum just
    # east of it (~22 m away): vertices must land metres away, not 38,000 km.
    west = {'lat': -16.8, 'lon': 179.9999, 'bearing_rad': 0.0,
            'source': 'navsat'}
    east = {'lat': -16.8, 'lon': -179.9999, 'bearing_rad': 0.0,
            'source': 'navsat'}
    lat, lon = site_store.ll_from_xy(5.0, 5.0, west)
    assert -180.0 <= lon < 180.0
    x, y = site_store.xy_from_ll(lat, lon, east)
    assert abs(x) < 100.0 and abs(y) < 100.0
    # And the round trip back to LL stays on the same spot on Earth.
    lat2, lon2 = site_store.ll_from_xy(x, y, east)
    assert lat2 == pytest.approx(lat, abs=1e-9)
    assert site_store._wrap_deg(lon2 - lon) == pytest.approx(0.0, abs=1e-9)
