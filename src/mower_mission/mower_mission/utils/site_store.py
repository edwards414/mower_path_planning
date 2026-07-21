# Copyright 2026 fxrbindi
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

"""Named zone-set "sites": WGS84-anchored snapshots of recorded objects.

The GPS-fed map frame re-originates at the boot pose, so map-frame XY saved
in one session lands in the wrong place after a reboot.  A site therefore
stores every vertex in lat/lon together with the datum (map-origin lat/lon +
bearing of map +X from true north) that was current at save time; loading
re-projects the vertices into whatever map frame exists now, using the
current datum.

The lat/lon <-> XY conversion is the same local equirectangular
approximation the app uses for the satellite overlay
(mower_lawer_app GeoAnchor.worldToLatLng) — sub-metre accurate over a
lawn-sized area.

Everything here is ROS-free and operates on plain dicts:

    datum = {'lat': .., 'lon': .., 'bearing_rad': .., 'source': ..}
    site  = {'version': 1, 'name': .., 'created_at': .., 'updated_at': ..,
             'datum': datum,
             'zones':      [{'id', 'ns', 'points_ll': [[lat, lon], ...]}],
             'risk_zones': [ .. same .. ],
             'channels':   [{ .. same .., 'color': {r,g,b,a}, 'scale'}],
             'meta': {'zone_count', 'risk_count', 'channel_count',
                      'area_m2', 'datum_source'}}

path_record_node owns the Marker <-> dict conversion.
"""

import json
import math
import os
import re
from datetime import datetime, timezone

M_PER_DEG_LAT = 111320.0

_INVALID_NAME_CHARS = re.compile(r'[\\/\x00-\x1f]')


# ── geo conversion ─────────────────────────────────────────────────────────

def _wrap_deg(deg):
    """Wrap a longitude difference into [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def ll_from_xy(x, y, datum):
    """Map-frame metres -> (lat, lon) using the given datum."""
    b = datum['bearing_rad']
    east = x * math.sin(b) - y * math.cos(b)
    north = x * math.cos(b) + y * math.sin(b)
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(datum['lat']))
    lat = datum['lat'] + north / M_PER_DEG_LAT
    lon = datum['lon'] + (
        0.0 if abs(m_per_deg_lon) < 1e-9 else east / m_per_deg_lon)
    return lat, _wrap_deg(lon)


def xy_from_ll(lat, lon, datum):
    """(lat, lon) -> map-frame metres using the given datum."""
    b = datum['bearing_rad']
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(datum['lat']))
    east = _wrap_deg(lon - datum['lon']) * m_per_deg_lon
    north = (lat - datum['lat']) * M_PER_DEG_LAT
    x = east * math.sin(b) + north * math.cos(b)
    y = -east * math.cos(b) + north * math.sin(b)
    return x, y


def polygon_area_m2(pts_xy):
    """Shoelace area of an XY polygon (open or closed vertex list)."""
    n = len(pts_xy)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        x1, y1 = pts_xy[i][0], pts_xy[i][1]
        x2, y2 = pts_xy[(i + 1) % n][0], pts_xy[(i + 1) % n][1]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


# ── site files ─────────────────────────────────────────────────────────────

def valid_name(name):
    """站名可用中文/emoji，只擋路徑字元與空名。"""
    name = (name or '').strip()
    if not name or name in ('.', '..') or _INVALID_NAME_CHARS.search(name):
        return None
    return name


def site_path(sites_dir, name):
    return os.path.join(sites_dir, name + '.json')


def build_site(name, datum, zones, risk_zones, channels, created_at=None):
    """Assemble a site dict from XY object dicts, converting vertices to LL.

    zones / risk_zones: [{'id', 'ns', 'points': [[x, y], ...]}]
    channels: same plus 'color' {r,g,b,a} and 'scale'.
    """
    now = datetime.now(timezone.utc).isoformat(timespec='seconds')

    def to_ll(objs, keep=()):
        out = []
        for o in objs:
            entry = {
                'id': o['id'],
                'ns': o['ns'],
                'points_ll': [list(ll_from_xy(p[0], p[1], datum))
                              for p in o['points']],
            }
            for k in keep:
                if k in o:
                    entry[k] = o[k]
            out.append(entry)
        return out

    area = sum(polygon_area_m2(o['points']) for o in zones)
    return {
        'version': 1,
        'name': name,
        'created_at': created_at or now,
        'updated_at': now,
        'datum': dict(datum),
        'zones': to_ll(zones),
        'risk_zones': to_ll(risk_zones),
        'channels': to_ll(channels, keep=('color', 'scale')),
        'meta': {
            'zone_count': len(zones),
            'risk_count': len(risk_zones),
            'channel_count': len(channels),
            'area_m2': round(area, 1),
            'datum_source': datum.get('source', ''),
        },
    }


def site_to_xy(site, datum):
    """Re-project a site's LL vertices into the current map frame.

    Returns (zones, risk_zones, channels) as XY object dicts of the same
    shape build_site() consumes.
    """
    def to_xy(objs, keep=()):
        out = []
        for o in objs:
            entry = {
                'id': o['id'],
                'ns': o['ns'],
                'points': [list(xy_from_ll(ll[0], ll[1], datum))
                           for ll in o['points_ll']],
            }
            for k in keep:
                if k in o:
                    entry[k] = o[k]
            out.append(entry)
        return out

    return (
        to_xy(site.get('zones', [])),
        to_xy(site.get('risk_zones', [])),
        to_xy(site.get('channels', []), keep=('color', 'scale')),
    )


def write_site(sites_dir, site):
    # 原子寫入：先寫 .tmp 再 os.replace，斷電/被 kill 不會留下半截檔
    # （active site 檔在每次 /edit_zone 後都會被覆寫，唯一副本必須保護）。
    os.makedirs(sites_dir, exist_ok=True)
    path = site_path(sites_dir, site['name'])
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(site, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def read_site(sites_dir, name):
    with open(site_path(sites_dir, name), 'r') as f:
        return json.load(f)


def delete_site(sites_dir, name):
    os.remove(site_path(sites_dir, name))


def rename_site(sites_dir, old_name, new_name):
    site = read_site(sites_dir, old_name)
    site['name'] = new_name
    write_site(sites_dir, site)
    delete_site(sites_dir, old_name)


def list_sites(sites_dir, active=None):
    """Listing payload for /site_list and SiteOp.sites_json (corrupt files
    are skipped rather than failing the whole listing)."""
    sites = []
    if os.path.isdir(sites_dir):
        for fname in sorted(os.listdir(sites_dir)):
            if not fname.endswith('.json'):
                continue
            try:
                with open(os.path.join(sites_dir, fname), 'r') as f:
                    site = json.load(f)
                meta = site.get('meta', {})
                sites.append({
                    'name': site.get('name', fname[:-len('.json')]),
                    'created_at': site.get('created_at', ''),
                    'updated_at': site.get('updated_at', ''),
                    'zone_count': meta.get('zone_count', 0),
                    'risk_count': meta.get('risk_count', 0),
                    'channel_count': meta.get('channel_count', 0),
                    'area_m2': meta.get('area_m2', 0.0),
                    'datum_source': meta.get('datum_source', ''),
                })
            except (OSError, ValueError):
                continue
    return {'active': active, 'sites': sites}
