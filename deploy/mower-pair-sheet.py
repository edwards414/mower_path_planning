#!/usr/bin/env python3
"""Printable A4 sheet of pairing-QR stickers for one robot (run on a workstation).

    ssh cat@<robot> mower-pair --json --no-qr > pair.json
    python3 deploy/mower-pair-sheet.py pair.json --pdf mower_pair_qr.pdf

Needs `pip install segno`. The PDF is printed with Google Chrome (headless) when
it is installed; otherwise open the .html next to it in any browser and print
it (A4, no margins). 2 x 2 labels of 80 x 100 mm, QR 56 mm at error
correction level H so a scratched sticker still scans. The QR carries the
pairing secret: keep the sheet with the robot, and re-print after
`sudo mower-pair --rotate`.
"""

import argparse
import html
import json
import os
import shutil
import subprocess
import sys

try:
    import segno
except ImportError:
    sys.exit('pip install segno')

CHROME = [
    '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
    'google-chrome', 'chromium', 'chromium-browser',
]


def label_html(pair: dict) -> str:
    qr = segno.make(pair['url'], error='h')
    svg = qr.svg_inline(scale=1, border=4, dark='#000', light='#fff', omitsize=True)
    return f'''
<div class="label">
  <div class="head">
    <div class="name">{html.escape(pair.get('name') or '')}</div>
    <div class="id">{html.escape(pair['robot_id'])}</div>
  </div>
  <div class="qr">{svg}</div>
  <div class="cta">用 Mower App 掃描配對</div>
  <div class="sub">Scan with the Mower app to pair</div>
  <div class="meta">區網 {html.escape(pair.get('lan') or '?')}:9090</div>
  <div class="note">含配對密鑰，勿拍照外流（重設：sudo mower-pair --rotate）</div>
</div>'''


def sheet_html(pair: dict) -> str:
    label = label_html(pair)
    rid = html.escape(pair['robot_id'])
    return f'''<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8">
<title>Mower pairing QR {rid}</title>
<style>
  @page {{ size: A4; margin: 0; }}
  html, body {{ margin: 0; padding: 0; background: #fff; }}
  body {{ width: 210mm; height: 297mm; font-family: -apple-system, "PingFang TC", "Noto Sans CJK TC", "Helvetica Neue", Arial, sans-serif; color: #000;
         -webkit-print-color-adjust: exact; print-color-adjust: exact; }}
  .sheet {{ position: absolute; top: 40mm; left: 25mm; width: 160mm; height: 200mm;
            display: grid; grid-template-columns: 80mm 80mm; grid-template-rows: 100mm 100mm; }}
  .label {{ box-sizing: border-box; width: 80mm; height: 100mm; padding: 5mm 5mm 4mm; overflow: hidden;
            border: 0.3mm dashed #999; display: flex; flex-direction: column; align-items: center; text-align: center; }}
  .head {{ width: 100%; display: flex; align-items: baseline; justify-content: space-between; border-bottom: 0.5mm solid #000; padding-bottom: 1.5mm; }}
  .name {{ font-size: 15pt; font-weight: 700; letter-spacing: 0.02em; }}
  .id {{ font-family: Menlo, "SF Mono", "DejaVu Sans Mono", monospace; font-size: 14pt; font-weight: 700; }}
  .qr {{ width: 56mm; height: 56mm; margin: 3mm 0 2mm; }}
  .qr svg {{ display: block; width: 56mm; height: 56mm; shape-rendering: crispEdges; }}
  .cta {{ font-size: 15pt; font-weight: 700; }}
  .sub {{ font-size: 8.5pt; color: #333; margin-top: 0.5mm; }}
  .meta {{ font-size: 8pt; color: #333; margin-top: 2mm; font-family: Menlo, "SF Mono", "DejaVu Sans Mono", monospace; }}
  .note {{ font-size: 6.5pt; color: #666; margin-top: auto; white-space: nowrap; }}
  .cut {{ position: absolute; left: 0; width: 210mm; text-align: center; font-size: 8pt; color: #888; }}
</style></head><body>
  <div class="cut" style="top:30mm">Mower 配對 QR — {rid}（沿虛線裁切，貼在車身乾淨平面；EC level H）</div>
  <div class="sheet">{label}{label}{label}{label}</div>
</body></html>'''


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pair_json', help="output of `mower-pair --json --no-qr` ('-' for stdin)")
    ap.add_argument('--pdf', help='write this PDF (needs Chrome); the .html is always written next to it')
    ap.add_argument('--html', help='HTML path (default: <pdf>.html or mower_pair_qr_<id>.html)')
    args = ap.parse_args()

    pair = json.load(sys.stdin if args.pair_json == '-' else open(args.pair_json))
    for key in ('url', 'robot_id'):
        if not pair.get(key):
            sys.exit(f'{key} missing: feed this the JSON from `mower-pair --json --no-qr`')
    html_path = args.html or (os.path.splitext(args.pdf)[0] + '.html' if args.pdf else f'mower_pair_qr_{pair["robot_id"]}.html')
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(sheet_html(pair))
    print(f'wrote {html_path}')
    if not args.pdf:
        return 0
    exe = next((c for c in CHROME if os.path.exists(c) or shutil.which(c)), None)
    if exe is None:
        print('Chrome not found: open the .html in a browser and print it to PDF (A4, no margins)')
        return 1
    out = os.path.abspath(args.pdf)
    subprocess.run(
        [exe, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', f'--print-to-pdf={out}',
         'file://' + os.path.abspath(html_path)],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
