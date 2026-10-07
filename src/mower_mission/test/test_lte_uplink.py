"""The 4G uplink pieces in deploy/: modem detection + signal for the app's
telemetry, and the LTE watchdog. Pure file / parser checks, no hardware."""
import importlib.util
from pathlib import Path

DEPLOY = Path(__file__).resolve().parents[3] / 'deploy'


def _link_status():
    spec = importlib.util.spec_from_file_location(
        'mower_link_status', DEPLOY / 'host' / 'mower-link-status.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_simcom_cell_report_gives_the_numbers_csq_hides():
    """lubancat's modem is a SIMCom SIM7600G-H (1e0e:9001): no +QCSQ, the
    LTE quality comes from +CPSI? (reply captured on the robot 2026-10-07)."""
    mod = _link_status()
    reply = ('AT+CPSI?\r\n+CPSI: LTE,Online,466-92,0x639D,134534710,430,'
             'EUTRAN-BAND7,3050,5,5,-103,-998,-722,13\r\n\r\nOK\r\n')
    assert mod.parse_cpsi(reply) == {
        'band': 'EUTRAN-BAND7', 'plmn': '46692',
        'rsrq_db': -10.3, 'rsrp_dbm': -99.8, 'sinr_db': 13.0,
    }
    assert mod.parse_cpsi('+CPSI: NO SERVICE,Online\r\nOK') is None
    assert mod.parse_cpsi('ERROR') is None


def test_modem_at_port_rule_covers_both_modems_and_watchdog_is_tight():
    rules = (DEPLOY / 'udev' / '99-mower.rules').read_text(encoding='utf-8')
    for vid, pid in (('2c7c', '0125'), ('1e0e', '9001')):
        line = [l for l in rules.splitlines()
                if f'ATTRS{{idVendor}}=="{vid}", ATTRS{{idProduct}}=="{pid}"' in l]
        assert len(line) == 1, (vid, pid)
        assert 'ENV{ID_USB_INTERFACE_NUM}=="02"' in line[0]
        assert 'SYMLINK+="lte_at"' in line[0]
    lte = (DEPLOY / 'host' / 'mower-lte.sh').read_text(encoding='utf-8')
    assert 'POLL_S=5\n' in lte
