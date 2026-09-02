"""Tests del nombrado/ruteo de archivos para la captura cruda de PPK."""
from datetime import datetime
import re

from um982_driver.raw_log_session import (default_session_name, raw_file_path,
                                          resolve_session_name)


def test_default_session_name_matches_eodav_pattern():
    assert re.match(r'^eodav_\d{8}_\d{6}$', default_session_name())


def test_default_session_name_usa_datetime_inyectado():
    assert default_session_name(datetime(2026, 8, 27, 14, 30, 5)) == 'eodav_20260827_143005'


def test_resolve_session_name_vacio_genera_default():
    ahora = datetime(2026, 8, 27, 14, 30, 5)
    assert resolve_session_name('', ahora) == default_session_name(ahora)


def test_resolve_session_name_pasa_nombre_explicito_intacto():
    assert resolve_session_name('bench_test_1') == 'bench_test_1'


def test_raw_file_path_basico():
    assert (raw_file_path('/data/eod_av/gnss_raw', 'eodav_20260827_143005')
            == '/data/eod_av/gnss_raw/eodav_20260827_143005_um982.raw')


def test_raw_file_path_con_slash_final_en_el_dir():
    assert (raw_file_path('/data/eod_av/gnss_raw/', 'eodav_20260827_143005')
            == '/data/eod_av/gnss_raw/eodav_20260827_143005_um982.raw')


def test_raw_file_path_sufijo_compatible_con_convbin():
    # PPK_IMPLEMENTATION.md post-procesa con:
    #   convbin -r unicore -v 3.04 -od -os -oi -ot <sesion>_um982.raw
    # Si este sufijo cambia sin querer, convbin deja de encontrar el archivo.
    assert raw_file_path('/x', 'sesion').endswith('_um982.raw')
