"""Tests del parseo de rumbo, contra sentencias REALES capturadas del equipo."""
from um982_driver.nmea_heading import (Hpr, checksum_ok, epoch_desde_tod,
                                       parse_hpr, ths_invalida)

# Capturadas con `gpspipe -r` en el vehiculo. No inventadas.
REALES = [
    '$GNTHS,202.4681,A*12',
    '$GNHPR,171035.80,202.4681,000.4902,000.0000,5,10,0.00,0999*5D',
    '$GNTHS,202.6144,A*1E',
    '$GNHPR,171035.90,202.6144,001.3018,000.0000,5,10,0.00,0999*54',
    '$GNTHS,203.5232,A*1E',
    '$GNHPR,171036.00,203.5232,-00.2631,000.0000,5,10,0.00,0999*4E',
]


def test_checksum_de_las_sentencias_reales():
    for linea in REALES:
        assert checksum_ok(linea), linea


def test_checksum_detecta_corrupcion():
    # Un digito cambiado en el rumbo tiene que invalidar la sentencia.
    assert not checksum_ok('$GNHPR,171035.80,202.4682,000.4902,000.0000,5,10,0.00,0999*5D')
    assert not checksum_ok('$GNHPR,171035.80,202.4681,000.4902,000.0000,5,10,0.00,0999*FF')
    assert not checksum_ok('$GNHPR,sin,asterisco')
    assert not checksum_ok('')


def test_parse_hpr_campos():
    h = parse_hpr(REALES[1])
    assert h is not None
    assert h.heading_deg == 202.4681
    assert h.pitch_deg == 0.4902
    assert h.roll_deg == 0.0
    assert h.quality == 5
    assert h.satellites == 10
    # 17:10:35.80 UTC
    assert abs(h.tod_s - (17 * 3600 + 10 * 60 + 35.80)) < 1e-6


def test_parse_hpr_pitch_negativo():
    # '-00.2631' con ceros a la izquierda: el caso que rompe un parser ingenuo.
    h = parse_hpr(REALES[5])
    assert h is not None
    assert h.pitch_deg == -0.2631


def test_roll_nunca_se_considera_medido():
    # Con dos antenas hay una sola linea base: el roll no es observable y el
    # receptor rellena 0.0000. Publicarlo como medido seria inventar el dato.
    for linea in REALES:
        h = parse_hpr(linea)
        if h is not None:
            assert h.roll_deg == 0.0
            assert h.roll_medido is False


def test_calidad():
    assert parse_hpr(REALES[1]).usable          # QF=5, flotante pero usable
    sin_sol = '$GNHPR,171035.80,202.4681,000.4902,000.0000,0,10,0.00,0999'
    sin_sol += '*%02X' % _cks(sin_sol)
    assert parse_hpr(sin_sol).usable is False   # QF=0, sin solucion


def test_parse_hpr_rechaza_lo_que_no_es_hpr():
    assert parse_hpr('$GNTHS,202.4681,A*12') is None
    assert parse_hpr('$GNGGA,174522.60,0208.66,S,07957.97,W,1,24,0.6,8.0,M,,M,,*5A') is None
    assert parse_hpr('') is None
    assert parse_hpr('basura') is None
    assert parse_hpr('$GN') is None


def test_acepta_cualquier_talker():
    # El prefijo depende de que constelaciones entraron en la solucion.
    base = 'GPHPR,171035.80,202.4681,000.4902,000.0000,5,10,0.00,0999'
    linea = '$' + base + '*%02X' % _cks('$' + base)
    assert parse_hpr(linea) is not None


def test_ths_invalida():
    assert not ths_invalida('$GNTHS,202.4681,A*12')      # A = autonomo, valida
    mala = '$GNTHS,202.4681,V'
    mala += '*%02X' % _cks(mala)
    assert ths_invalida(mala)
    assert not ths_invalida('$GNHPR,171035.80,202.4681,000.4902,000.0000,5,10,0.00,0999*5D')


def test_epoch_desde_tod_caso_normal():
    # host a las 17:10:36 UTC del 2026-08-06; receptor dice 17:10:35.80
    ahora = 1785949836.0
    tod = ahora % 86400 - 0.2
    assert abs(epoch_desde_tod(tod, ahora) - (ahora - 0.2)) < 1e-6


def test_epoch_desde_tod_cruce_de_medianoche():
    # host recien pasada la medianoche; el receptor todavia reporta 23:59:59.9
    ahora = 1785974400.0 + 0.05          # 00:00:00.05 UTC
    tod = 86399.9                        # 23:59:59.9
    esperado = ahora - 0.15
    assert abs(epoch_desde_tod(tod, ahora) - esperado) < 1e-6
    # y al reves: host antes de medianoche, receptor ya en 00:00:00.1
    ahora = 1785974400.0 - 0.05
    assert abs(epoch_desde_tod(0.1, ahora) - (ahora + 0.15)) < 1e-6


def _cks(linea):
    cuerpo = linea[1:].partition('*')[0]
    c = 0
    for ch in cuerpo:
        c ^= ord(ch)
    return c


def test_hpr_es_inmutable():
    h = Hpr(1.0, 2.0, 3.0, 0.0, 5, 10)
    try:
        h.heading_deg = 9.0
    except AttributeError:
        return
    raise AssertionError('Hpr deberia ser inmutable')
