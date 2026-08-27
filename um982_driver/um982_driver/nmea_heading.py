"""Parseo de las sentencias de rumbo del receptor GNSS, sin dependencias de ROS.

Vive aparte para poder testearlo sin levantar un nodo ni tener gpsd corriendo:
test/test_nmea_heading.py lo importa y lo valida contra sentencias reales
capturadas del equipo.

DE DONDE SALEN ESTAS SENTENCIAS
El receptor ya las emite por el mismo puerto serie que gpsd tiene abierto para
alimentar el refclock NMEA+PPS de chrony. Medido con `gpspipe -r`: ~10 Hz de
$GNHPR y $GNTHS, mezcladas con GGA/RMC/GSV. gpsd no las traduce a nada que ROS
consuma -- gpsd_client publica NavSatFix y GPSFix, y el `track` de GPSFix es
rumbo sobre el suelo (inservible parado), no el rumbo de doble antena.

    $GNHPR,171035.80,202.4681,000.4902,000.0000,5,10,0.00,0999*5D
           |         |        |        |        | |  |    |
           |         |        |        |        | |  |    +-- id de estacion
           |         |        |        |        | |  +------- edad del dif.
           |         |        |        |        | +---------- satelites
           |         |        |        |        +------------ calidad (QF)
           |         |        |        +--------------------- alabeo (roll)
           |         |        +------------------------------ cabeceo (pitch)
           |         +--------------------------------------- rumbo (heading)
           +------------------------------------------------- UTC hhmmss.ss

    $GNTHS,202.4681,A*12
           |        +-- modo: A=autonomo, E=estimado, M=manual, S=simulado,
           |                  V=INVALIDO
           +----------- rumbo verdadero (mismo valor que el campo 2 de HPR)

THS es redundante con HPR y trae menos: se usa HPR, y THS solo para descartar
el dato cuando su modo dice V.
"""
import math
from typing import NamedTuple, Optional, Tuple

# Calidad del campo 5 de HPR. Convencion Unicore, igual que la de GGA.
QF_SIN_SOLUCION = 0
QF_RTK_FIJO = 4
QF_RTK_FLOTANTE = 5


class Hpr(NamedTuple):
    """Una sentencia $--HPR ya parseada y validada."""

    tod_s: float           # UTC segundos desde medianoche (del receptor)
    heading_deg: float     # rumbo verdadero, horario desde el norte
    pitch_deg: float
    roll_deg: float        # ver roll_medido: con doble antena viene siempre 0
    quality: int
    satellites: int

    @property
    def roll_medido(self) -> bool:
        """False cuando el roll no es una medicion.

        Un sistema de DOS antenas tiene una sola linea base: puede resolver
        rumbo y cabeceo, pero NO el giro alrededor de esa linea. El receptor
        rellena el campo con 0.0000 en vez de dejarlo vacio, asi que publicarlo
        como si fuera medido afirma una lectura que no existe. Se marca como no
        disponible en la covarianza.
        """
        return False

    @property
    def usable(self) -> bool:
        """False si el receptor dice que no tiene solucion de rumbo."""
        return self.quality != QF_SIN_SOLUCION


def checksum_ok(linea: str) -> bool:
    """Valida el checksum NMEA (XOR de todo lo que hay entre '$' y '*')."""
    if not linea.startswith('$') or '*' not in linea:
        return False
    cuerpo, _, resto = linea[1:].partition('*')
    if len(resto) < 2:
        return False
    try:
        esperado = int(resto[:2], 16)
    except ValueError:
        return False
    calculado = 0
    for ch in cuerpo:
        calculado ^= ord(ch)
    return calculado == esperado


def _tod(campo: str) -> Optional[float]:
    """hhmmss.ss -> segundos desde medianoche UTC."""
    if len(campo) < 6:
        return None
    try:
        return int(campo[0:2]) * 3600 + int(campo[2:4]) * 60 + float(campo[4:])
    except ValueError:
        return None


def parse_hpr(linea: str) -> Optional[Hpr]:
    """Parsea una $--HPR. Devuelve None si no lo es, o si viene corrupta.

    Acepta cualquier prefijo de dos letras (GN, GP, GB...): el talker depende
    de que constelaciones entraron en la solucion y no es estable.
    """
    linea = linea.strip()
    if len(linea) < 7 or not linea.startswith('$') or linea[3:6] != 'HPR':
        return None
    if not checksum_ok(linea):
        return None
    campos = linea[1:linea.index('*')].split(',')
    if len(campos) < 7:
        return None
    tod = _tod(campos[1])
    if tod is None:
        return None
    try:
        return Hpr(
            tod_s=tod,
            heading_deg=float(campos[2]),
            pitch_deg=float(campos[3]),
            roll_deg=float(campos[4]),
            quality=int(campos[5]),
            satellites=int(campos[6]),
        )
    except ValueError:
        return None


def ths_invalida(linea: str) -> bool:
    """Indica si es una $--THS cuyo modo dice 'V' (rumbo no valido)."""
    linea = linea.strip()
    if len(linea) < 7 or not linea.startswith('$') or linea[3:6] != 'THS':
        return False
    if not checksum_ok(linea):
        return False
    campos = linea[1:linea.index('*')].split(',')
    return len(campos) >= 3 and campos[2].strip().upper() == 'V'


def epoch_desde_tod(tod_s: float, ahora_epoch: float) -> float:
    """Combina la hora del dia del receptor con la fecha del reloj del host.

    HPR trae hora UTC pero NO fecha. Se toma la fecha del host (disciplinado por
    el PPS del mismo receptor, ~3 us) y se elige el dia que deja la diferencia
    mas chica: asi el cruce de medianoche no tira el timestamp 24 h.
    """
    dia = 86400.0
    medianoche = ahora_epoch - (ahora_epoch % dia)
    candidatos = (medianoche + tod_s - dia, medianoche + tod_s, medianoche + tod_s + dia)
    return min(candidatos, key=lambda t: abs(t - ahora_epoch))


def euler_deg_to_quaternion(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (degrees, REP-103 body frame) to a quaternion (x, y, z, w)."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw
