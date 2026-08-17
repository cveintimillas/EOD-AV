"""Tests del filtro de movimiento de fix_to_path (sin hardware, sin GPS).

    colcon test --packages-select gps_bringup
    # o directamente:
    python3 -m pytest gps_bringup/test/test_fix_to_path.py -v

Que se testea
-------------
Un receptor GNSS QUIETO no devuelve dos veces la misma posicion: cada fix trae
ruido. Acumular todos en el Path dibuja una maranha de varios metros sin que
nada se haya movido -- es lo que se veia en RViz. Estos tests generan fixes
sinteticos con ruido conocido y verifican las dos propiedades que importan:

  1. quieto  -> el Path NO crece (la maranha desaparece)
  2. movil   -> la trayectoria se conserva, en longitud y en detalle

Las dos juntas: un filtro que borra la maranha pero tambien borra el movimiento
real no sirve, y es facil escribirlo sin darse cuenta.

Se testean los tres regimenes de ruido que se dan en la practica: receptor que
no reporta covarianza, receptor que la reporta, y RTK fijo (centimetros).
"""

import importlib.util
import math
import pathlib
import random
from collections import deque

import pytest

_SCRIPT = pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'fix_to_path.py'

R = 6378137.0            # radio ecuatorial WGS84 (m), igual que el nodo
LAT0, LON0 = -34.6037, -58.3816


def _load_module():
    spec = importlib.util.spec_from_file_location('fix_to_path', _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def fix_to_path():
    return _load_module()


class _Header:
    stamp = 'stamp'


def _fix(navsatfix_cls, dx=0.0, dy=0.0, sigma=None, noise=0.0, rng=None):
    """NavSatFix a dx/dy metros del origen, con ruido gaussiano de `noise` m."""
    msg = navsatfix_cls()
    msg.header = _Header()
    nx = rng.gauss(0.0, noise) if (rng and noise) else 0.0
    ny = rng.gauss(0.0, noise) if (rng and noise) else 0.0
    msg.latitude = LAT0 + math.degrees((dy + ny) / R)
    msg.longitude = LON0 + math.degrees(
        (dx + nx) / (R * math.cos(math.radians(LAT0))))
    if sigma is not None:
        msg.position_covariance_type = 2      # DIAGONAL_KNOWN
        msg.position_covariance = [sigma ** 2, 0.0, 0.0,
                                   0.0, sigma ** 2, 0.0,
                                   0.0, 0.0, sigma ** 2]
    return msg


def _node(fix_to_path, **overrides):
    node = fix_to_path.FixToPath()
    node.child = 'base_link'
    for key, value in overrides.items():
        setattr(node, key, value)
    if 'smooth_window' in overrides:
        # El maxlen del deque se fija en __init__ a partir del parametro, asi
        # que un setattr posterior no lo cambia: hay que rehacerlo. (Este
        # detalle hizo fallar la contraprueba: seguia promediando con ventana
        # 10 y "sin filtro" no era sin filtro.)
        node._window = deque(maxlen=max(1, int(overrides['smooth_window'])))
    return node


def _extent(node):
    """Distancia maxima entre dos puntos del Path (tamanio de la maranha)."""
    pts = [(p.pose.position.x, p.pose.position.y) for p in node.path.poses]
    if len(pts) < 2:
        return 0.0
    return max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in pts for b in pts)


# --- quieto: la maranha tiene que desaparecer -----------------------------

@pytest.mark.parametrize('sigma,noise', [
    (None, 1.0),      # el receptor no reporta covarianza -> fallback_sigma_m
    (1.5, 1.0),       # la reporta
    (0.02, 0.02),     # RTK fijo, centimetros
])
def test_quieto_no_dibuja_maranha(fix_to_path, sigma, noise):
    rng = random.Random(7)
    node = _node(fix_to_path, origin_settle_fixes=20)
    from sensor_msgs.msg import NavSatFix
    for _ in range(600):                      # 60 s a 10 Hz
        node.cb(_fix(NavSatFix, sigma=sigma, noise=noise, rng=rng))
    # Medido sobre 30 semillas: 1-3 puntos (mediana 2) contra 600 sin filtro.
    # No queda en exactamente 1 porque el promedio movil sigue haciendo un
    # random walk lento; lo que importa es el orden de magnitud.
    assert len(node.path.poses) <= 5
    # Y esos pocos puntos abarcan a lo sumo ~3 m, contra 6-7,7 m sin filtro.
    assert _extent(node) < 4.0


def test_sin_filtro_si_dibuja_maranha(fix_to_path):
    """Contraprueba: con el filtro apagado el problema reaparece.

    Si este test empieza a fallar es que el filtro dejo de ser opcional, o que
    el generador de ruido dejo de generar ruido -- en cualquiera de los dos
    casos los tests de arriba estarian pasando por el motivo equivocado.
    """
    from sensor_msgs.msg import NavSatFix
    rng = random.Random(7)
    node = _node(fix_to_path, min_distance_m=0.0, distance_sigma_k=0.0,
                 origin_settle_fixes=1, smooth_window=1)
    for _ in range(600):
        node.cb(_fix(NavSatFix, noise=1.0, rng=rng))
    assert len(node.path.poses) == 600
    assert _extent(node) > 5.0        # medido: 6,0-7,7 m sobre 30 semillas


# --- movil: la trayectoria real NO se puede perder ------------------------

def test_movimiento_se_conserva_sin_covarianza(fix_to_path):
    from sensor_msgs.msg import NavSatFix
    rng = random.Random(7)
    node = _node(fix_to_path, origin_settle_fixes=20)
    for i in range(600):                      # 0,1 m por fix -> 60 m
        node.cb(_fix(NavSatFix, dx=i * 0.1, noise=1.0, rng=rng))
    xs = [p.pose.position.x for p in node.path.poses]
    assert max(xs) - min(xs) > 55.0           # la longitud se conserva
    assert len(node.path.poses) > 10          # y hay puntos suficientes


def test_rtk_conserva_el_detalle_fino(fix_to_path):
    """Con RTK el umbral baja solo y no se pierde la trayectoria de detalle."""
    from sensor_msgs.msg import NavSatFix
    rng = random.Random(7)
    node = _node(fix_to_path, origin_settle_fixes=20)
    for i in range(600):                      # 0,05 m por fix -> 30 m
        node.cb(_fix(NavSatFix, dx=i * 0.05, sigma=0.02, noise=0.02, rng=rng))
    xs = [p.pose.position.x for p in node.path.poses]
    assert max(xs) - min(xs) > 28.0
    # Muchos mas puntos que en el caso sin RTK: el umbral se adapto al ruido.
    assert len(node.path.poses) > 80


# --- origen promediado ----------------------------------------------------

def test_el_primer_fix_malo_no_arrastra_el_origen(fix_to_path):
    """Un primer fix a 50 m no debe dejar el frame 'map' desplazado 50 m."""
    from sensor_msgs.msg import NavSatFix
    rng = random.Random(7)
    node = _node(fix_to_path, origin_settle_fixes=20)
    node.cb(_fix(NavSatFix, dx=40.0, dy=30.0))          # outlier a 50 m
    for _ in range(19):
        node.cb(_fix(NavSatFix, noise=0.5, rng=rng))
    lat0, lon0 = node.origin
    error = math.hypot(
        math.radians(lon0 - LON0) * R * math.cos(math.radians(LAT0)),
        math.radians(lat0 - LAT0) * R)
    assert error < 5.0                        # sin promediar serian 50 m


def test_sin_fix_no_publica_tf(fix_to_path):
    """Con lat/lon en NaN no hay pose: no se publica TF ni se fija origen."""
    from sensor_msgs.msg import NavSatFix
    node = _node(fix_to_path)
    msg = NavSatFix()
    msg.header = _Header()
    msg.latitude = float('nan')
    msg.longitude = float('nan')
    node.cb(msg)
    assert node.origin is None
    assert not node.br.sent
