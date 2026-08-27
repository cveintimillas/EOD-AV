"""Tests del mapeo Bayer -> RGB del debayer. Corren sin ROS ni hardware.

    colcon test --packages-select eod_av_launch
    # o, sin workspace:
    python3 -m pytest eod_av_launch/test/test_debayer.py -v

Que se testea y por que
-----------------------
El nombre del patron Bayer de ROS y el de OpenCV NO coinciden: ROS nombra el
bloque 2x2 empezando arriba a la izquierda, OpenCV lo nombra a partir del
segundo pixel de la segunda fila. El resultado es que la tabla correcta parece
cruzada (`bayer_rggb8 -> COLOR_BayerBG2RGB`) y es facilisimo "corregirla" mal:
el sintoma es una imagen que se ve perfecta salvo que los rojos y los azules
estan intercambiados, algo que en una escena de exterior puede pasar
desapercibido bastante tiempo.

Estos tests construyen un mosaico sintetico con valores R/G/B conocidos para
cada uno de los 4 patrones y verifican que salga exactamente ese color. Si
alguien "arregla" la tabla, esto falla en el acto.

No se testea la calidad de la interpolacion (eso es cosa de OpenCV), solo que
cada canal termine donde corresponde.
"""

import cv2

import numpy as np

import pytest

from eod_av_launch.debayer_node import _BAYER_TO_CV

# Valores bien separados: si dos canales se cruzan, se nota.
RED, GREEN, BLUE = 200, 100, 40

# encoding de ROS -> como se ve el bloque 2x2 del mosaico, leido (0,0) (0,1)
# (1,0) (1,1). Es la definicion del encoding, no una eleccion.
PATTERN_OF = {
    'bayer_rggb8': 'rggb',
    'bayer_bggr8': 'bggr',
    'bayer_gbrg8': 'gbrg',
    'bayer_grbg8': 'grbg',
}


def _mosaic(pattern, tiles=(8, 8)):
    """Mosaico de color uniforme (RED, GREEN, BLUE) con el patron dado."""
    value = {'r': RED, 'g': GREEN, 'b': BLUE}
    block = np.zeros((2, 2), dtype=np.uint8)
    for index, color in enumerate(pattern):
        block[divmod(index, 2)] = value[color]
    return np.tile(block, tiles)


@pytest.mark.parametrize('encoding', sorted(PATTERN_OF))
def test_cada_encoding_reconstruye_su_color(encoding):
    """El mapeo ROS -> OpenCV pone cada canal donde corresponde."""
    mosaic = _mosaic(PATTERN_OF[encoding])
    rgb = cv2.cvtColor(mosaic, _BAYER_TO_CV[encoding])

    # Solo el interior: en el borde OpenCV extrapola y el valor es aproximado.
    interior = rgb[2:-2, 2:-2]
    assert np.all(interior[..., 0] == RED), 'canal R'
    assert np.all(interior[..., 1] == GREEN), 'canal G'
    assert np.all(interior[..., 2] == BLUE), 'canal B'


@pytest.mark.parametrize('encoding', sorted(PATTERN_OF))
def test_el_patron_equivocado_cruza_los_canales(encoding):
    """Leer un mosaico RGGB con otro patron NO da el mismo color.

    Es la contraprueba del test anterior: confirma que estos tests fallarian
    de verdad si la tabla estuviera mal, en vez de pasar por casualidad.
    """
    rgb = cv2.cvtColor(_mosaic('rggb'), _BAYER_TO_CV[encoding])
    center = tuple(int(v) for v in rgb[4, 4])
    if encoding == 'bayer_rggb8':
        assert center == (RED, GREEN, BLUE)
    else:
        assert center != (RED, GREEN, BLUE)


def test_estan_los_cuatro_patrones_de_8_bits():
    """La tabla cubre los 4 encodings Bayer de 8 bits de sensor_msgs."""
    assert set(_BAYER_TO_CV) == set(PATTERN_OF)


def test_salida_rgb_no_bgr():
    """Las constantes convierten a RGB (que es lo que dice publicar el nodo).

    Un mosaico rojo puro tiene que salir (255, 0, 0). Con las variantes 2BGR
    saldria (0, 0, 255) y RViz mostraria todo con los colores invertidos.
    """
    mosaic = _mosaic('rggb')
    mosaic[:] = 0
    mosaic[0::2, 0::2] = 255          # solo los fotositos rojos, al maximo
    rgb = cv2.cvtColor(mosaic, _BAYER_TO_CV['bayer_rggb8'])
    r, g, b = (int(v) for v in rgb[4, 4])
    assert (r, g, b) == (255, 0, 0)
