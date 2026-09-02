"""
Nombrado y ruteo de archivos para la captura cruda de PPK, sin dependencias de ROS.

Vive aparte (como nmea_heading.py) para poder testearlo sin levantar un nodo.
El formato de nombre de sesion (`eodav_%Y%m%d_%H%M%S`) copia a proposito el de
eod_av_launch/scripts/record_dataset.sh (`date +%Y%m%d_%H%M%S`), que es el
unico precedente de nombrado de sesion en el repo -- asi el .raw y la carpeta
del bag de una misma corrida se identifican a simple vista. Hora local, no
UTC: record_dataset.sh tampoco usa UTC.
"""
from datetime import datetime
import os
from typing import Optional

_SESSION_TIMESTAMP_FORMAT = 'eodav_%Y%m%d_%H%M%S'
_RAW_FILE_SUFFIX = '_um982.raw'


def default_session_name(now: Optional[datetime] = None) -> str:
    """Genera un nombre de sesion con la hora actual (o `now`, para tests)."""
    if now is None:
        now = datetime.now()
    return now.strftime(_SESSION_TIMESTAMP_FORMAT)


def resolve_session_name(session_name_param: str, now: Optional[datetime] = None) -> str:
    """'' -> nombre autogenerado; cualquier otra cosa se devuelve intacta."""
    if session_name_param:
        return session_name_param
    return default_session_name(now)


def raw_file_path(raw_log_dir: str, session_name: str) -> str:
    """Ruta del archivo crudo para una sesion, compatible con `convbin -ot <sesion>_um982.raw`."""
    return os.path.join(raw_log_dir, f'{session_name}{_RAW_FILE_SUFFIX}')
