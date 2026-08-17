#!/usr/bin/env bash
#
# record_dataset.sh -- graba SOLO los topicos crudos del EOD-AV.
#
# ALTERNATIVA a `ros2 bag record -a`, y la recomendada: `-a` no avisa cuando el
# workspace no esta sourceado y deja el bag sin radar en silencio (ver [1]).
#
# Pensado para usarse junto a:
#     ros2 launch eod_av_launch all_sensors.launch.py mode:=record
#
# Por que una lista explicita y no `ros2 bag record -a`:
#   -a arrastraria /rosout, /parameter_events, y sobre todo los topicos
#   DERIVADOS (nubes del radar, markers, /gps_path...), que son reconstruibles
#   a partir del crudo. En modo record esos nodos ni siquiera corren, pero la
#   lista explicita deja documentado que entra al dataset y que no.
#
# Uso:
#   ./record_dataset.sh [carpeta_destino]
#
set -euo pipefail

OUT="${1:-eodav_$(date +%Y%m%d_%H%M%S)}"

# --- Comprobaciones previas -------------------------------------------------

# [1] EL WORKSPACE TIENE QUE ESTAR SOURCEADO. Esto ABORTA, no avisa.
#
# Si no lo esta, `ros2 bag record` no puede resolver los tipos de mensaje
# propios y SALTEA esos topicos con un warning que se pierde entre cientos de
# lineas. El bag sale sin el radar y sin /lidar_ptp, y no te enteras hasta que
# lo abris. Paso en campo -- 62 s de grabacion, 9,5 GiB, cero radar:
#
#   [WARN] Topic '/unfiltered_radar_packet_1' has unknown type
#          'ars430_ros_publisher/msg/RadarPacket'. Only topics with known type
#          are supported. Reason: 'package 'ars430_ros_publisher' not found,
#          searching: [/opt/ros/jazzy]        <-- SOLO /opt/ros: falta el source
#
# Un dataset incompleto es peor que no grabar: no se nota, y cuando se nota ya
# no se puede repetir la salida a campo.
FALTAN_PKGS=()
for p in ars430_ros_publisher hesai_ros_driver; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || FALTAN_PKGS+=("$p")
done
if [[ ${#FALTAN_PKGS[@]} -gt 0 ]]; then
  echo "ERROR: no se resuelven los paquetes de mensajes: ${FALTAN_PKGS[*]}" >&2
  echo >&2
  echo "  Esta terminal no tiene el workspace sourceado. Si grabaras asi, los" >&2
  echo "  topicos de esos paquetes se SALTEAN en silencio y el bag sale" >&2
  echo "  incompleto (sin radar, sin /lidar_ptp)." >&2
  echo >&2
  echo "  Solucion:   source <workspace>/install/setup.bash" >&2
  exit 1
fi

# [2] Sin PTP los timestamps de los sensores no son comparables entre si y el
# dataset no fusiona: avisar antes de grabar, no despues.
if command -v chronyc >/dev/null 2>&1; then
  if ! chronyc sources 2>/dev/null | grep -qE '^\^\*|^#\*'; then
    echo "AVISO: chrony no muestra una fuente sincronizada (^* o #*)." >&2
    echo "       Revisa el PTP/GPS antes de grabar: chronyc sources -v" >&2
  fi
fi

TOPICS=(
  # --- camaras: crudo Bayer (1 B/px). El debayer se hace en playback --------
  /arena_camera_node/enp6s0/image
  /arena_camera_node/enp7s0/image
  /arena_camera_node/enp8s0/image

  # --- LiDAR: la nube, que es autocontenida. Los paquetes crudos pesarian 4x
  # --- menos pero necesitarian el archivo de correccion angular para poder
  # --- reconstruirse: una dependencia externa que vuelve fragil al dataset.
  /lidar_points
  /lidar_ptp            # estado de sincronizacion: diminuto y vale oro despues
  # /lidar_imu existe como topico pero el XT32M2X NO tiene IMU: en el SDK solo
  # llenan imu_config los parsers 1_4, 1_8 y 7_3 (AT128/OT128), no el 6_1 que
  # usa este LiDAR. Por eso sale con 0 mensajes. Se graba igual, cuesta cero, y
  # si alguna vez se cambia de cabezal aparece solo.
  /lidar_imu

  # --- radar: detecciones sin filtrar ---------------------------------------
  /unfiltered_radar_packet_1
  /radar_objects_raw_1
  # La MISMA informacion que /unfiltered_radar_packet_1, pero como PointCloud2.
  # Se graban las dos a proposito: el mensaje propio es la fuente de verdad
  # (lleva SNR, velocidad radial y demas campos que la nube no), y la nube es lo
  # unico que un visor externo sabe dibujar sin conocer el paquete. La nube
  # cuesta poco al lado de las camaras.
  /radar_pointcloud_99

  # --- GNSS -----------------------------------------------------------------
  /fix
  /extended_fix
  # Rumbo de doble antena y velocidad. Los publica gnss_heading_gpsd leyendo
  # las $GNHPR que ya pasan por gpsd (enable_gnss_heading:=true por defecto).
  /gnss/heading
  /gnss/velocity

  # --- TF: /tf_static lleva las extrinsecas (base_link -> cada sensor), sin
  # --- las cuales el bag no fusiona. Siempre esta, en los dos modos.
  # --- /tf (la pose map->base_link) es DERIVADA y esta en OPCIONALES: en
  # --- record no se publica; se regenera en playback con fix_to_path.
  /tf_static
)

# OPCIONALES: no siempre estan, y su ausencia es NORMAL. Van aparte para que no
# disparen el aviso de sensor faltante -- si cada corrida avisa por algo
# esperable, el aviso deja de leerse y el dia que falte el radar tampoco se va
# a leer. Se graban solo si estan publicando.
OPCIONALES=(
  # DERIVADOS: en record NO se publican. Quedan listados para que el mismo
  # script sirva estando corriendo el pipeline (live o playback), donde si
  # existen. En record salen como "no activo" y no se graban, que es lo
  # correcto: son reconstruibles del crudo.
  /tf                       # map->base_link, la publica fix_to_path
  /gps_path                 # trayectoria
  /filtered_radar_packet_1  # detecciones filtradas
  /radar_pointcloud_1       # nube ya filtrada
  /radar/cluster_markers    # clusters DBSCAN
  /visualization_marker
)

# [3] Que de la lista esta publicando AHORA. No aborta: /gnss/velocity y
# /gnss/heading solo existen con enable_um982_heading:=true, y un sensor puede
# tardar en arrancar. Pero se muestra en pantalla, porque la otra forma de
# enterarse de que faltaba un sensor es abriendo el bag al dia siguiente.
VIVOS="$(ros2 topic list 2>/dev/null || true)"
AUSENTES=()
GRABAR=()

echo "==> Grabando en: $OUT"
for t in "${TOPICS[@]}"; do
  GRABAR+=("$t")
  if grep -qxF "$t" <<<"$VIVOS"; then
    echo "    [ok]      $t"
  else
    echo "    [AUSENTE] $t"
    AUSENTES+=("$t")
  fi
done
# Los opcionales se agregan SOLO si estan vivos: pasarle a `ros2 bag record` un
# topico que no existe lo deja esperandolo para siempre.
for t in "${OPCIONALES[@]}"; do
  if grep -qxF "$t" <<<"$VIVOS"; then
    echo "    [ok]      $t  (opcional)"
    GRABAR+=("$t")
  else
    echo "    [ -- ]    $t  (opcional, no activo)"
  fi
done
if [[ ${#AUSENTES[@]} -gt 0 ]]; then
  echo
  echo "AVISO: ${#AUSENTES[@]} topico(s) de la lista no se estan publicando." >&2
  echo "       Si alguno es un sensor que esperabas, cortá ahora (Ctrl-C) y" >&2
  echo "       revisá el launch antes de gastar la salida a campo." >&2
  echo "       Arranco en 5 s..." >&2
  sleep 5
fi

# --storage mcap: mucho mejor throughput que sqlite3 para este caudal.
# --max-cache-size 1 GiB: amortigua las rafagas de escritura.
#
# CORTE POR TIEMPO, NO POR TAMANIO
# Antes era --max-bag-size 4 GiB. El problema del corte por tamanio es que el
# caudal de este equipo NO es constante: se midio ~153 MB/s con las tres
# camaras + LiDAR (9,5 GiB en 62 s), pero baja mucho si se apaga una camara.
# Con 4 GiB fijos, un trozo puede cubrir 25 s o varios minutos segun la
# configuracion, y no se sabe sin abrirlo.
#
# Con corte por tiempo cada trozo cubre SIEMPRE el mismo intervalo de mundo:
# se puede saltar a un momento de la salida a campo mirando los nombres, y una
# corrupcion cuesta un intervalo conocido en vez de "lo que hubiera entrado".
# A 153 MB/s, 4 min son ~37 GiB por trozo.
RECORD_SPLIT_S="${RECORD_SPLIT_S:-240}"   # 4 minutos
echo "    (corte cada ${RECORD_SPLIT_S}s; cambialo con RECORD_SPLIT_S=<segundos>)"

exec ros2 bag record \
  --storage mcap \
  --max-cache-size 1073741824 \
  --max-bag-duration "$RECORD_SPLIT_S" \
  -o "$OUT" \
  "${GRABAR[@]}"
