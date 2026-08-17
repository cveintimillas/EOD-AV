#!/usr/bin/env bash
#
# setup_ptp_sync.sh — EOD-AV
# Aprovisiona sincronización PTP Nivel A para 3x Triton + 1x Hesai,
# referenciados a GPS vía simpleRTK3B Compass (PPS + NMEA).
#
# ARQUITECTURA REAL (confirmada con ethtool -i / lspci, no es la que se asumió al principio):
#   - Las 3 Triton están en una tarjeta multipuerto Realtek RTL8125 (driver r8169),
#     que NO expone reloj de hardware PTP (PHC). Van con timestamping por SOFTWARE.
#   - El Hesai está en la NIC onboard Intel I225-V (driver igc), que SÍ tiene PHC real.
#     Va con timestamping por HARDWARE + phc2sys.
#   Por eso hay DOS instancias de ptp4l (una por modo de timestamping) y UN solo
#   phc2sys (solo para el puerto que sí tiene PHC que disciplinar).
#
# GPS — UN SOLO ADAPTADOR para NMEA + PPS (no dos):
#   El simpleRTK3B Compass tiene su propio puerto USB nativo (solo alimentación +
#   passthrough, no expone TPS/PPS), pero NMEA y PPS se toman de un adaptador
#   USB-serial FTDI FT232R aparte, cableado así: TPS→DCD, TX→RX, GND→GND (los
#   tres al mismo adaptador). Con los dos en el mismo device, gpsd correlaciona
#   el fix y el pulso PPS automáticamente vía sysfs (SHM 0 = fix, SHM 1 = PPS).
#   Se probó primero con NMEA por el puerto USB nativo del compass y PPS por un
#   segundo adaptador separado: gpsd no puede correlacionar dos devices USB
#   distintos, y el PPS quedaba con un corrimiento de ~367ms fijo pero espurio.
#   Un solo adaptador con las tres señales es la forma preferida.
#
#   Si aun así el NMEA sale por el USB del receptor y el PPS por un FTDI aparte,
#   está soportado: GPS_NMEA_SERIAL=<serial>. En ese modo el PPS NO pasa por
#   gpsd (por lo de arriba) sino por el driver PPS nativo de chrony con
#   `lock NMEA`. Ver el bloque de configuración más abajo.
#
# Qué hace:
#   1. Instala linuxptp, chrony, gpsd y herramientas relacionadas.
#   2. Verifica hardware timestamping en cada interfaz.
#   3. Instala una regla udev que fija el adaptador USB-serial del GPS a un
#      nombre persistente /dev/gps_pps por número de serie — ttyUSBn cambia de
#      numeración al reconectar o reordenar el enumerado USB (pasó varias veces
#      en pruebas y rompía los servicios en silencio).
#   4. Activa pps-ldisc sobre ese adaptador (como servicio persistente).
#   5. Reemplaza el gpsd.service/gpsd.socket empaquetado (poco confiable para pasar
#      DEVICES por env-var) por un gpsd-eodav.service propio, con el ExecStart explícito.
#   6. Agrega los refclocks a chrony (SHM 0 fix + SHM 1 PPS, correlacionados por gpsd).
#   7. Genera ptp4l-hw.conf (Hesai, hardware) y ptp4l-sw.conf (3x Triton, software).
#   8. Crea servicios systemd: ptp4l-hw, ptp4l-sw, y phc2sys (solo para el puerto con PHC).
#   9. Da cap_net_raw al binario del radar (captura libpcap sin correr como root).
#
# ANTES DE CORRER: confirmá con `ip link show`, `ethtool -i <iface>` y `lspci -nn | grep -i eth`
# que los nombres de interfaz de abajo coinciden con tu PC real — pueden variar.
# Confirmá también el número de serie del adaptador USB-serial del GPS con:
#   udevadm info -q property -n /dev/ttyUSBx | grep ID_SERIAL_SHORT
#
# Uso: sudo bash setup_ptp_sync.sh

set -euo pipefail

# Directorio de este script. Se usa para ubicar el workspace (paso 9) sin
# depender de variables de entorno: bajo sudo no hay AMENT_PREFIX_PATH ni ros2.
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ============================================================
# CONFIGURACIÓN — EDITAR ESTOS VALORES ANTES DE EJECUTAR
# ============================================================
CAM1_IFACE="enp6s0"            # Triton 1 — Realtek RTL8125, sin PHC (software timestamping)
CAM2_IFACE="enp7s0"            # Triton 2 — Realtek RTL8125, sin PHC (software timestamping)
CAM3_IFACE="enp8s0"            # Triton 3 — Realtek RTL8125, sin PHC (software timestamping)
LIDAR_IFACE="enp11s0"          # Hesai — Intel I225-V onboard, CON PHC (hardware timestamping)
# ID_SERIAL_SHORT del adaptador USB-serial del GPS (TPS->DCD, TX->RX, GND->GND).
# VACIO = autodetectar: si hay exactamente un ttyUSB*/ttyACM* conectado se usa
# ese. Con varios, el script aborta y los lista para que elijas.
#
# Estaba fijo en "A5069RR4" (un FT232R). Hardcodear el serial hace que al
# cambiar de adaptador la regla udev deje de matchear, /dev/gps_pps no se cree
# nunca, y todo lo de abajo falle EN SILENCIO: gps-pps-ldattach y gpsd-eodav no
# arrancan, y el gpsd.socket de Ubuntu contesta igual en el puerto 2947 pero sin
# ningun device, con lo cual /fix queda mudo sin un solo error visible.
# Se puede forzar por entorno:  GPS_SERIAL=D30HIC4U ./setup_ptp_sync.sh
GPS_SERIAL="${GPS_SERIAL:-}"
GPS_DEV="/dev/gps_pps"         # symlink persistente (via udev, ver paso 3) -> ese adaptador

# TOPOLOGIA DE DOS ADAPTADORES (opcional).
#
# Por defecto se asume UN adaptador que lleva NMEA (TX->RX) y PPS (TPS->DCD):
# es lo mejor, porque gpsd correlaciona el fix con su propio pulso via sysfs.
#
# Si el NMEA sale por el USB nativo del receptor y el PPS por un FTDI aparte,
# poné aca el serial del adaptador de NMEA. gpsd NO puede correlacionar dos
# devices USB distintos (se probo: el PPS quedaba con un corrimiento fijo pero
# espurio de ~370 ms), asi que en ese modo el PPS NO va por gpsd: se usa el
# driver PPS nativo de chrony con `lock NMEA`, que es la forma correcta.
#
#   GPS_SERIAL=<el del FTDI con DCD>  GPS_NMEA_SERIAL=<el del receptor>  ./setup_ptp_sync.sh
GPS_NMEA_SERIAL="${GPS_NMEA_SERIAL:-}"
GPS_NMEA_DEV="/dev/gps_nmea"   # symlink al adaptador de NMEA (solo en modo dos adaptadores)
GPS_PPS_SRC="/dev/gps_pps_src" # symlink al /dev/ppsN de ese adaptador (idem)

# GNSS_ONLY=1 -> configurar SOLO la cadena del GNSS (udev + ldattach + gpsd +
# refclocks de chrony) y saltear todo lo de PTP sobre las interfaces de red.
# Sirve para probar el GNSS en el banco con las camaras y el LiDAR
# desconectados: el paso [2/9] lee `ethtool -T` de cada interfaz y, con
# `set -e`, el script abortaba ahi antes de llegar a configurar el GPS.
#   sudo GNSS_ONLY=1 ./setup_ptp_sync.sh
GNSS_ONLY="${GNSS_ONLY:-0}"
PTP_DOMAIN=0

# Transporte PTP para las camaras: auto | L2 | UDPv4.
#
# `auto` (recomendado) = L2 si las interfaces de camara son esclavas de un
# bridge Linux, UDPv4 si no. El motivo esta medido, no supuesto: ver el paso
# [7/9]. Se puede forzar:  CAM_PTP_TRANSPORT=UDPv4 ./setup_ptp_sync.sh
CAM_PTP_TRANSPORT="${CAM_PTP_TRANSPORT:-auto}"

# FIX_BRIDGE_STP=1 -> apagar STP en el bridge de las camaras.
#
# Con tres equipos terminales (una camara por puerto) NO puede haber bucles,
# asi que STP no protege de nada, y su forward_delay hace que tras cada replug
# el puerto pase ~30 s en LISTENING/LEARNING antes de reenviar: las camaras
# "desaparecen" ese rato. Apagarlo es seguro EN ESTA TOPOLOGIA.
#
# Va detras de un flag y no por defecto porque toca la configuracion de red del
# host, que es del usuario y no de este script.
#   sudo FIX_BRIDGE_STP=1 ./setup_ptp_sync.sh
FIX_BRIDGE_STP="${FIX_BRIDGE_STP:-0}"

# Retardo del NMEA, en segundos, que chrony compensa en el refclock SHM 0.
#
# El sentence NMEA llega por USB DESPUES del segundo al que se refiere. Ese
# retardo depende del receptor, del baudrate y del stack USB, asi que no hay un
# valor universal. Como calibrarlo: correr con el PPS andando y mirar
#
#     chronyc sources -v
#
# La linea NMEA muestra el residuo. Si dice p.ej. "+177ms", el retardo real es
# NMEA_OFFSET + 0,177 y hay que subir la variable en esa cantidad.
#
# Con el PPS vivo esto es COSMETICO (el NMEA va con noselect y solo desambigua
# el segundo, para lo cual sobra medio segundo de margen). Importa cuando NO hay
# PPS: ahi el NMEA es la unica fuente y su residuo es el error del reloj.
NMEA_OFFSET="${NMEA_OFFSET:-0.2}"

# --- Calidad anunciada del grandmaster, y cadencia hacia el Hesai -----------
#
# El LiDAR reportaba "PTP: Free Run" en su web UI pese a tener la configuracion
# correcta (Clock Source PTP, 1588v2, UDP/IP, domain 0) y pese a que el
# intercambio E2E cerraba (su Delay_Req contestado en 36 us). Dos causas
# probables, las dos de una linea:
#
# 1. clockClass. El default de linuxptp es 248, que en IEEE-1588 significa
#    literalmente "reloj libre, no trazable a una referencia primaria". Un
#    esclavo que busca una fuente buena puede negarse a engancharse a eso.
#    Aca el grandmaster SI es trazable: chrony lo disciplina con el PPS del
#    GNSS (medido: 4,3 us de RMS) y phc2sys lo pasa al PHC. clockClass 6 =
#    "sincronizado a una referencia primaria", que es la verdad.
#
#    OJO: si el PPS se cae, chrony pasa a NMEA y el 6 deja de ser cierto.
#    linuxptp no lo degrada solo. Para volver al comportamiento conservador:
#    PTP_CLOCK_CLASS=248
#
# 2. logSyncInterval. El Hesai esta configurado con logSyncInterval=1 (un Sync
#    cada 2 s) y linuxptp manda con 0 (uno por segundo). Varias
#    implementaciones exigen que coincidan.
#
# Se aplican SOLO a la instancia del Hesai (ptp4l-hw). La de las camaras se
# deja como esta: enganchan bien con los valores actuales y no hay motivo para
# arriesgar lo unico que ya funciona.
PTP_CLOCK_CLASS="${PTP_CLOCK_CLASS:-6}"
# Calidad declarada del grandmaster. Los defaults de linuxptp son
# clockAccuracy 0xFE y offsetScaledLogVariance 0xFFFF, que significan
# "DESCONOCIDA" las dos. Anunciar clockClass 6 ("sincronizado a una referencia
# primaria") junto con accuracy y varianza desconocidas es CONTRADICTORIO, y un
# esclavo que valida la calidad del maestro puede rechazarlo por eso.
#
#   0x21 = mejor que 25 ns   (realista: el PHC sigue a CLOCK_REALTIME con
#                             offsets de decenas de ns, medido con phc_ctl)
#   0x4E5D = varianza tipica de un grandmaster bueno
#
# MEDIDO: ni `utc_offset` ni `leapfile` logran poner currentUtcOffsetValid ni
# timeTraceable en 1 -- linuxptp 4.0 no expone esos flags como opcion. Asi que
# accuracy y varianza son las unicas dos palancas de calidad que quedan.
PTP_CLOCK_ACCURACY="${PTP_CLOCK_ACCURACY:-0x21}"
PTP_OFFSET_VARIANCE="${PTP_OFFSET_VARIANCE:-0x4E5D}"
PTP_LOG_SYNC_INTERVAL="${PTP_LOG_SYNC_INTERVAL:-1}"
PTP_LOG_ANNOUNCE_INTERVAL="${PTP_LOG_ANNOUNCE_INTERVAL:-1}"
# ============================================================

SW_IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE")
ALL_IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE" "$LIDAR_IFACE")

# Devuelve el nombre del bridge del que `$1` es esclavo, o vacio si no lo es.
#
# El `return 0` final NO es decorativo. Sin el, la funcion terminaba con el
# estado del `[[ ... ]] && echo`, o sea 1 cuando la interfaz NO esta en un
# bridge. Con `set -e`, una asignacion `b=$(bridge_master_of eth0)` toma ese
# estado y ABORTA EL SCRIPT en silencio. Sintoma exacto: el paso [2/9] imprimia
# las lineas de las camaras y el script moria antes de [3/9], sin un solo
# mensaje de error, dejando los .conf viejos en /etc/linuxptp. (Bug introducido
# en v17, encontrado en campo, reproducido y corregido en v20.)
bridge_master_of() {
  local m
  m=$(ip -o link show "$1" 2>/dev/null | sed -n 's/.* master \([^ ]*\).*/\1/p')
  if [[ -n "$m" && -d "/sys/class/net/$m/bridge" ]]; then echo "$m"; fi
  return 0
}

echo "==> [1/9] Instalando paquetes"
# POR QUE NO ES UN `apt update && apt install` PELADO
# `apt update` devuelve != 0 por fallas que NO afectan a la instalacion --
# tipicamente los metadatos dep11/Components (descripciones de appstream), que
# se caen solos cuando el mirror esta sincronizando:
#
#   E: Failed to fetch .../noble-updates/main/dep11/Components-amd64.yml.xz
#      File has unexpected size (180596 != 180220). Mirror sync in progress?
#
# Con `set -e` eso mataba TODO el setup en el paso 1, sin haber tocado nada,
# por un archivo de descripciones. Y en una re-corrida los paquetes ya estan:
# no hay ningun motivo para depender de la red.
#
# Orden: mirar que falta -> solo si falta algo, tocar apt -> tolerar que
# `update` falle (los indices en cache sirven) -> pero NO tolerar que `install`
# falle, porque sin los paquetes nada de lo que sigue tiene sentido.
PKGS=(linuxptp chrony pps-tools setserial gpsd gpsd-clients libcap2-bin)
FALTAN=()
for p in "${PKGS[@]}"; do
  [[ "$(dpkg-query -W -f='${Status}' "$p" 2>/dev/null)" == "install ok installed" ]] \
    || FALTAN+=("$p")
done
if [[ "${SKIP_APT:-0}" == "1" ]]; then
  echo "  (SKIP_APT=1: no toco apt)"
  [[ ${#FALTAN[@]} -gt 0 ]] && echo "  ⚠ pero faltan: ${FALTAN[*]}"
elif [[ ${#FALTAN[@]} -eq 0 ]]; then
  echo "  ✓ los ${#PKGS[@]} paquetes ya estan instalados -- no toco apt"
else
  echo "  faltan ${#FALTAN[@]}: ${FALTAN[*]}"
  if ! apt update; then
    echo "  ⚠ 'apt update' fallo. Si es solo dep11/Components es cosmetico:"
    echo "    sigo con los indices que ya estan en cache."
  fi
  if ! apt install -y "${FALTAN[@]}"; then
    echo "  ✖ no se pudieron instalar: ${FALTAN[*]}" >&2
    echo "    Sin estos paquetes el resto del setup no puede funcionar." >&2
    echo "    Reintenta, o instalalos a mano y volve a correr con SKIP_APT=1." >&2
    exit 1
  fi
fi

echo "==> [2/9] Verificando hardware timestamping en cada interfaz"
if [[ "$GNSS_ONLY" == "1" ]]; then
  echo "  (GNSS_ONLY=1: salteado)"
fi
for i in $([[ "$GNSS_ONLY" == "1" ]] || printf '%s\n' "${ALL_IFACES[@]}"); do
  echo "--- $i ---"
  ethtool -T "$i" 2>/dev/null | grep -E "PTP Hardware Clock|HWTSTAMP" \
    || echo "  ⚠ No se pudo leer $i — confirmá el nombre de interfaz con 'ip link show'"
done
echo "  (Esperado: $LIDAR_IFACE con 'PTP Hardware Clock: 0'; las 3 Triton con 'none')"

# --- Deteccion de bridge sobre las interfaces de camara --------------------
# No es cosmetico: cambia el transporte que hay que usar en ptp4l-sw.conf.
# Ver el bloque de comentarios del paso [7/9] para la medicion que lo motiva.
CAM_BRIDGE=""
if [[ "$GNSS_ONLY" != "1" ]]; then
  for i in "${SW_IFACES[@]}"; do
    b=$(bridge_master_of "$i")
    [[ -n "$b" ]] && { echo "  ℹ $i es esclava del bridge '$b'"; CAM_BRIDGE="$b"; }
  done
  b=$(bridge_master_of "$LIDAR_IFACE")
  if [[ -n "$b" ]]; then
    echo "  ⚠ $LIDAR_IFACE (LiDAR, timestamping por HARDWARE) es esclava del bridge '$b'." >&2
    echo "    El timestamping por hardware se toma en la NIC, asi que el PHC sigue" >&2
    echo "    sirviendo, pero el Delay_Req del Hesai puede no llegar a ptp4l-hw por" >&2
    echo "    el mismo motivo que en las camaras. Sacalo del bridge si podes." >&2
  fi
  if [[ -n "$CAM_BRIDGE" && "$(cat "/sys/class/net/$CAM_BRIDGE/bridge/stp_state" 2>/dev/null)" == "1" ]]; then
    if [[ "$FIX_BRIDGE_STP" == "1" ]]; then
      echo "  Apagando STP en '$CAM_BRIDGE' (FIX_BRIDGE_STP=1)"
      # Dos capas: el sysfs surte efecto ya, y NetworkManager lo hace
      # persistente. Si el bridge no lo maneja NM, el nmcli falla sin ruido y
      # queda solo el cambio en caliente, que igual se reporta abajo.
      echo 0 > "/sys/class/net/$CAM_BRIDGE/bridge/stp_state" 2>/dev/null || true
      NM_CON=$(nmcli -g GENERAL.CONNECTION device show "$CAM_BRIDGE" 2>/dev/null || true)
      if [[ -n "$NM_CON" ]]; then
        nmcli con mod "$NM_CON" bridge.stp no 2>/dev/null \
          && echo "  ✓ persistente en la conexion NetworkManager '$NM_CON'" \
          || echo "  ⚠ no se pudo persistir en NetworkManager ('$NM_CON')" >&2
      else
        echo "  ⚠ '$CAM_BRIDGE' no lo maneja NetworkManager: el cambio NO es persistente." >&2
        echo "    Se revierte al reiniciar. Fijalo donde tengas definido el bridge." >&2
      fi
      if [[ "$(cat "/sys/class/net/$CAM_BRIDGE/bridge/stp_state" 2>/dev/null)" == "0" ]]; then
        echo "  ✓ STP apagado en '$CAM_BRIDGE'"
      else
        echo "  ✖ STP sigue activo en '$CAM_BRIDGE'" >&2
      fi
    else
      echo "  ⚠ El bridge '$CAM_BRIDGE' tiene STP activo. Con tres equipos terminales no" >&2
      echo "    puede haber bucles, y el forward_delay hace que tras cada replug el" >&2
      echo "    puerto tarde ~30 s en reenviar (las camaras 'desaparecen' ese rato)." >&2
      echo "    Para apagarlo:  sudo FIX_BRIDGE_STP=1 $0" >&2
    fi
  fi
fi

echo "==> [3/9] Regla udev para nombre persistente del adaptador GPS ($GPS_DEV)"

# --- Detectar/validar el adaptador ANTES de escribir la regla --------------
# Todo lo que sigue (ldattach, gpsd, chrony) cuelga de que esta regla matchee,
# asi que un serial equivocado tiene que abortar aca y no seguir construyendo
# servicios que no van a poder arrancar.
declare -a CAND_DEV=() CAND_SER=() CAND_VID=() CAND_MODEL=()
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$dev" ]] || continue
  props=$(udevadm info -q property -n "$dev" 2>/dev/null)
  ser=$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"$props")
  vid=$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$props")
  mdl=$(sed -n 's/^ID_MODEL=//p' <<<"$props")
  [[ -n "$ser" ]] || continue
  CAND_DEV+=("$dev"); CAND_SER+=("$ser"); CAND_VID+=("$vid"); CAND_MODEL+=("${mdl:-?}")
done

# El PPS entra por la linea DCD del puerto serie: pps_ldisc no mira ninguna
# otra. Hay chips USB-serial muy comunes que directamente NO tienen DCD, y con
# esos el pulso no puede llegar por mas que el cable este bien puesto.
#
#   FT232R  (ID_MODEL_ID 6001)  control de modem completo -> DCD si
#   FT230X  (6015, "Basic UART") TXD/RXD/RTS/CTS + CBUS   -> DCD NO
#   FT231X  (6015, "USB UART")   control completo          -> DCD si (en el chip;
#                                                             el breakout puede
#                                                             no sacarlo)
#
# Costo real de no chequearlo: una jornada entera buscando un cable suelto y
# revisando gpsd, chrony y permisos, con el pulso imposible desde el principio
# porque el chip no tiene el pin.
tiene_dcd() {   # $1 = ID_MODEL
  case "$1" in
    *FT230X*|*Basic_UART*|*Basic\ UART*) return 1 ;;   # sin DCD, seguro
    *) return 0 ;;                                      # asumir que si
  esac
}

if (( ${#CAND_DEV[@]} == 0 )); then
  echo "  ✖ No hay ningun ttyUSB*/ttyACM* con numero de serie conectado." >&2
  echo "    Conecta el adaptador del GPS y volve a correr el script." >&2
  exit 1
fi

echo "  Adaptadores USB-serial detectados:"
for i in "${!CAND_DEV[@]}"; do
  dcd_nota=""
  tiene_dcd "${CAND_MODEL[$i]}" || dcd_nota="   <-- SIN linea DCD: no puede entregar PPS"
  printf "    %-14s serial=%-12s %-22s%s\n" \
    "${CAND_DEV[$i]}" "${CAND_SER[$i]}" "${CAND_MODEL[$i]}" "$dcd_nota"
done

if [[ -z "$GPS_SERIAL" ]]; then
  if (( ${#CAND_DEV[@]} == 1 )); then
    GPS_SERIAL="${CAND_SER[0]}"
    GPS_VENDOR="${CAND_VID[0]}"
    echo "  → Autodetectado: GPS_SERIAL=$GPS_SERIAL (${CAND_DEV[0]})"
  else
    echo "  ✖ Hay ${#CAND_DEV[@]} adaptadores y no se cual es el GPS." >&2
    echo "    Elegi uno y volve a correr:  GPS_SERIAL=<serial> $0" >&2
    exit 1
  fi
else
  GPS_VENDOR=""
  for i in "${!CAND_SER[@]}"; do
    if [[ "${CAND_SER[$i]}" == "$GPS_SERIAL" ]]; then
      GPS_VENDOR="${CAND_VID[$i]}"
    fi
  done
  if [[ -z "$GPS_VENDOR" ]]; then
    echo "  ✖ GPS_SERIAL=$GPS_SERIAL no coincide con ningun adaptador conectado." >&2
    echo "    Ese era el modo de falla silencioso: la regla udev no matchea," >&2
    echo "    $GPS_DEV no se crea, y gpsd-eodav nunca arranca." >&2
    echo "    Corregi GPS_SERIAL (o dejalo vacio para autodetectar)." >&2
    exit 1
  fi
  echo "  → GPS_SERIAL=$GPS_SERIAL verificado (idVendor=$GPS_VENDOR)"
fi

# --- el adaptador elegido, ¿puede entregar PPS? ----------------------------
GPS_MODEL=""
for i in "${!CAND_SER[@]}"; do
  [[ "${CAND_SER[$i]}" == "$GPS_SERIAL" ]] && GPS_MODEL="${CAND_MODEL[$i]}"
done
if ! tiene_dcd "$GPS_MODEL"; then
  echo >&2
  echo "  ✖ El adaptador elegido es un $GPS_MODEL: NO tiene linea DCD." >&2
  echo "    pps_ldisc lee el PPS unicamente por DCD, asi que con este chip el" >&2
  echo "    pulso no puede llegar aunque el cable TPS este bien puesto:" >&2
  echo "    /dev/ppsN se va a crear igual y ppstest va a dar 'Connection timed" >&2
  echo "    out' para siempre." >&2
  echo >&2
  echo "    Necesitas un adaptador con control de modem completo (FT232R," >&2
  echo "    FT231X con el pin sacado, o un puerto RS-232 real)." >&2
  echo >&2
  echo "    El NMEA y la posicion SI funcionan con este adaptador. Si queres" >&2
  echo "    seguir sin PPS -- reloj a ~10-100 ms en vez de ~1 us, suficiente" >&2
  echo "    para probar PTP entre sensores pero no para hora absoluta:" >&2
  echo "        sudo ALLOW_NO_PPS=1 GNSS_ONLY=1 $0" >&2
  echo >&2
  if [[ "${ALLOW_NO_PPS:-0}" != "1" ]]; then
    exit 1
  fi
  echo "  ⚠ ALLOW_NO_PPS=1: sigo sin PPS. El refclock PPS de chrony no va a" >&2
  echo "    recibir nada nunca; la referencia va a ser solo NMEA." >&2
fi

# --- segundo adaptador (solo modo dos adaptadores) -------------------------
GPS_NMEA_VENDOR=""
if [[ -n "$GPS_NMEA_SERIAL" ]]; then
  if [[ "$GPS_NMEA_SERIAL" == "$GPS_SERIAL" ]]; then
    echo "  ✖ GPS_NMEA_SERIAL y GPS_SERIAL son el mismo ($GPS_SERIAL)." >&2
    echo "    Si un solo adaptador lleva NMEA y PPS, dejá GPS_NMEA_SERIAL vacio." >&2
    exit 1
  fi
  for i in "${!CAND_SER[@]}"; do
    [[ "${CAND_SER[$i]}" == "$GPS_NMEA_SERIAL" ]] && GPS_NMEA_VENDOR="${CAND_VID[$i]}"
  done
  if [[ -z "$GPS_NMEA_VENDOR" ]]; then
    echo "  ✖ GPS_NMEA_SERIAL=$GPS_NMEA_SERIAL no coincide con ningun adaptador conectado." >&2
    exit 1
  fi
  echo "  → Modo DOS ADAPTADORES:"
  echo "      PPS  : $GPS_SERIAL ($GPS_MODEL) -> $GPS_DEV"
  echo "      NMEA : $GPS_NMEA_SERIAL -> $GPS_NMEA_DEV"
fi

cat > /etc/udev/rules.d/99-eodav-gps.rules <<EOF
# EOD-AV: nombre persistente para el adaptador USB-serial FTDI FT232R que lleva
# NMEA (TX->RX) y PPS (TPS->DCD) del simpleRTK3B Compass. ttyUSBn no es estable
# entre reconexiones (el orden de enumeración USB puede cambiar), lo que rompe
# gpsd-eodav.service / gps-pps-ldattach.service en silencio. Fijar por número
# de serie evita ese problema de raíz.
SUBSYSTEM=="tty", ATTRS{idVendor}=="$GPS_VENDOR", ATTRS{serial}=="$GPS_SERIAL", SYMLINK+="gps_pps"
# NO se puede poner una regla para el /dev/ppsN. Antes habia esta:
#     SUBSYSTEM=="pps", ATTRS{serial}=="$GPS_SERIAL", SYMLINK+="gps_pps_src"
# y NUNCA matchea. El motivo esta en el kernel, drivers/pps/clients/pps-ldisc.c:
#
#     info.dev = NULL;
#     pps = pps_register_source(&info, ...);
#
# El dispositivo PPS que crea la linea de disciplina se registra SIN padre, asi
# que no cuelga del arbol del USB y ATTRS{} (que sube por los padres) no tiene
# donde encontrar el numero de serie. Lo unico que lleva es
# /sys/class/pps/ppsN/name, que vale "usbserial<n>" -- atado al numero de
# ttyUSB, o sea tan inestable como lo que queriamos evitar.
#
# Consecuencia practica: chrony apunta al /dev/ppsN concreto, y si el USB se
# re-enumera ese numero cambia. El remedio es volver a correr este script: el
# paso [6/9] detecta el device nuevo y CORRIGE la linea de chrony.conf sola.
EOF
if [[ -n "$GPS_NMEA_SERIAL" ]]; then
  cat >> /etc/udev/rules.d/99-eodav-gps.rules <<EOF
# Adaptador separado que lleva solo NMEA (modo dos adaptadores).
SUBSYSTEM=="tty", ATTRS{idVendor}=="$GPS_NMEA_VENDOR", ATTRS{serial}=="$GPS_NMEA_SERIAL", SYMLINK+="gps_nmea"
EOF
fi
udevadm control --reload-rules
udevadm trigger
sleep 1
if [[ ! -e "$GPS_DEV" ]]; then
  # Abortar, NO avisar y seguir: gps-pps-ldattach.service hace ldattach sobre
  # $GPS_DEV y gpsd-eodav.service lo tiene como Requires=, asi que sin el
  # symlink los dos servicios quedan muertos y el unico sintoma aguas abajo es
  # "/fix no publica nada", que parece un problema de ROS y no lo es.
  echo "  ✖ $GPS_DEV no se creo pese a que el serial coincide." >&2
  echo "    Revisa la regla: udevadm test /sys/class/tty/\$(basename \$(readlink -f /dev/ttyUSB0))" >&2
  exit 1
fi
echo "  ✓ $GPS_DEV -> $(readlink -f "$GPS_DEV")"

echo "==> [4/9] Servicio persistente para activar pps-ldisc sobre $GPS_DEV"
modprobe pps_ldisc

# Type=simple + `ldattach -d`, NO Type=oneshot.
#
# Por defecto ldattach se demoniza, asi que con Type=oneshot/RemainAfterExit el
# proceso real queda huerfano: systemd lo da por "active (exited)" sin tenerlo.
# Consecuencias medidas en campo:
#   - `systemctl enable --now` NO hace nada si la unidad ya figura activa, asi
#     que al cambiar de adaptador la ldisc VIEJA sigue puesta sobre el tty
#     viejo. Sintoma exacto: /dev/gps_pps -> ttyUSB1 (FT232R, con DCD) pero
#     /dev/pps1 llamandose 'usbserial0' (= ttyUSB0, FT230X, SIN DCD), y ppstest
#     dando "Connection timed out" para siempre.
#   - Un `restart` dejaba dos ldattach corriendo sobre ttys distintos.
# `-d` mantiene ldattach en primer plano y systemd pasa a ser el duenio real
# del proceso, con lo que restart y stop funcionan de verdad.
cat > /etc/systemd/system/gps-pps-ldattach.service <<EOF
[Unit]
Description=Activa pps-ldisc (N_PPS) sobre $GPS_DEV — fuente del PPS del GNSS
Before=gpsd-eodav.service chrony.service

[Service]
Type=simple
ExecStart=/usr/sbin/ldattach -d 18 $GPS_DEV
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

# --- helpers de PPS ---------------------------------------------------------
# El nombre de la fuente PPS es 'usbserialN', donde N es el numero de ttyUSBN.
pps_dev_for_tty() {   # $1 = ttyUSBN  ->  imprime /dev/ppsM, o nada
  local tty="$1" p n
  for p in /sys/class/pps/pps*; do
    [[ -e "$p/name" ]] || continue
    n="$(cat "$p/name")"
    if [[ "$n" == "usbserial${tty#ttyUSB}" || "$n" == "$tty" ]]; then
      echo "/dev/$(basename "$p")"; return 0
    fi
  done
  return 0
}

# Deteccion de pulsos por SYSFS, no con `ppstest | grep`.
#
# El intento anterior era `timeout 8 ppstest "$dev" | grep -q assert` y daba un
# FALSO NEGATIVO sistematico. Causa (reproducida): con stdout hacia un PIPE,
# libc pasa a buffer de BLOQUE (4 KB); cada linea de ppstest son ~80 bytes, asi
# que hacen falta ~50 pulsos --50 segundos-- antes de que el buffer se vacie y
# grep vea algo. El timeout mataba el proceso mucho antes. En una terminal es
# line-buffered, y por eso a mano "funcionaba".
#
# /sys/class/pps/ppsN/assert trae "<sec>.<nsec>#<seq>" y se actualiza en cada
# flanco: exacto, instantaneo y sin buffering de por medio.
pps_pulsa() {   # $1 = /dev/ppsN
  local f="/sys/class/pps/$(basename "$1")/assert" a1 a2
  [[ -r "$f" ]] || return 1
  a1="$(cat "$f" 2>/dev/null || true)"
  sleep 3
  a2="$(cat "$f" 2>/dev/null || true)"
  [[ -n "$a2" && "$a1" != "$a2" ]]
}

GPS_TTY="$(basename "$(readlink -f "$GPS_DEV")")"          # p.ej. ttyUSB1
GPS_PPS_DEV=""
PPS_ALIVE=0

# NO reenganchar la ldisc si ya esta bien puesta Y pulsando.
#
# El script mataba y reiniciaba ldattach en CADA corrida, incluso con todo
# perfecto. Eso destruye un /dev/ppsN que funciona y crea uno nuevo con el
# contador en cero; y si el reenganche no prende, un PPS que andaba deja de
# andar POR CORRER EL SETUP. Es el sintoma que aparecio en campo: PPS OK, se
# corre el script, PPS muerto con assert en '0.000000000#0'.
YA="$(pps_dev_for_tty "$GPS_TTY")"
if [[ -n "$YA" ]] && pps_pulsa "$YA"; then
  GPS_PPS_DEV="$YA"; PPS_ALIVE=1
  echo "  ✓ la ldisc YA esta sobre $GPS_TTY y $GPS_PPS_DEV pulsa -- no la toco"
else
  # ORDEN: stop -> pkill huerfanos -> start.  NO al reves.
  #
  # La unidad tiene Restart=always / RestartSec=2. Un `pkill -x ldattach` mata
  # tambien el proceso GESTIONADO por systemd, que entonces programa un
  # auto-restart a los 2 s; si justo despues se hace `systemctl restart`, quedan
  # DOS ciclos de attach/detach sobre el mismo tty en un par de segundos.
  # Resultado medido en campo: /dev/ppsN existe pero su contador se queda en
  # '0.000000000#0' -- la ldisc quedo puesta sin armar la interrupcion de DCD.
  # Y no es que falte senal: el test de flancos de DCD daba 20 en 10 s.
  #
  # `systemctl stop` primero cancela la intencion de auto-restart, y recien ahi
  # el pkill toca UNICAMENTE huerfanos de corridas viejas.
  systemctl daemon-reload
  systemctl stop gps-pps-ldattach.service 2>/dev/null || true
  if pgrep -x ldattach >/dev/null 2>&1; then
    echo "  Matando ldattach huerfanos de corridas anteriores"
    pkill -x ldattach || true
  fi
  sleep 1
  systemctl enable gps-pps-ldattach.service >/dev/null 2>&1 || true
  systemctl start gps-pps-ldattach.service
  sleep 2

  GPS_PPS_DEV="$(pps_dev_for_tty "$GPS_TTY")"
  if [[ -z "$GPS_PPS_DEV" ]]; then
    echo "  ✖ No aparecio ninguna fuente PPS para $GPS_DEV ($GPS_TTY)." >&2
    echo "    Fuentes PPS presentes:" >&2
    for p in /sys/class/pps/pps*; do
      [[ -e "$p/name" ]] && echo "      /dev/$(basename "$p") -> $(cat "$p/name")" >&2
    done
    echo "    (una llamada 'ptp0' es el PHC de la NIC Intel, no el GNSS)" >&2
    echo "    Revisa:  systemctl status gps-pps-ldattach" >&2
    exit 1
  fi
  echo "  ✓ ldisc sobre $GPS_DEV ($GPS_TTY) -> $GPS_PPS_DEV"
  # Un reintento acotado: el primer attach tras cerrar el tty a veces no arma la
  # interrupcion de DCD. Un ciclo limpio de stop/start lo resuelve.
  for intento in 1 2; do
    if pps_pulsa "$GPS_PPS_DEV"; then PPS_ALIVE=1; break; fi
    if [[ $intento -eq 1 ]]; then
      echo "  (la ldisc no capturo flancos; reintento un ciclo limpio)"
      systemctl stop gps-pps-ldattach.service 2>/dev/null || true
      sleep 2
      systemctl start gps-pps-ldattach.service
      sleep 3
      GPS_PPS_DEV="$(pps_dev_for_tty "$GPS_TTY")"
      [[ -n "$GPS_PPS_DEV" ]] || break
    fi
  done
fi

if [[ "$PPS_ALIVE" == "1" ]]; then
  echo "  ✓ el PPS pulsa en $GPS_PPS_DEV"
else
  echo "  ⚠ NO llegan flancos por $GPS_PPS_DEV."
  echo "    (assert quedo en '$(cat "/sys/class/pps/$(basename "$GPS_PPS_DEV")/assert" 2>/dev/null)')"
  echo "    La ldisc esta sobre el adaptador correcto ($GPS_TTY, con DCD), asi"
  echo "    que la parte de software esta bien. Para separar cable de ldisc:"
  echo "        sudo ./diagnose_ptp.sh 1     # incluye un test de flancos de DCD"
  echo "    Si ese test tambien da 0, es fisico:"
  echo "      - el cable TPS/TIMEPULSE tiene que ir al pad DCD (DTR esta al lado)"
  echo "      - el TIMEPULSE del receptor activo (1 Hz, ancho 100 ms, alineado a UTC)"
  echo "      - GND compartido entre el receptor y el adaptador"
  echo "    Sigo igual y configuro chrony para andar SIN PPS (ver paso 6)."
fi

echo "==> [5/9] Configurando gpsd (NMEA + PPS en $GPS_DEV)"
# El gpsd.service/gpsd.socket empaquetado por Ubuntu arma su ExecStart a partir de
# /etc/default/gpsd vía sustitución de variables de systemd, y en la práctica esa
# sustitución no aplica bien DEVICES (queda "Referenced but unset environment
# variable... OPTIONS" en el log). Se reemplaza por un servicio propio con el
# comando explícito, igual que ptp4l-hw/ptp4l-sw/phc2sys-eodav más abajo.
systemctl stop gpsd.service gpsd.socket 2>/dev/null || true
systemctl disable --now gpsd.service gpsd.socket 2>/dev/null || true
pkill gpsd 2>/dev/null || true

# En modo DOS ADAPTADORES gpsd abre SOLO el de NMEA. Darle tambien el del PPS
# es contraproducente: no puede correlacionar el fix de un device USB con el
# pulso de otro, y el resultado es un SHM 1 con un corrimiento fijo pero espurio
# (~370 ms, medido). El PPS lo toma chrony directo, con `lock NMEA`.
if [[ -n "$GPS_NMEA_SERIAL" ]]; then
  GPSD_DEV="$GPS_NMEA_DEV"
  GPSD_DESC="NMEA en $GPS_NMEA_DEV (el PPS lo toma chrony directo de $GPS_PPS_SRC)"
else
  GPSD_DEV="$GPS_DEV"
  GPSD_DESC="NMEA + PPS en $GPS_DEV (un solo adaptador, gpsd correlaciona via sysfs)"
fi

cat > /etc/systemd/system/gpsd-eodav.service <<EOF
[Unit]
Description=gpsd EOD-AV — $GPSD_DESC
After=gps-pps-ldattach.service
Requires=gps-pps-ldattach.service

[Service]
ExecStart=/usr/sbin/gpsd -N -n $GPSD_DEV
Restart=always

[Install]
WantedBy=multi-user.target
EOF

echo "==> [6/9] Agregando refclocks GPS+PPS a chrony"
CHRONY_CONF="/etc/chrony/chrony.conf"
# Preferir el symlink udev (estable entre reconexiones); si udev no lo creo,
# usar el /dev/ppsN concreto que se verifico en el paso 4.
# Con PPS vivo el NMEA va con `noselect`: solo desambigua el segundo, es
# demasiado ruidoso para seleccionarlo. Sin PPS, `noselect` dejaria a chrony sin
# ninguna fuente y el reloj en deriva libre (se midio 508 ppm, contra los +-50
# de un cristal normal), asi que se lo deja seleccionable: ~100 ms es mucho peor
# que 1 us, pero es infinitamente mejor que nada, y todos los sensores quedan
# coherentes entre si igual.
if [[ "${PPS_ALIVE:-0}" == "1" ]]; then
  NMEA_SELECT="noselect"
else
  NMEA_SELECT=""
  echo "  ⚠ Sin PPS: el NMEA queda SELECCIONABLE (sin noselect) para que chrony"
  echo "    tenga al menos una fuente. Precision ~100 ms en vez de ~1 us."
fi

# El /dev/ppsN NO se puede fijar con udev cuando lo crea pps_ldisc: el kernel
# registra ese dispositivo sin padre (info.dev = NULL en pps-ldisc.c), asi que
# no hay ATTRS{serial} al que agarrarse. Ver el paso [3/9]. $GPS_PPS_SRC se
# mantiene solo por si alguna vez se usa un PPS de otra fuente (una placa con
# pin PPS propio, por ejemplo) que si tenga padre en el arbol de dispositivos.
if [[ -e "$GPS_PPS_SRC" ]]; then
  PPS_REFCLOCK_DEV="$GPS_PPS_SRC"
  echo "  ✓ hay un symlink estable ($GPS_PPS_SRC): lo uso para el refclock"
else
  PPS_REFCLOCK_DEV="$GPS_PPS_DEV"
  echo "  ℹ chrony apunta a $PPS_REFCLOCK_DEV (numero concreto, no symlink)."
  echo "    Es lo normal con pps_ldisc y no es un error. Lo unico a recordar:"
  echo "    si desenchufas y volves a enchufar el USB del GNSS, ese numero"
  echo "    puede cambiar -> volve a correr este script y se corrige solo."
fi
if [[ -f "$CHRONY_CONF" ]] && ! grep -q "agregado por setup_ptp_sync.sh" "$CHRONY_CONF"; then
  if [[ -n "$GPS_NMEA_SERIAL" ]]; then
    cat >> "$CHRONY_CONF" <<EOF

# --- agregado por setup_ptp_sync.sh (modo DOS ADAPTADORES) ---
# El NMEA viene por un adaptador ($GPS_NMEA_DEV, via gpsd -> SHM 0) y el PPS por
# otro ($GPS_PPS_SRC). gpsd NO puede correlacionar dos devices USB distintos, asi
# que el PPS NO pasa por gpsd: lo toma el driver PPS nativo de chrony.
#
# 'lock NMEA' es lo que los une: el pulso da la FRACCION de segundo (precisa) y
# el NMEA da el SEGUNDO ENTERO (grosero). Sin 'lock', chrony no sabe a que
# segundo pertenece cada pulso y el refclock PPS queda inutilizable.
#
# NMEA va con 'noselect' a proposito: llega por USB decenas de ms despues del
# segundo, demasiado ruidoso para seleccionarlo como fuente. Solo desambigua.
refclock SHM 0 refid NMEA offset $NMEA_OFFSET delay 0.2 $NMEA_SELECT
refclock PPS $PPS_REFCLOCK_DEV lock NMEA refid PPS precision 1e-7 prefer
EOF
  else
    cat >> "$CHRONY_CONF" <<EOF

# --- agregado por setup_ptp_sync.sh (modo UN ADAPTADOR) ---
# NMEA y PPS llegan por el mismo adaptador ($GPS_DEV), así gpsd correlaciona el
# fix (SHM 0) con su propio PPS (SHM 1) vía sysfs — no hace falta offset extra
# en la línea de PPS (a diferencia de una config con dos adaptadores separados).
refclock SHM 0 offset $NMEA_OFFSET delay 0.2 refid NMEA
refclock SHM 1 refid PPS precision 1e-7 prefer
EOF
  fi
elif [[ ! -f "$CHRONY_CONF" ]]; then
  echo "  ⚠ $CHRONY_CONF no existe. Si chrony esta instalado, buscalo con:"
  echo "      systemctl cat chrony | grep -i conf"
  echo "    y agregale a mano los dos refclock SHM (ver este script)."
else
  # Ya habia un bloque de este script. No se reescribe entero (puede tener
  # ajustes hechos a mano), pero SI se corrige el device del refclock PPS: es
  # justo el valor que cambia al mover la ldisc de adaptador, y dejarlo viejo
  # hace que chrony abra un /dev/ppsN que no es el del GNSS -- Reach 0 para
  # siempre, sin ningun error visible.
  ACTUAL=$(sed -n 's/^refclock PPS \([^ ]*\).*/\1/p' "$CHRONY_CONF" | head -1)
  if [[ -n "$ACTUAL" && "$ACTUAL" != "$PPS_REFCLOCK_DEV" ]]; then
    sed -i "s#^refclock PPS [^ ]*#refclock PPS $PPS_REFCLOCK_DEV#" "$CHRONY_CONF"
    echo "  ✓ refclock PPS corregido: $ACTUAL -> $PPS_REFCLOCK_DEV"
  fi
  # El offset del NMEA: si NMEA_OFFSET cambio, aplicarlo. Sin esto, correr con
  # `NMEA_OFFSET=0.38` sobre un chrony.conf que ya tenia el bloque no hacia
  # absolutamente nada -- el valor viejo quedaba puesto y el residuo no bajaba.
  ACTUAL_OFF=$(sed -n 's/^refclock SHM 0 .*offset \([^ ]*\).*/\1/p' "$CHRONY_CONF" | head -1)
  if [[ -n "$ACTUAL_OFF" && "$ACTUAL_OFF" != "$NMEA_OFFSET" ]]; then
    sed -i "/^refclock SHM 0 /s/offset [^ ]*/offset $NMEA_OFFSET/" "$CHRONY_CONF"
    echo "  ✓ offset del NMEA corregido: $ACTUAL_OFF -> $NMEA_OFFSET"
  fi
  # Y el noselect del NMEA, que depende de si el PPS pulsa o no (paso 4).
  if [[ "$NMEA_SELECT" == "noselect" ]]; then
    if ! grep -q '^refclock SHM 0 .*noselect' "$CHRONY_CONF"; then
      sed -i '/^refclock SHM 0 /s/$/ noselect/' "$CHRONY_CONF"
      echo "  ✓ NMEA -> noselect (hay PPS, el NMEA solo desambigua el segundo)"
    fi
  else
    if grep -q '^refclock SHM 0 .*noselect' "$CHRONY_CONF"; then
      sed -i '/^refclock SHM 0 /s/ *noselect//' "$CHRONY_CONF"
      echo "  ✓ NMEA -> seleccionable (no hay PPS; sin esto chrony se queda sin fuentes)"
    fi
  fi
fi

# --- Validar la config ANTES de reiniciar chrony ---------------------------
# Un chrony.conf invalido (p.ej. un refclock apuntando a un device inexistente)
# hace que el servicio no arranque y el reloj se quede sin disciplinar, que es
# peor que la situacion previa.
#
# La bandera es `-p` ("Print configuration and exit"), NO `-Q`.
#   -p   Print configuration and exit      <- parsea y sale al instante
#   -Q   Log offset and exit               <- ARRANCA y espera una medicion
#   -q   Set clock and exit                <- idem, ademas toca el reloj
# Con `-Q` el script se colgaba aca indefinidamente: chronyd se queda esperando
# que alguna fuente conteste, y con el NMEA en poll 16 s y los servidores de red
# inalcanzables eso no pasa nunca. El `timeout` es cinturon y tiradores.
if command -v chronyd >/dev/null 2>&1; then
  if timeout 15 chronyd -p -f "$CHRONY_CONF" >/dev/null 2>&1; then
    echo "  ✓ $CHRONY_CONF valida (sintaxis)"
  else
    echo "  ✖ $CHRONY_CONF NO valida. chronyd no va a arrancar. Salida:" >&2
    timeout 15 chronyd -p -f "$CHRONY_CONF" 2>&1 | sed 's/^/      /' >&2
    exit 1
  fi
fi

# `-p` valida SINTAXIS, no que los devices existan: un `refclock PPS /dev/ppsN`
# con un device inexistente pasa el parseo y despues hace fallar el arranque
# del servicio. Ese chequeo hay que hacerlo aparte.
CREF=$(sed -n 's/^refclock PPS \([^ ]*\).*/\1/p' "$CHRONY_CONF" | head -1)
if [[ -n "$CREF" && ! -e "$CREF" ]]; then
  echo "  ✖ chrony.conf tiene 'refclock PPS $CREF' pero ese device no existe." >&2
  echo "    chronyd no va a arrancar. Devices PPS presentes:" >&2
  for p in /sys/class/pps/pps*; do
    [[ -e "$p/name" ]] && echo "      /dev/$(basename "$p") -> $(cat "$p/name")" >&2
  done
  exit 1
fi

if [[ "$GNSS_ONLY" == "1" ]]; then
  echo
  echo "==> [7/9] (PTP sobre las interfaces de red): SALTEADO por GNSS_ONLY=1"
  echo "    Volve a correr sin GNSS_ONLY, con camaras y LiDAR conectados, para"
  echo "    completar la sincronizacion PTP."

  # Los enable --now viven en el paso [8/9], que es donde se arrancan TODOS los
  # servicios. La primera version de GNSS_ONLY salia antes de llegar ahi, asi
  # que escribia las unit files de gps-pps-ldattach y gpsd-eodav y no las
  # arrancaba nunca: el script terminaba diciendo "Cadena del GNSS lista" y
  # `systemctl is-active gpsd-eodav.service` devolvia "inactive". Aca se
  # arrancan los DOS servicios del GNSS (y solo esos).
  echo
  echo "==> [8/9] Arrancando los servicios del GNSS"
  systemctl daemon-reload
  systemctl enable --now gps-pps-ldattach.service
  systemctl enable --now gpsd-eodav.service
  systemctl restart chrony 2>/dev/null || true

  sleep 2
  echo
  if systemctl is-active --quiet gpsd-eodav.service; then
    echo "  ✓ gpsd-eodav.service activo"
  else
    echo "  ✖ gpsd-eodav.service NO arranco. Mira por que con:" >&2
    echo "      systemctl status gpsd-eodav.service --no-pager -l" >&2
    echo "      journalctl -u gpsd-eodav.service -n 40 --no-pager" >&2
    exit 1
  fi

  echo
  echo "==> Cadena del GNSS lista. Verifica con:"
  echo "    ls -l $GPS_DEV"
  echo "    systemctl status gpsd-eodav.service"
  echo "    gpspipe -w -n 5 localhost:2947"
  echo "    ros2 run gps_bringup diagnose_gnss.sh"
  exit 0
fi

echo "==> [7/9] Escribiendo /etc/linuxptp/ptp4l-hw.conf (Hesai) y ptp4l-sw.conf (3x Triton)"
mkdir -p /etc/linuxptp
# Huella de los conf ANTES de reescribirlos, para saber en el paso [8/9] si hay
# que REINICIAR el demonio o alcanza con arrancarlo. Ver el comentario alli.
sha_de() { [[ -f "$1" ]] && sha256sum "$1" | cut -d" " -f1 || echo "(no existia)"; }
SHA_HW_ANTES="$(sha_de /etc/linuxptp/ptp4l-hw.conf)"
SHA_SW_ANTES="$(sha_de /etc/linuxptp/ptp4l-sw.conf)"

{
  echo "[global]"
  echo "domainNumber        $PTP_DOMAIN"
  echo "priority1           128"
  echo "priority2           128"
  echo "delay_mechanism     E2E"
  echo "network_transport   UDPv4"
  echo "serverOnly          1"
  # Ver el bloque de configuracion: 248 (default) significa "reloj libre",
  # y el Hesai se quedaba en Free Run. 6 = trazable a referencia primaria.
  echo "clockClass          $PTP_CLOCK_CLASS"
  echo "clockAccuracy       $PTP_CLOCK_ACCURACY"
  echo "offsetScaledLogVariance $PTP_OFFSET_VARIANCE"
  # Tienen que COINCIDIR con lo que el Hesai tiene configurado en su web UI.
  echo "logSyncInterval     $PTP_LOG_SYNC_INTERVAL"
  echo "logAnnounceInterval $PTP_LOG_ANNOUNCE_INTERVAL"
  # Escala de tiempo: ptp4l SIEMPRE anuncia escala PTP (TAI) con
  # currentUtcOffset. En linuxptp 4.0 NO existe la opcion ptp_timescale
  # (verificado: "unknown option ptp_timescale"). Como CLOCK_REALTIME y el
  # PHC estan los dos en UTC (phc2sys -O 0), hay que anunciar offset 0: asi
  # un esclavo que calcula UTC = PTP - offset obtiene UTC, y uno que ignora
  # el offset tambien. Con el default de 37 el LiDAR quedaria 37 s corrido
  # respecto de las camaras. Ver setup/ARQUITECTURA_SINCRONIZACION.md 2.6.
  echo "utc_offset          0"
  echo "timeSource          0x20"   # GPS
  # uds_address va en [global]: si se agrega después de una sección de
  # interfaz (ej. [enp11s0]), ptp4l lo interpreta como opción de puerto y
  # falla en el arranque con "unknown option uds_address" (status 254).
  # Hace falta un socket propio por instancia porque corremos dos ptp4l en
  # simultáneo (hw y sw) y ambas compiten por el socket default /var/run/ptp4l.
  echo "uds_address         /var/run/ptp4l-hw"
  echo
  echo "[$LIDAR_IFACE]"
} > /etc/linuxptp/ptp4l-hw.conf

# --- Transporte de la instancia de camaras ---------------------------------
# MEDIDO (linuxptp 4.0, reproduccion con veth+bridge en un netns):
#
#   ptp4l en una interfaz suelta, UDPv4      -> path delay 1395..1552 ns  OK
#   ptp4l en un ESCLAVO de bridge, UDPv4     -> path delay 0             ROTO
#   ptp4l en el bridge (br0), UDPv4          -> "interface 'br0' does not
#                                               support requested timestamping
#                                               mode" y no arranca
#   ptp4l en un ESCLAVO de bridge, L2        -> path delay 1507..1696 ns  OK
#
# Por que: en un puerto de bridge el rx_handler (br_handle_frame) se queda con
# la trama ANTES de que llegue a los protocol handlers de IP, asi que un socket
# UDP con SO_BINDTODEVICE sobre el esclavo nunca ve el Delay_Req que manda la
# camara. El sentido master->esclavo si funciona (ptp4l transmite directo por
# el netdev, sin pasar por el bridge), y por eso el sintoma es enganioso: la
# camara ve Announce y Sync, dice "new foreign master", pasa a UNCALIBRATED y
# reporta offset -- pero con `path delay 0` para siempre, o sea sin compensar
# la latencia del enlace y sin cerrar nunca el intercambio E2E.
#
# Con transporte L2 (AF_PACKET) el problema desaparece: los taps de AF_PACKET
# corren en __netif_receive_skb_core ANTES del rx_handler del bridge.
#
# Se descarto STP y mcast_snooping como causa: el mismo test con
# `stp_state 0 mcast_snooping 0` sigue dando path delay 0.
#
# NO VERIFICADO EN HARDWARE: que las Triton acepten PTP sobre L2. Antes de dar
# esto por bueno hay que mirar que transporte habla la camara (ver el paso de
# verificacion al final del script).
# CORRECCION v20 (dato de campo): las Triton hablan PTP sobre UDPv4, no sobre
# L2 — se capturaron sus Delay_Req en `udp port 319` con clock identity
# 1c:0f:af:... (OUI de LUCID) y CERO paquetes en `ether proto 0x88f7`. O sea que
# poner L2 automaticamente por estar en un bridge, como hacia v17, las dejaba
# sin grandmaster. Con camaras GigE Vision, UDPv4 no es opcional.
#
# Consecuencia de quedarse en UDPv4 sobre un bridge: el path delay no se mide
# (queda en 0) y el offset del esclavo arrastra un sesgo igual al retardo de ida
# del enlace. MEDIDO en la reproduccion: 1400-1700 ns. Eso esta MUY por debajo
# del piso de ruido del timestamping por software (20-100 us), asi que en la
# practica no es el limitante. Lo que si hay que confirmar es que la camara
# igual llegue a PtpStatus == Slave: algunas implementaciones se niegan a
# declararse sincronizadas sin cerrar el intercambio E2E.
case "$CAM_PTP_TRANSPORT" in
  auto)
    SW_TRANSPORT="UDPv4"
    if [[ -n "$CAM_BRIDGE" ]]; then
      echo "  ⚠ Camaras en el bridge '$CAM_BRIDGE', transporte UDPv4." >&2
      echo "    En un puerto de bridge el rx_handler se queda con la trama antes de" >&2
      echo "    los protocol handlers de IP, asi que el Delay_Req de la camara NO" >&2
      echo "    llega a ptp4l y el path delay queda en 0 (sesgo de ~1,5 us, por" >&2
      echo "    debajo del ruido del timestamping por software)." >&2
      echo "    VERIFICA que las camaras lleguen igual a PtpStatus == Slave." >&2
      echo "    Si no llegan, hay que sacarlas del bridge: no hay arreglo por" >&2
      echo "    software (probado: ptp4l sobre el bridge no arranca con ningun" >&2
      echo "    modo de timestamping, y darle IP al esclavo tampoco cambia nada)." >&2
    fi
    ;;
  L2|UDPv4) SW_TRANSPORT="$CAM_PTP_TRANSPORT" ;;
  *) echo "CAM_PTP_TRANSPORT invalido: '$CAM_PTP_TRANSPORT' (auto|L2|UDPv4)" >&2; exit 1 ;;
esac

{
  echo "[global]"
  echo "domainNumber        $PTP_DOMAIN"
  echo "priority1           128"
  echo "priority2           128"
  echo "delay_mechanism     E2E"
  echo "network_transport   $SW_TRANSPORT"
  echo "serverOnly          1"
  echo "utc_offset          0"      # ver comentario en ptp4l-hw.conf
  echo "timeSource          0x20"   # GPS
  echo "time_stamping       software"
  echo "uds_address         /var/run/ptp4l-sw"
  echo
  for i in "${SW_IFACES[@]}"; do
    echo "[$i]"
  done
} > /etc/linuxptp/ptp4l-sw.conf

echo "==> [8/9] Creando servicios systemd: ptp4l-hw, ptp4l-sw, phc2sys (un solo puerto con PHC)"

# El paquete linuxptp de Ubuntu trae sus propios ptp4l.service/phc2sys.service
# y las plantillas ptp4l@.service/phc2sys@.service, listas para engancharse al
# instalar el paquete. Si quedan activos pelean por el mismo PHC/socket que
# nuestros servicios -eodav: se vio en pruebas como "clockcheck: clock
# frequency changed unexpectedly!" con el offset saltando erráticamente en
# phc2sys-eodav.service. Se usa `mask` (no solo `disable`) porque un `apt
# upgrade` de linuxptp puede re-habilitarlos solos; `disable` no sobrevive a eso.
echo "  Neutralizando servicios default de linuxptp (Ubuntu) para evitar conflicto de PHC/socket"
systemctl stop ptp4l.service phc2sys.service 2>/dev/null || true
systemctl mask ptp4l.service phc2sys.service 2>/dev/null || true
for i in "${ALL_IFACES[@]}"; do
  systemctl stop "ptp4l@$i.service" "phc2sys@$i.service" 2>/dev/null || true
  systemctl mask "ptp4l@$i.service" "phc2sys@$i.service" 2>/dev/null || true
done

cat > /etc/systemd/system/ptp4l-hw.service <<EOF
[Unit]
Description=ptp4l grandmaster EOD-AV — puerto con PHC real ($LIDAR_IFACE, Hesai)
After=network-online.target chronyd.service
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l-hw.conf -i $LIDAR_IFACE -m
Restart=always

[Install]
WantedBy=multi-user.target
EOF

SW_IFACE_ARGS=""
for i in "${SW_IFACES[@]}"; do
  SW_IFACE_ARGS="$SW_IFACE_ARGS -i $i"
done

cat > /etc/systemd/system/ptp4l-sw.service <<EOF
[Unit]
Description=ptp4l grandmaster EOD-AV — puertos sin PHC (3x Triton, software timestamping)
After=network-online.target chronyd.service
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l-sw.conf $SW_IFACE_ARGS -m
Restart=always

[Install]
WantedBy=multi-user.target
EOF

cat > /etc/systemd/system/phc2sys-eodav.service <<EOF
[Unit]
Description=phc2sys: copia CLOCK_REALTIME al PHC de $LIDAR_IFACE (único puerto con PHC real)
After=chronyd.service ptp4l-hw.service

[Service]
ExecStart=/usr/sbin/phc2sys -c $LIDAR_IFACE -s CLOCK_REALTIME -O 0 -m
Restart=always

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now gps-pps-ldattach.service
systemctl enable --now gpsd-eodav.service
systemctl restart chrony
# `systemctl enable --now` NO reinicia una unidad que ya esta activa: solo la
# arranca si estaba parada. Como el paso [7/9] acaba de REESCRIBIR los .conf,
# con `--now` el demonio seguia corriendo con la configuracion VIEJA, y el
# cambio no se veia hasta el proximo reboot.
#
# Sintoma exacto en campo: se agrego `clockClass 6` al conf, el script dijo que
# todo bien, y `pmc ... GET PARENT_DATA_SET` seguia devolviendo
# `gm.ClockClass 248` -- el valor del conf anterior. Es la misma trampa que ya
# habia mordido con gps-pps-ldattach (ver el comentario del paso [4/9]).
#
# Se reinicia SOLO lo que cambio: reiniciar ptp4l-sw sin motivo obliga a las
# tres camaras a reenganchar el PTP, y no hay razon para molestarlas si su
# configuracion es la misma.
reiniciar_si_cambio() {   # $1 = unidad, $2 = sha antes, $3 = archivo
  systemctl enable "$1" >/dev/null 2>&1 || true
  if [[ "$2" != "$(sha_de "$3")" ]]; then
    echo "  $3 cambio -> reiniciando $1"
    systemctl restart "$1"
  else
    systemctl start "$1" 2>/dev/null || true
  fi
}
reiniciar_si_cambio ptp4l-hw.service "$SHA_HW_ANTES" /etc/linuxptp/ptp4l-hw.conf
reiniciar_si_cambio ptp4l-sw.service "$SHA_SW_ANTES" /etc/linuxptp/ptp4l-sw.conf
# phc2sys no lee ningun .conf: su configuracion esta en el ExecStart de la
# unidad, que el paso [8/9] acaba de reescribir. daemon-reload + restart.
systemctl enable phc2sys-eodav.service >/dev/null 2>&1 || true
systemctl restart phc2sys-eodav.service

# --- Que el demonio CORRIENDO coincida con el archivo ----------------------
#
# El sha de arriba responde "¿cambio el .conf EN ESTA CORRIDA?", y esa es la
# pregunta equivocada. Si una corrida anterior escribio el .conf nuevo pero no
# reinicio el demonio, la siguiente ve el archivo igual, no reinicia, y el
# proceso queda con la configuracion vieja PARA SIEMPRE.
#
# Paso exactamente eso: v37 escribio `clockClass 6`, no reinicio; v38 comparo el
# sha, lo vio igual, tampoco reinicio, y `pmc` seguia devolviendo 248 con el
# archivo en disco diciendo 6.
#
# La pregunta correcta es "¿lo que ANUNCIA el demonio coincide con lo
# configurado?". Se le pregunta, y si no coincide se reinicia y se vuelve a
# verificar. Asi se auto-corrige sin importar el historial.
clockclass_anunciada() {
  pmc -u -b 0 -s /var/run/ptp4l-hw 'GET PARENT_DATA_SET' 2>/dev/null \
    | sed -n 's/.*gm\.ClockClass *//p' | tr -d ' \r'
}

sleep 3
CC_REAL="$(clockclass_anunciada)"
if [[ -n "$CC_REAL" && "$CC_REAL" != "$PTP_CLOCK_CLASS" ]]; then
  echo "  ptp4l-hw anuncia clockClass $CC_REAL pero el .conf dice $PTP_CLOCK_CLASS"
  echo "  -> reiniciando ptp4l-hw para que tome el archivo"
  systemctl restart ptp4l-hw.service
  sleep 4
  CC_REAL="$(clockclass_anunciada)"
fi
if [[ -z "$CC_REAL" ]]; then
  echo "  ⚠ no pude consultar ptp4l-hw para verificar clockClass" >&2
  echo "    revisa:  systemctl status ptp4l-hw" >&2
elif [[ "$CC_REAL" == "$PTP_CLOCK_CLASS" ]]; then
  echo "  ✓ ptp4l-hw esta anunciando clockClass $CC_REAL (lo configurado)"
else
  echo "  ✖ ptp4l-hw sigue anunciando clockClass $CC_REAL tras reiniciar." >&2
  echo "    El .conf dice $PTP_CLOCK_CLASS. Revisa que ptp4l este leyendo" >&2
  echo "    /etc/linuxptp/ptp4l-hw.conf:  systemctl cat ptp4l-hw | grep ExecStart" >&2
fi

echo ""
echo "==> [9/9] cap_net_raw al binario del radar (captura libpcap sin ser root)"
# POR QUE ESTE PASO VIVE ACA Y NO EN EL README
# El radar no tiene PTP: se captura con libpcap, y pcap_open_live() necesita
# CAP_NET_RAW. La capacidad se pega al INODO del binario, asi que **cada
# `colcon build` de ars430_ros_publisher la borra**: el archivo se reemplaza y
# el nuevo nace sin nada. El sintoma es silencioso y enganioso -- el nodo
# arranca, crea sus publishers y los topicos APARECEN en `ros2 topic list`,
# pero no sale ni un mensaje. Por eso se rehace en cada corrida del setup.
#
# El binario se ubica caminando hacia arriba desde este script: bajo sudo no
# hay AMENT_PREFIX_PATH ni `ros2` en el PATH (sudo borra PYTHONPATH), asi que
# `ros2 pkg prefix` NO sirve aca.
RADAR_REL="install/ars430_ros_publisher/lib/ars430_ros_publisher/radar_publisher"
RADAR_BIN=""
d="$SETUP_DIR"
# 8 saltos: con el layout colcon habitual (ws/src/EOD-AV-mod/setup) el
# workspace queda a 4, asi que sobra margen para arboles mas profundos.
for _ in 1 2 3 4 5 6 7 8; do
  if [[ -x "$d/$RADAR_REL" ]]; then RADAR_BIN="$d/$RADAR_REL"; break; fi
  d="$(dirname "$d")"
  [[ "$d" == "/" ]] && break
done
if [[ -z "$RADAR_BIN" ]]; then
  echo "  ⚠ no encontre $RADAR_REL subiendo desde $SETUP_DIR"
  echo "    Si todavia no compilaste el radar es normal. Despues de compilarlo:"
  echo "      sudo setcap 'cap_net_raw=pe' <workspace>/$RADAR_REL"
elif ! command -v setcap >/dev/null 2>&1; then
  echo "  ⚠ setcap no esta instalado:  apt install libcap2-bin"
else
  setcap 'cap_net_raw=pe' "$RADAR_BIN"
  if getcap "$RADAR_BIN" 2>/dev/null | grep -q cap_net_raw; then
    echo "  ✓ $(getcap "$RADAR_BIN")"
  else
    echo "  ✖ setcap no fallo pero getcap no ve la capacidad." >&2
    echo "    Suele ser el filesystem: sobre NFS, overlayfs o un FAT/exFAT no" >&2
    echo "    se pueden guardar capacidades. Fijate:  df -T $RADAR_BIN" >&2
  fi
fi

echo ""
echo "==> Listo. Verificá con:"
echo "   cat /sys/class/pps/pps*/name   # identificá cuál pps corresponde a $GPS_DEV (el nombre incluye 'usbserialN')"
echo "   sudo ppstest $GPS_PPS_DEV       # el que verifico el paso 4"
echo "   chronyc sources -v"
echo "   sudo pmc -u -b 0 -s /var/run/ptp4l-hw 'GET PARENT_DATA_SET'   # bus hw (Hesai)"
echo "   sudo pmc -u -b 0 -s /var/run/ptp4l-sw 'GET PARENT_DATA_SET'   # bus sw (3x Triton)"
echo "   # (el socket default /var/run/ptp4l ya no corresponde a ninguna instancia — ver paso 7)"
echo "   sudo phc_ctl $LIDAR_IFACE cmp   # el único puerto con PHC real que tiene sentido comparar"
if [[ -n "$GPS_NMEA_SERIAL" ]]; then
echo ""
echo "   # MODO DOS ADAPTADORES — verificá los dos symlinks:"
echo "   ls -l $GPS_NMEA_DEV $GPS_DEV $GPS_PPS_SRC"
echo "   # $GPS_PPS_SRC lo crea una regla udev sobre el subsistema 'pps'. Si NO"
echo "   # aparece, buscá el dispositivo real y corregí la ruta en chrony.conf:"
echo "   for p in /sys/class/pps/pps*; do echo \"/dev/\$(basename \$p) -> \$(cat \$p/name)\"; done"
echo "   # (el script ya dejo puesto: refclock PPS $PPS_REFCLOCK_DEV)"
echo "   sudo systemctl restart chrony && sleep 20 && chronyc sources -v"
fi
echo ""
echo "   # LO QUE HAY QUE MIRAR SI O SI (transporte de las camaras = $SW_TRANSPORT):"
echo "   #   'path delay' distinto de 0 = el intercambio E2E cierra."
echo "   #   'path delay' 0 constante   = la camara no habla este transporte,"
echo "   #                                o el bridge se esta comiendo el Delay_Req."
echo "   sudo pmc -u -b 0 -s /var/run/ptp4l-sw 'GET CURRENT_DATA_SET'"
echo "   sudo pmc -u -b 0 -s /var/run/ptp4l-sw 'GET PORT_DATA_SET'   # mirá peerMeanPathDelay"
echo ""
echo "   # Que transporte PTP habla realmente cada camara (con PtpEnable ya activo):"
echo "   sudo timeout 15 tcpdump -ni $CAM1_IFACE -e 'ether proto 0x88f7'          # PTP sobre L2"
echo "   sudo timeout 15 tcpdump -ni $CAM1_IFACE 'udp port 319 or udp port 320'   # PTP sobre UDPv4"
echo "   # Si solo aparece trafico en el segundo:  CAM_PTP_TRANSPORT=UDPv4 sudo -E ./setup_ptp_sync.sh"
echo "   # y entonces las camaras TIENEN que salir del bridge (ver paso 2)."
