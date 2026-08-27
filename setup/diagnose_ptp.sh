#!/usr/bin/env bash
#
# diagnose_ptp.sh — EOD-AV
#
# Recolecta TODO el estado de la cadena de sincronizacion en una sola pasada y
# lo imprime en un formato pensado para pegar tal cual.
#
#   sudo ./diagnose_ptp.sh              # niveles 0..3 (no necesita ROS corriendo)
#   sudo ./diagnose_ptp.sh 4            # ademas el nivel 4 (necesita los nodos ROS 2 vivos)
#   sudo ./diagnose_ptp.sh 1            # solo hasta el nivel 1
#   sudo ./diagnose_ptp.sh > /tmp/sync.txt 2>&1
#
# Niveles (cada uno depende del anterior; no tiene sentido mirar el 2 si el 1
# esta roto, porque un eslabon roto se disfraza de "sincronizacion mala" en el
# siguiente):
#
#   0  Inventario: version desplegada, interfaces, bridge, capacidades de la NIC
#   1  Referencia de tiempo: PPS -> chrony -> CLOCK_REALTIME
#   2  Distribucion: ptp4l x2, phc2sys, PHC
#   3  Trafico real en el cable (que transporte PTP habla cada dispositivo)
#   4  ROS 2: timestamps de los topicos
#
# NO usa `set -e` a proposito: un comando que falla es un DATO, no un motivo
# para abortar el diagnostico.

set -uo pipefail

MAXLEVEL="${1:-3}"

CAM1_IFACE="${CAM1_IFACE:-enp6s0}"
CAM2_IFACE="${CAM2_IFACE:-enp7s0}"
CAM3_IFACE="${CAM3_IFACE:-enp8s0}"
RADAR_IFACE="${RADAR_IFACE:-enp9s0}"
LIDAR_IFACE="${LIDAR_IFACE:-enp11s0}"

# ---------------------------------------------------------------- helpers ---
c_ok=$'\033[32m'; c_bad=$'\033[31m'; c_warn=$'\033[33m'; c_off=$'\033[0m'
[[ -t 1 ]] || { c_ok=""; c_bad=""; c_warn=""; c_off=""; }

seccion() { printf '\n\n========== %s ==========\n' "$*"; }
paso()    { printf '\n--- %s\n' "$*"; }
ok()      { printf '%s[ OK ]%s %s\n'    "$c_ok"   "$c_off" "$*"; }
bad()     { printf '%s[FALLA]%s %s\n'   "$c_bad"  "$c_off" "$*"; }
warn()    { printf '%s[ ? ]%s %s\n'     "$c_warn" "$c_off" "$*"; }
info()    { printf '       %s\n' "$*"; }

# Corre un comando mostrandolo, con timeout, sin abortar si falla.
run() {
  local t=$1; shift
  printf '$ %s\n' "$*"
  timeout "$t" "$@" 2>&1 | sed 's/^/  /'
  local rc=${PIPESTATUS[0]}
  [[ $rc -eq 124 ]] && printf '  (timeout de %ss)\n' "$t"
  return 0
}

tiene() { command -v "$1" >/dev/null 2>&1; }

if [[ $EUID -ne 0 ]]; then
  warn "No sos root. pmc, ppstest y tcpdump van a fallar. Corré con sudo."
fi

printf 'diagnose_ptp.sh — %s\n' "$(date -Is)"
printf 'host: %s   kernel: %s\n' "$(hostname)" "$(uname -r)"

# ============================================================ NIVEL 0 =======
seccion "NIVEL 0 — inventario"

paso "Version del workspace desplegada"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if grep -q CAM_PTP_TRANSPORT "$SCRIPT_DIR/setup_ptp_sync.sh" 2>/dev/null; then
  ok "setup_ptp_sync.sh es v17 o posterior (tiene CAM_PTP_TRANSPORT)"
else
  bad "setup_ptp_sync.sh es ANTERIOR a v17. Nada de lo que sigue aplica:"
  info "el transporte PTP de las camaras todavia es UDPv4 fijo."
fi
if grep -q 'utc_offset' "$SCRIPT_DIR/setup_ptp_sync.sh" 2>/dev/null; then
  ok "setup_ptp_sync.sh es v18 o posterior (fija utc_offset)"
else
  warn "setup_ptp_sync.sh anterior a v18: sin utc_offset explicito (riesgo de +-37 s)"
fi
info "ruta: $SCRIPT_DIR/setup_ptp_sync.sh"

paso "Interfaces"
run 10 ip -br link show

paso "Pertenencia a bridge y capacidades de timestamping"
for i in "$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE" "$RADAR_IFACE" "$LIDAR_IFACE"; do
  [[ -e "/sys/class/net/$i" ]] || { bad "$i no existe"; continue; }
  m=$(ip -o link show "$i" 2>/dev/null | sed -n 's/.* master \([^ ]*\).*/\1/p')
  phc=$(ethtool -T "$i" 2>/dev/null | sed -n 's/^PTP Hardware Clock: //p')
  drv=$(ethtool -i "$i" 2>/dev/null | sed -n 's/^driver: //p')
  printf '  %-9s driver=%-8s PHC=%-6s bridge=%s\n' \
    "$i" "${drv:-?}" "${phc:-?}" "${m:-(ninguno)}"
done
CAM_BRIDGE=$(ip -o link show "$CAM1_IFACE" 2>/dev/null | sed -n 's/.* master \([^ ]*\).*/\1/p')
if [[ -n "$CAM_BRIDGE" ]]; then
  warn "Las camaras estan en el bridge '$CAM_BRIDGE' -> el transporte PTP TIENE que ser L2"
  info "(medido: con UDPv4 sobre un esclavo de bridge el path delay queda en 0)"
  st=$(cat "/sys/class/net/$CAM_BRIDGE/bridge/stp_state" 2>/dev/null)
  [[ "$st" == "1" ]] && warn "STP activo en $CAM_BRIDGE: ~30 s de corte tras cada replug"
fi
# Ojo: preguntar por el bridge SOLO si la interfaz existe. Si no, un
# `ip link show` vacio se lee igual que "no esta en ningun bridge" y el
# diagnostico devuelve un OK sobre una interfaz inexistente.
if [[ ! -e "/sys/class/net/$LIDAR_IFACE" ]]; then
  bad "$LIDAR_IFACE (LiDAR) no existe — revisá el nombre con 'ip -br link show'"
elif [[ -n "$(ip -o link show "$LIDAR_IFACE" 2>/dev/null | sed -n 's/.* master .*/x/p')" ]]; then
  bad "$LIDAR_IFACE (LiDAR) esta en un bridge — sacalo"
else
  ok "$LIDAR_IFACE no esta en ningun bridge"
fi

paso "IPs"
run 10 ip -4 -br addr show

paso "Capacidad de captura del radar (cap_net_raw)"
# Se pierde en cada 'colcon build' que recompile el nodo, asi que es facil que
# se caiga sin que nadie lo note: el nodo arranca igual y no publica nada.
RADAR_REL="lib/ars430_ros_publisher/radar_publisher"
RADAR_BIN=""
d="$SCRIPT_DIR"
while [[ "$d" != "/" ]]; do
  [[ -x "$d/install/ars430_ros_publisher/$RADAR_REL" ]] && {
    RADAR_BIN="$d/install/ars430_ros_publisher/$RADAR_REL"; break; }
  d="$(dirname "$d")"
done
if [[ -z "$RADAR_BIN" ]]; then
  warn "no encontre el binario del radar (¿workspace sin compilar?)"
elif getcap "$RADAR_BIN" 2>/dev/null | grep -q cap_net_raw; then
  ok "$(getcap "$RADAR_BIN")"
else
  bad "el binario del radar NO tiene cap_net_raw: pcap_open_live va a fallar"
  info "y el nodo arranca igual, sin publicar una sola deteccion."
  info "  sudo setcap 'cap_net_raw=pe' $RADAR_BIN"
fi

[[ "$MAXLEVEL" -ge 1 ]] || exit 0

# ============================================================ NIVEL 1 =======
seccion "NIVEL 1 — referencia de tiempo (PPS -> chrony -> CLOCK_REALTIME)"

paso "Que es cada /dev/pps* (CRITICO: el PHC de la Intel tambien registra uno)"
# Puede haber MAS DE UNA fuente serie: cada ldattach de una corrida anterior
# deja la suya, y ademas el otro adaptador (el del NMEA) tambien puede tener
# una. Elegir "la ultima que aparezca" estaba mal: en campo tomo /dev/pps2
# (usbserial0 = el FT230X sin DCD) en vez de /dev/pps1 (usbserial1 = el FT232R
# bueno), y el diagnostico reporto DOS fallas que no existian.
#
# La correcta es la que corresponde al tty al que apunta /dev/gps_pps.
GPS_TTY_REF=""
[[ -e /dev/gps_pps ]] && GPS_TTY_REF="$(basename "$(readlink -f /dev/gps_pps)")"
GPS_PPS=""
GPS_PPS_FALLBACK=""
shopt -s nullglob
for p in /sys/class/pps/pps*; do
  n=$(cat "$p/name" 2>/dev/null)
  d="/dev/$(basename "$p")"
  case "$n" in
    ptp*) info "$d -> '$n'  = PHC de la NIC Intel, NO es el GNSS" ;;
    usbserial*|ttyS*)
      if [[ -n "$GPS_TTY_REF" &&
            ( "$n" == "usbserial${GPS_TTY_REF#ttyUSB}" || "$n" == "$GPS_TTY_REF" ) ]]; then
        info "$d -> '$n'  = adaptador serie, ESTE es el del GNSS (coincide con /dev/gps_pps)"
        GPS_PPS="$d"
      else
        info "$d -> '$n'  = adaptador serie, pero NO es a donde apunta /dev/gps_pps"
        GPS_PPS_FALLBACK="$d"
      fi ;;
    *) info "$d -> '$n'" ;;
  esac
done
shopt -u nullglob
# Sin /dev/gps_pps no hay con que decidir: se usa la unica serie que haya.
[[ -z "$GPS_PPS" ]] && GPS_PPS="$GPS_PPS_FALLBACK"
if [[ -z "$GPS_PPS" ]]; then
  bad "No hay ningun /dev/pps* que venga de un puerto serie."
  info "El pps_ldisc no esta enganchado, o el adaptador no tiene linea DCD."
  info "  sudo modprobe pps_ldisc && sudo ldattach 18 /dev/gps_pps"
else
  ok "PPS del GNSS = $GPS_PPS"
fi

paso "La ldisc, ¿esta sobre el adaptador CORRECTO?"
# El fallo silencioso mas caro de toda la cadena: /dev/gps_pps apunta al
# adaptador bueno, pero ldattach quedo enganchado a OTRO tty de una corrida
# anterior. ppstest da "Connection timed out" para siempre y no hay ni un
# error en ningun log. Se detecta comparando el destino del symlink con el
# nombre de la fuente PPS ('usbserialN' <-> ttyUSBN).
if [[ -e /dev/gps_pps ]]; then
  gtty=$(basename "$(readlink -f /dev/gps_pps)")
  info "/dev/gps_pps -> $gtty"
  if [[ -n "$GPS_PPS" ]]; then
    pname=$(cat "/sys/class/pps/$(basename "$GPS_PPS")/name" 2>/dev/null)
    if [[ "$pname" == "usbserial${gtty#ttyUSB}" || "$pname" == "$gtty" ]]; then
      ok "la ldisc esta sobre $gtty, que es a donde apunta /dev/gps_pps"
    else
      bad "DESAJUSTE: /dev/gps_pps -> $gtty pero la fuente PPS se llama '$pname'"
      info "'usbserialN' corresponde a ttyUSBN, asi que la ldisc quedo sobre OTRO"
      info "adaptador (tipico: sobrevivio de una corrida anterior). El PPS no puede"
      info "llegar nunca. Arreglo:"
      info "  sudo pkill -x ldattach"
      info "  sudo systemctl restart gps-pps-ldattach"
      info "  for p in /sys/class/pps/pps*; do echo \"/dev/\$(basename \$p) -> \$(cat \$p/name)\"; done"
    fi
  fi
else
  warn "/dev/gps_pps no existe: la regla udev no matcheo (ver paso 3 del setup)"
fi

paso "El refclock PPS de chrony, ¿apunta a ese mismo dispositivo?"
cref=$(sed -n 's/^refclock PPS \([^ ]*\).*/\1/p' /etc/chrony/chrony.conf 2>/dev/null | head -1)
if [[ -z "$cref" ]]; then
  info "(no hay 'refclock PPS' en chrony.conf; el PPS iria por gpsd/SHM 1)"
elif [[ -n "$GPS_PPS" && "$(readlink -f "$cref" 2>/dev/null || echo "$cref")" != "$GPS_PPS" ]]; then
  bad "chrony usa 'refclock PPS $cref' pero el PPS del GNSS es $GPS_PPS"
  info "  sudo sed -i 's#^refclock PPS [^ ]*#refclock PPS $GPS_PPS#' /etc/chrony/chrony.conf"
  info "  sudo systemctl restart chrony"
else
  ok "chrony apunta a $cref"
fi

paso "Pulsos reales del PPS del GNSS"
if [[ -n "$GPS_PPS" ]] && tiene ppstest; then
  info "Mirá la PARTE FRACCIONARIA del assert:"
  info "  cerca de .000000 / .999999 -> reloj disciplinado, correcto"
  info "  otro valor, derivando      -> el reloj todavia no convergio (ver chrony abajo)"
  info "  'timed out'                -> no llegan flancos: cable en DCD, o el chip no tiene DCD"
  run 6 ppstest "$GPS_PPS"
else
  warn "sin ppstest o sin dispositivo PPS del GNSS"
fi

paso "Flancos de DCD directo del chip (separa CABLE de disciplina de linea)"
# ppstest mide el PPS DESPUES de pps_ldisc. Si da timeout no distingue entre
# "no llega senal por el cable" y "la ldisc no esta enganchando los flancos".
# Este test lee el bit DCD del propio chip con TIOCMGET, sin pasar por la ldisc.
#
#   flancos ~= 2 por segundo  -> el cable y el TIMEPULSE estan BIEN; el problema
#                                (si ppstest falla) es la ldisc.
#   flancos == 0              -> no llega senal: cable en el pad equivocado
#                                (DTR esta al lado de DCD), TIMEPULSE apagado,
#                                o GND sin compartir. Ningun software lo arregla.
if [[ -e /dev/gps_pps ]] && command -v python3 >/dev/null 2>&1; then
  info "10 s midiendo /dev/gps_pps ..."
  python3 - <<'PYEOF' 2>&1 | sed 's/^/  /'
import fcntl, struct, termios, time
TIOCM_CD = 0x040
try:
    f = open('/dev/gps_pps')
except Exception as e:
    print('no se pudo abrir /dev/gps_pps:', e); raise SystemExit
prev = None; n = 0; t0 = time.time()
while time.time() - t0 < 10:
    try:
        cur = struct.unpack('I', fcntl.ioctl(f, termios.TIOCMGET, struct.pack('I', 0)))[0] & TIOCM_CD
    except Exception as e:
        print('TIOCMGET fallo:', e); break
    if prev is not None and cur != prev:
        n += 1
    prev = cur
    time.sleep(0.001)
print('flancos de DCD en 10 s: %d   (esperado ~20 con PPS de 1 Hz)' % n)
print('0 = el pulso NO llega al chip: es cable/TIMEPULSE/GND, no software.')
PYEOF
else
  warn "sin /dev/gps_pps: no se puede medir DCD"
fi

paso "gpsd"
run 10 systemctl is-active gpsd-eodav.service gpsd.service gpsd.socket
if tiene gpspipe; then
  run 10 gpspipe -w -n 4 localhost:2947
else
  warn "gpspipe no instalado (apt install gpsd-clients)"
fi

paso "NMEA — cadencia del receptor y saturacion del enlace serie"
# Por que esta comprobacion: el header.stamp de /fix es la EPOCA del receptor
# (use_gps_time:=true), asi que si el enlace serie no da abasto no se corrompe
# ningun timestamp -- pero se PIERDEN epocas enteras, y eso es perdida de dato
# que no se ve en ninguna tabla de sync_check.
#
# Se usa `gpspipe -n <cuenta>` y NO `timeout N gpspipe`: con timeout el pipe
# queda en modo bloque (4 KB) y si el corte llega antes de llenarlo no sale
# NADA. Es el mismo falso negativo que dio ppstest en su momento. Con -n el
# proceso termina solo y vacia sus buffers.
if ! tiene gpspipe; then
  warn "gpspipe no instalado (apt install gpsd-clients) — se saltea"
elif ! tiene awk; then
  warn "awk no disponible — se saltea"
else
  NMEA_CAP="$(mktemp)"
  gpspipe -r -n 600 localhost:2947 >"$NMEA_CAP" 2>/dev/null || true
  NMEA_BYTES=$(wc -c <"$NMEA_CAP")
  printf '$ gpspipe -r -n 600   (%s bytes)\n' "$NMEA_BYTES"

  # Inventario: que sentencias manda y cuantas de cada una.
  info "sentencias capturadas:"
  awk '/^\$/ {print substr($0,2,5)}' "$NMEA_CAP" \
    | sort | uniq -c | sort -rn | head -12 | sed 's/^/         /'

  # Cadencia real, a partir del campo UTC de GGA (hhmmss.ss).
  eval "$(awk -F, '
    /GGA/ && $2 ~ /^[0-9][0-9][0-9][0-9][0-9][0-9]\./ {
      t = substr($2,1,2)*3600 + substr($2,3,2)*60 + substr($2,5)
      if (t < prev) t += 86400          # cruce de medianoche
      if (prev != "") {
        d = t - prev
        if (d > 0) { n++; k = sprintf("%.2f", d); cnt[k]++ }
      }
      prev = t; if (first == "") first = t; last = t
    }
    END {
      if (n < 10) { print "NMEA_N=0"; exit }
      for (k in cnt) if (cnt[k] + 0 > best + 0) { best = cnt[k]; mode = k }
      printf "NMEA_N=%d NMEA_PERIODO=%s NMEA_HUECOS=%d NMEA_SPAN=%.2f\n",
             n + 1, mode, n - best, last - first
    }' "$NMEA_CAP")"

  if [[ "${NMEA_N:-0}" -lt 10 ]]; then
    warn "no se capturaron suficientes GGA para juzgar la cadencia"
  else
    NMEA_HZ=$(awk -v p="$NMEA_PERIODO" 'BEGIN{ printf "%.1f", (p>0)? 1/p : 0 }')
    info "epocas: $NMEA_N en ${NMEA_SPAN}s -> ${NMEA_HZ} Hz (periodo modal ${NMEA_PERIODO}s)"
    if [[ "${NMEA_HUECOS:-0}" -eq 0 ]]; then
      ok "sin epocas faltantes: el enlace serie da abasto"
    else
      NMEA_PCT=$(awk -v h="$NMEA_HUECOS" -v n="$NMEA_N" 'BEGIN{printf "%.1f", 100*h/n}')
      warn "$NMEA_HUECOS epocas faltantes de $NMEA_N (${NMEA_PCT}%)"
      info "No corrompe timestamps: el sello sigue siendo la epoca real. Lo que"
      info "se pierde son posiciones. Mira el ancho de banda de abajo ANTES de"
      info "culpar al enlace serie -- si esta holgado, la perdida no es por ahi."
    fi

    # Presupuesto de ancho de banda: 8N1 = 10 bits por caracter.
    BPS_CFG=$(gpspipe -w -n 8 localhost:2947 2>/dev/null \
              | grep -o '"bps":[0-9]*' | head -1 | cut -d: -f2)
    if [[ -n "$BPS_CFG" && "${NMEA_SPAN:-0}" != "0.00" ]]; then
      awk -v b="$NMEA_BYTES" -v s="$NMEA_SPAN" -v cfg="$BPS_CFG" 'BEGIN{
        need = (b/s)*10
        printf "       solo el NMEA reenviado: %.0f bit/s de %d (%.0f%%)\n",
               need, cfg, 100*need/cfg
      }'
      # ESTE es el numero que vale. El de arriba SUBESTIMA: `gpspipe -r` solo
      # muestra el NMEA que gpsd reenvia, y no cuenta el binario que gpsd
      # consume en silencio (RTCM3 del enlace RTK, UBX). Con `-R` (super raw)
      # salen TODOS los bytes tal como llegan del dispositivo.
      #
      # Por que importa: se midio una latencia de entrega de ~400 ms cuyos
      # saltos son multiplos enteros de 5,26 ms = 61 caracteres a 115200 = una
      # sentencia. O sea ~4600 bytes encolados. Una cola asi NO puede existir
      # sobre un enlace al 29%: se drenaria sola. Si el enlace real esta cerca
      # del 100%, todo cierra.
      if tiene python3; then
        info "carga REAL del cable (incluye binario que gpsd no reenvia):"
        python3 - "$BPS_CFG" <<'PY' 2>/dev/null | sed 's/^/       /' || info "  (no se pudo medir)"
import subprocess, sys, time
cfg = int(sys.argv[1])
p = subprocess.Popen(['gpspipe', '-R'], stdout=subprocess.PIPE)
t0 = time.time(); n = 0
try:
    while n < 60000 and time.time() - t0 < 20:
        b = p.stdout.read(4096)
        if not b: break
        n += len(b)
finally:
    p.kill()
dt = time.time() - t0
if n == 0 or dt <= 0:
    print("no llego nada por gpspipe -R"); sys.exit(1)
bps = n / dt * 10
print("%d bytes en %.2f s = %.0f bit/s de %d (%.0f%%)" % (n, dt, bps, cfg, 100*bps/cfg))
if bps > 0.8 * cfg:
    print(">80%: el cable esta al limite. ESO explica la cola y los huecos.")
    print("Subi el baud del receptor, o apaga las sentencias que no uses.")
else:
    print("Enlace holgado tambien contando el binario: la cola NO es del cable.")
    print("Si aun asi hay retardo de entrega, se genera DENTRO del receptor")
    print("(tiempo de solucion RTK/rumbo). No hay palanca desde el host, y no")
    print("afecta al dataset: el sello sigue siendo la epoca real.")
PY
      fi
    else
      info "no se pudo leer 'bps' de gpsd para el presupuesto de ancho de banda"
    fi
  fi
  rm -f "$NMEA_CAP"
fi

paso "chrony — la unica cosa que debe disciplinar CLOCK_REALTIME"
run 15 chronyc tracking
run 15 chronyc sources -v
info "Esperado: refid PPS seleccionado (marca '*'), NMEA con '#?' por el noselect."
info "'Not synchronised' aca invalida TODO lo de los niveles 2 y 4."

paso "Nadie mas tocando el reloj"
for u in systemd-timesyncd ntp ntpsec openntpd; do
  a=$(systemctl is-active "$u" 2>/dev/null)
  [[ "$a" == "active" ]] && bad "$u esta activo y pelea con chrony por CLOCK_REALTIME"
done
ok "revision de daemons NTP en conflicto terminada"

[[ "$MAXLEVEL" -ge 2 ]] || exit 0

# ============================================================ NIVEL 2 =======
seccion "NIVEL 2 — distribucion PTP"

paso "Servicios"
for u in ptp4l-hw ptp4l-sw phc2sys-eodav gps-pps-ldattach; do
  a=$(systemctl is-active "$u.service" 2>/dev/null)
  if [[ "$a" == "active" ]]; then ok "$u: $a"; else bad "$u: ${a:-no existe}"; fi
done

paso "Configuraciones generadas"
for f in /etc/linuxptp/ptp4l-hw.conf /etc/linuxptp/ptp4l-sw.conf; do
  printf '\n  # %s\n' "$f"
  sed 's/^/    /' "$f" 2>/dev/null || printf '    (no existe)\n'
done
T=$(sed -n 's/^network_transport *//p' /etc/linuxptp/ptp4l-sw.conf 2>/dev/null)
if [[ -n "$T" ]]; then
  ok "transporte de camaras: $T (bridge: ${CAM_BRIDGE:-no})"
  if [[ -n "$CAM_BRIDGE" ]]; then
    warn "en un bridge el Delay_Req de la camara no llega a ptp4l: el path delay"
    info "queda en 0 y el esclavo arrastra ~1,5 us de sesgo (por debajo del ruido"
    info "del timestamping por software). Lo decisivo es PtpStatus == Slave."
  fi
fi
if [[ ! -f /etc/linuxptp/ptp4l-hw.conf ]]; then
  bad "no existe /etc/linuxptp/ptp4l-hw.conf: setup_ptp_sync.sh nunca llego al paso 7"
else
  U=$(sed -n 's/^utc_offset *//p' /etc/linuxptp/ptp4l-hw.conf)
  if [[ "$U" == "0" ]]; then ok "utc_offset 0 (todo en UTC, coherente con phc2sys -O 0)"
  else warn "utc_offset='${U:-ausente -> ptp4l usa 37}' -> el LiDAR puede quedar 37 s corrido"; fi
fi

paso "Estado de cada instancia de ptp4l"
for inst in hw sw; do
  s="/var/run/ptp4l-$inst"
  printf '\n  ### instancia %s (%s)\n' "$inst" "$s"
  if [[ ! -S "$s" && ! -e "$s" ]]; then bad "el socket $s no existe (ptp4l-$inst no corre)"; continue; fi
  run 10 pmc -u -b 0 -s "$s" 'GET PORT_DATA_SET'
  run 10 pmc -u -b 0 -s "$s" 'GET CURRENT_DATA_SET'
  run 10 pmc -u -b 0 -s "$s" 'GET TIME_PROPERTIES_DATA_SET'
  run 10 pmc -u -b 0 -s "$s" 'GET PARENT_DATA_SET'
done
info ""
info "COMO LEERLO:"
info "  portState MASTER en todos los puertos = la PC es grandmaster (correcto)"
info "  currentUtcOffset debe ser 0 con este diseno (todo en UTC)"
info ""
info "  OJO con meanPathDelay / offsetFromMaster / peerMeanPathDelay: en un"
info "  GRANDMASTER valen 0 SIEMPRE, porque no tiene master del cual medirse"
info "  (stepsRemoved 0). NO son un criterio de nada desde este lado. El path"
info "  delay lo calcula el ESCLAVO; para verlo hay que preguntarle al equipo"
info "  (PtpStatus de la camara, web UI del Hesai) o mirar el cable (nivel 3)."

paso "PHC de la Intel vs CLOCK_REALTIME"
if tiene phc_ctl; then
  run 10 phc_ctl "$LIDAR_IFACE" cmp
  info "Esperado: offset en microsegundos. 37 s exactos = problema TAI/UTC."
fi
run 10 journalctl -u phc2sys-eodav -n 15 --no-pager
run 10 journalctl -u ptp4l-sw -n 15 --no-pager
run 10 journalctl -u ptp4l-hw -n 15 --no-pager

[[ "$MAXLEVEL" -ge 3 ]] || exit 0

# ============================================================ NIVEL 3 =======
seccion "NIVEL 3 — trafico PTP real en el cable"

if ! tiene tcpdump; then
  warn "tcpdump no instalado (apt install tcpdump) — este nivel se saltea"
else
  for i in "$CAM1_IFACE" "$LIDAR_IFACE"; do
    printf '\n  ### %s — PTP sobre L2 (ethertype 0x88f7)\n' "$i"
    run 14 tcpdump -ni "$i" -c 10 -e 'ether proto 0x88f7'
    printf '\n  ### %s — PTP sobre UDPv4 (puertos 319/320)\n' "$i"
    run 14 tcpdump -ni "$i" -c 10 'udp port 319 or udp port 320'
  done
  info ""
  info "COMO LEERLO: donde haya paquetes, ESE es el transporte que habla el"
  info "dispositivo. Las camaras y ptp4l-sw tienen que coincidir; el Hesai y"
  info "ptp4l-hw tambien. Si el Hesai no emite en ninguno de los dos, tiene el"
  info "PTP deshabilitado en su web UI."
  info ""
  info "ESTA ES LA PRUEBA QUE IMPORTA — el intercambio E2E:"
  info "  1. Buscá una linea 'Delay_Req' que venga del SENSOR (no de la PC)."
  info "  2. Mirá si la PC contesta en el PUERTO 320 dentro del milisegundo."
  info "     Ese paquete es el Delay_Resp."
  info "  Si contesta  -> el enlace PTP cierra de verdad."
  info "  Si no        -> el Delay_Req no llegó a ptp4l (tipico de puerto de"
  info "                  bridge: tcpdump lo ve porque AF_PACKET corre antes"
  info "                  del rx_handler, pero ptp4l no)."
fi

[[ "$MAXLEVEL" -ge 4 ]] || exit 0

# ============================================================ NIVEL 4 =======
seccion "NIVEL 4 — timestamps en ROS 2"

if ! tiene ros2; then
  bad "ros2 no esta en el PATH — hacé 'source install/setup.bash' antes"
  exit 0
fi

# El nivel 4 NO necesita root: `ros2 topic list/delay` corren como usuario.
# Y bajo sudo se rompen: sudo borra PYTHONPATH de la variable de entorno por
# seguridad (esta en su env_delete), asi que ni `sudo -E` lo conserva y ros2cli
# no se encuentra a si mismo:
#   importlib.metadata.PackageNotFoundError: No package metadata was found for ros2cli
if ! ros2 topic list >/dev/null 2>&1; then
  bad "ros2 no puede listar topicos."
  if [[ $EUID -eq 0 ]]; then
    info "Estas corriendo como root. sudo elimina PYTHONPATH (env_delete), asi que"
    info "ros2cli no arranca ni con 'sudo -E'. El nivel 4 no necesita root:"
    info ""
    info "    ./diagnose_ptp.sh 4        # SIN sudo, con el install/setup.bash sourceado"
    info ""
    info "(Los niveles 0-3 si necesitan root, corrélos aparte con sudo.)"
  else
    info "¿Esta corriendo el launch? ¿Sourceaste install/setup.bash?"
  fi
  exit 0
fi

paso "Topicos vivos"
run 20 ros2 topic list

paso "Retardo por topico (now - header.stamp)"
info "Esperado: todos en el mismo orden de magnitud (decenas de ms)."
info "  +-37 s exactos  -> TAI/UTC"
info "  negativo        -> el sensor esta adelantado: reloj sin disciplinar"
info "  horas           -> PTP nunca engancho, el equipo usa su contador interno"
info "  creciendo lineal-> el reloj del sensor corre libre"
# Los nombres de topico NO se hardcodean: cambian entre versiones del stack y
# entre drivers. (La lista fija anterior daba "no publicado" para las tres
# camaras cuando en realidad publicaban en /arena_camera_node/<iface>/image.)
# Se descubren de `ros2 topic list` por patron, uno por familia de sensor.
mapfile -t ALL_TOPICS < <(ros2 topic list 2>/dev/null)
declare -a PICK=()
pick_one() {   # $1 = regex, $2 = etiqueta
  local t
  for t in "${ALL_TOPICS[@]}"; do
    if [[ "$t" =~ $1 ]]; then PICK+=("$t"); return 0; fi
  done
  info "(ningun topico de $2)"
  return 0
}
pick_one 'points|pointcloud'            'LiDAR/nube'
pick_one 'imu'                          'IMU'
pick_one "$CAM1_IFACE.*(image|Image)"   "camara $CAM1_IFACE"
pick_one "$CAM2_IFACE.*(image|Image)"   "camara $CAM2_IFACE"
pick_one "$CAM3_IFACE.*(image|Image)"   "camara $CAM3_IFACE"
pick_one '^/fix$'                       'GNSS'
pick_one 'radar.*(packet|pointcloud)'   'radar'

if (( ${#PICK[@]} == 0 )); then
  bad "no se encontro ningun topico de sensor: ¿esta corriendo el launch?"
else
  info "Topicos elegidos: ${PICK[*]}"
  for t in "${PICK[@]}"; do
    printf '\n  ### %s\n' "$t"
    run 12 ros2 topic delay "$t"
  done
fi

paso "PTP de las camaras segun el log del nodo"
info "Buscá 'PtpStatus' en la salida del launch: tiene que decir Slave en las 3."
info "arena_camera_node lo lee UNA sola vez al arrancar y no espera al enganche,"
info "asi que si dice Listening/Uncalibrated los timestamps de esa sesion no sirven."

printf '\n\n== fin ==\n'
