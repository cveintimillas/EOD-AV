#!/usr/bin/env bash
#
# diagnose_gnss.sh -- recorre la cadena completa del GNSS y dice DONDE se corta.
#
#   ros2 run gps_bringup diagnose_gnss.sh
#
# La cadena tiene 6 eslabones y cada uno puede fallar en silencio:
#
#   1. adaptador USB           /dev/gps_pps existe y nadie mas lo tiene abierto
#   2. gpsd                    corriendo y con ese device
#   3. gpsd ve al receptor     llegan sentencias NMEA (independiente del lock)
#   4. gpsd reporta posicion   mode >= 2 (esto SI depende de ver cielo)
#   5. gpsd_client (ROS)       el componente cargo y publica /fix
#   6. fix_to_path (ROS)       publica la TF map -> base_link
#
# El sintoma "RViz dice map not found y /fix no muestra nada" es compatible con
# fallas en 1, 2, 3, 5 y 6, y ademas es el comportamiento NORMAL en 4 cuando se
# prueba bajo techo. Este script separa esos casos.
#
set -uo pipefail

GPS_DEV="${GPS_DEV:-/dev/gps_pps}"
GPSD_HOST="${GPSD_HOST:-localhost}"
GPSD_PORT="${GPSD_PORT:-2947}"

RED=$'\033[31m'; GREEN=$'\033[32m'; YEL=$'\033[33m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
fails=0

ok()   { echo "  ${GREEN}[ OK ]${OFF} $*"; }
warn() { echo "  ${YEL}[WARN]${OFF} $*"; }
bad()  { echo "  ${RED}[FALLA]${OFF} $*"; fails=$((fails+1)); }
step() { echo; echo "${BOLD}$*${OFF}"; }
hint() { echo "         -> $*"; }

have() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------------------
step "1/6  Adaptador USB-serial ($GPS_DEV)"
# ---------------------------------------------------------------------------
if [[ -e "$GPS_DEV" ]]; then
  ok "$GPS_DEV existe -> $(readlink -f "$GPS_DEV" 2>/dev/null || echo '?')"

  # El PPS entra SOLO por la linea DCD del puerto serie. Hay chips USB-serial
  # muy comunes que no la tienen, y con esos /dev/ppsN se crea igual pero
  # ppstest da "Connection timed out" para siempre: parece un cable suelto y es
  # el chip. Chequearlo aca ahorra buscar en el lugar equivocado.
  model=$(udevadm info -q property -n "$GPS_DEV" 2>/dev/null | sed -n 's/^ID_MODEL=//p')
  case "$model" in
    *FT230X*|*Basic_UART*)
      bad "el adaptador es un $model: NO tiene linea DCD -> el PPS es imposible"
      hint "El NMEA y la posicion funcionan igual; lo que no va a llegar nunca"
      hint "es el pulso, y sin pulso chrony queda con precision de ~10-100 ms."
      hint "Hace falta un adaptador con control de modem completo: FT232R,"
      hint "FT231X con el pin sacado, o un puerto RS-232 real."
      ;;
    "") warn "no pude leer ID_MODEL del adaptador" ;;
    *)  ok "adaptador: $model (deberia tener DCD)" ;;
  esac

  if have fuser; then
    holders=$(fuser "$GPS_DEV" 2>/dev/null | tr -s ' ')
    if [[ -n "${holders// /}" ]]; then
      procs=$(ps -o comm= -p ${holders} 2>/dev/null | sort -u | tr '\n' ' ')
      if [[ "$procs" == *gpsd* ]]; then
        ok "lo tiene abierto gpsd (correcto): $procs"
      else
        bad "lo tiene abierto un proceso que NO es gpsd: $procs"
        hint "gpsd tiene que ser el unico duenio del puerto. Cerra ese proceso."
      fi
    else
      warn "nadie tiene el puerto abierto (gpsd deberia tenerlo)"
    fi
  fi
else
  bad "$GPS_DEV NO existe"
  # Sin esto hay que ir a mano a comparar el serial del adaptador contra el de
  # la regla udev, que es justo el error mas comun al cambiar de adaptador.
  RULE=/etc/udev/rules.d/99-eodav-gps.rules
  expected=""
  if [[ -r "$RULE" ]]; then
    expected=$(sed -n 's/.*ATTRS{serial}=="\([^"]*\)".*/\1/p' "$RULE" | head -1)
    [[ -n "$expected" ]] && echo "         regla udev instalada: espera serial ${BOLD}$expected${OFF}"
  else
    hint "no hay regla udev en $RULE -> setup_ptp_sync.sh nunca corrio en esta maquina"
  fi

  found=0; match=0
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "$dev" ]] || continue
    ser=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')
    vid=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')
    [[ -n "$ser" ]] || continue
    found=$((found+1))
    if [[ -n "$expected" && "$ser" == "$expected" ]]; then
      match=1
      echo "         conectado: $dev serial=$ser idVendor=$vid  ${GREEN}(coincide)${OFF}"
    else
      echo "         conectado: $dev serial=$ser idVendor=$vid"
    fi
  done

  if (( found == 0 )); then
    hint "No hay NINGUN ttyUSB*/ttyACM* conectado: revisa el cable USB."
  elif [[ -n "$expected" ]] && (( match == 0 )); then
    echo "         ${RED}El serial esperado por udev no coincide con ningun adaptador conectado.${OFF}"
    hint "Esa es la causa: la regla no matchea, $GPS_DEV no se crea, y"
    hint "gps-pps-ldattach + gpsd-eodav no arrancan. Aguas abajo el unico"
    hint "sintoma es '/fix no publica nada', que parece un problema de ROS."
    hint "Arreglo:  cd setup && sudo GPS_SERIAL=<el serial de arriba> ./setup_ptp_sync.sh"
    hint "(o dejalo vacio y lo autodetecta si hay un solo adaptador)"
  else
    hint "Hay adaptador y la regla parece correcta: sudo udevadm control --reload-rules && sudo udevadm trigger"
  fi
fi

# ---------------------------------------------------------------------------
step "2/6  Servicio gpsd"
# ---------------------------------------------------------------------------
if systemctl is-active --quiet gpsd-eodav.service 2>/dev/null; then
  ok "gpsd-eodav.service activo"
elif pgrep -x gpsd >/dev/null 2>&1; then
  warn "hay un gpsd corriendo, pero NO es gpsd-eodav.service"
  hint "cmdline: $(tr '\0' ' ' < /proc/$(pgrep -x gpsd | head -1)/cmdline 2>/dev/null)"
  hint "El gpsd de Ubuntu no aplica bien DEVICES; usa el del setup:"
  hint "  sudo systemctl disable --now gpsd.service gpsd.socket"
  hint "  sudo systemctl restart gpsd-eodav.service"
elif systemctl is-enabled --quiet gpsd.socket 2>/dev/null || \
     systemctl is-active --quiet gpsd.socket 2>/dev/null; then
  # Caso enganioso: no hay proceso gpsd, pero el socket de systemd escucha en
  # 2947 y levanta uno SIN DEVICES en cuanto alguien se conecta. Por eso el
  # paso 3 puede decir "gpsd responde" justo despues de que este paso diga que
  # no hay gpsd: son dos cosas distintas.
  bad "gpsd-eodav.service NO corre; lo que escucha en 2947 es gpsd.socket de Ubuntu"
  hint "Ese gpsd se levanta por activacion de socket y NO tiene ningun device,"
  hint "asi que responde pero online==0 y /fix nunca publica."
  hint "sudo systemctl disable --now gpsd.socket gpsd.service"
  hint "sudo systemctl restart gpsd-eodav.service"
  hint "Si gpsd-eodav.service no existe, corre setup/setup_ptp_sync.sh."
else
  bad "no hay ningun gpsd corriendo"
  hint "sudo systemctl restart gpsd-eodav.service && systemctl status gpsd-eodav.service"
  hint "Si el servicio no existe, corre setup/setup_ptp_sync.sh."
fi

# ---------------------------------------------------------------------------
step "3/6  gpsd ve al receptor (llegan datos crudos)"
# ---------------------------------------------------------------------------
if have gpspipe; then
  raw=$(timeout 8 gpspipe -w -n 12 "$GPSD_HOST:$GPSD_PORT" 2>/dev/null)
  if [[ -z "$raw" ]]; then
    bad "gpsd no respondio en $GPSD_HOST:$GPSD_PORT"
    hint "Probá: gpspipe -w -n 5 $GPSD_HOST:$GPSD_PORT"
  else
    ok "gpsd responde en $GPSD_HOST:$GPSD_PORT"
    if grep -q '"class":"DEVICES"' <<<"$raw"; then
      devlist=$(grep -o '"path":"[^"]*"' <<<"$raw" | head -3 | tr '\n' ' ')
      if [[ -n "$devlist" ]]; then
        ok "gpsd tiene device(s): $devlist"
      else
        bad "gpsd corre pero NO tiene ningun device abierto"
        hint "Ese es el caso en que /fix queda mudo AUNQUE el nodo ROS este vivo:"
        hint "gpsd_client descarta todo mientras online==0 (isOnline() en el parser)."
        hint "sudo systemctl restart gpsd-eodav.service"
      fi
    fi
    if grep -q '"class":"TPV"' <<<"$raw"; then
      ok "llegan mensajes TPV (el receptor esta hablando)"
    else
      bad "no llega ningun TPV: gpsd no recibe NMEA del receptor"
      hint "Revisa baudrate/cableado TX->RX. Crudo: timeout 5 cat $GPS_DEV"
    fi
  fi
else
  warn "gpspipe no instalado, no puedo consultar gpsd directamente"
  hint "sudo apt install gpsd-clients"
fi

# ---------------------------------------------------------------------------
step "4/6  Lock del receptor (esto SI depende de ver cielo)"
# ---------------------------------------------------------------------------
if have gpspipe; then
  modes=$(timeout 8 gpspipe -w -n 20 "$GPSD_HOST:$GPSD_PORT" 2>/dev/null \
          | grep -o '"mode":[0-9]*' | cut -d: -f2 | sort -u | tr '\n' ' ')
  best=$(tr ' ' '\n' <<<"$modes" | sort -n | tail -1)
  case "${best:-0}" in
    3) ok "mode=3 (fix 3D). Hay posicion." ;;
    2) ok "mode=2 (fix 2D). Hay posicion." ;;
    *) warn "sin fix (mode=${best:-0}). NO es una falla del software."
       hint "Bajo techo esto es lo esperado. La antena necesita cielo despejado"
       hint "y el primer lock en frio puede tardar varios minutos."
       hint "Con check_fix_by_variance:=false /fix igual se publica, con"
       hint "status=-1 y lat/lon NaN: eso alcanza para validar toda la cadena ROS."
       hint "Satelites a la vista:  timeout 10 gpspipe -w $GPSD_HOST:$GPSD_PORT | grep -o '\"used\":true' | wc -l"
       ;;
  esac
fi

# ---------------------------------------------------------------------------
step "5/6  ROS: nodo gpsd_client y topico /fix"
# ---------------------------------------------------------------------------
if ! have ros2; then
  warn "ros2 no esta en el PATH; salteo la parte ROS (source install/setup.bash)"
else
  # --- 5a. el paquete gpsd_client, ¿esta instalado? ------------------------
  # Se chequea ANTES que los nodos porque es la causa mas basica y la que da un
  # error mas opaco: el contenedor arranca igual y solo escupe "Could not find
  # requested resource in ament index".
  if ros2 pkg prefix gpsd_client >/dev/null 2>&1; then
    ok "el paquete gpsd_client esta instalado ($(ros2 pkg prefix gpsd_client 2>/dev/null))"
    if timeout 15 ros2 component types 2>/dev/null \
         | grep -q 'gpsd_client::GPSDClientComponent'; then
      ok "el componente gpsd_client::GPSDClientComponent esta registrado"
    else
      bad "el paquete esta pero el componente NO esta registrado en el ament index"
      hint "Reinstala el paquete: sudo apt install --reinstall ros-${ROS_DISTRO:-jazzy}-gpsd-client"
    fi
  else
    bad "el paquete gpsd_client NO esta instalado"
    hint "Ese es el error 'Could not find requested resource in ament index' que"
    hint "tira el contenedor: no hay nada que cargar. NO es un problema de"
    hint "parametros ni de gpsd."
    hint "sudo apt install ros-${ROS_DISTRO:-jazzy}-gpsd-client"
    hint "o, desde el workspace (package.xml de gps_bringup ya lo declara):"
    hint "  rosdep install --from-paths src --ignore-src -r -y"
  fi

  # --- 5b. nodos -----------------------------------------------------------
  nodes=$(timeout 10 ros2 node list 2>/dev/null)
  # OJO: match EXACTO. '/gpsd_client' como subcadena tambien matchea
  # '/gpsd_client_container', y con eso el diagnostico daba OK justo en el caso
  # en que el componente NO habia cargado -- el error que se queria detectar.
  if grep -qx '/gpsd_client' <<<"$nodes"; then
    ok "el nodo /gpsd_client existe (el componente cargo)"
  elif grep -qx '/gpsd_client_container' <<<"$nodes"; then
    bad "el CONTENEDOR esta vivo pero el componente gpsd_client NO cargo"
    hint "Mira el error exacto en la salida del launch. Los dos casos tipicos:"
    hint " (a) 'Could not find requested resource in ament index' -> el paquete"
    hint "     gpsd_client no esta instalado (ver 5a)."
    hint " (b) NoParameterOverrideProvidedException -> falta alguno de los 7"
    hint "     parametros obligatorios (GPSDClientComponent los declara con la"
    hint "     sobrecarga solo-tipo de rclcpp, que exige override)."
    hint "Comprobacion: ros2 param list /gpsd_client"
  else
    bad "no hay ni contenedor ni nodo: el bring-up del GNSS no esta corriendo"
    hint "ros2 launch eod_av_launch gnss.launch.py"
  fi

  # --- 5c. /fix: lo que importa es que tenga PUBLISHERS --------------------
  # `ros2 topic list` muestra un topico que solo tiene suscriptores, y aca
  # siempre hay dos (fix_to_path y gnss_watchdog). O sea que "el topico existe"
  # no dice absolutamente nada sobre si el GNSS publica: hay que contar
  # publishers.
  pubs=$(timeout 10 ros2 topic info /fix 2>/dev/null \
         | sed -n 's/.*[Pp]ublisher count: *//p' | head -1)
  pubs=${pubs:-0}
  if (( pubs > 0 )); then
    ok "/fix tiene $pubs publicador(es)"
    echo "         esperando hasta 15 s un mensaje en /fix..."
    msg=$(timeout 15 ros2 topic echo /fix --once 2>/dev/null)
    if [[ -z "$msg" ]]; then
      bad "hay publicador pero NO llega ningun mensaje"
      hint "Significa online==0 en gpsd: el device no esta abierto (ver paso 3)."
      hint "gpsd_client hace return antes de publicar mientras eso pase."
    else
      ok "/fix publica"
      lat=$(grep -m1 'latitude:' <<<"$msg" | awk '{print $2}')
      st=$(grep -m1 'status:' <<<"$msg" | awk '{print $2}')
      echo "         latitude=$lat  status=$st"
      if [[ "$lat" == "nan" || "$lat" == ".nan" ]]; then
        warn "lat/lon en NaN -> el receptor todavia no tiene fix (ver paso 4)"
        hint "La cadena ROS esta BIEN. Falta senal, no software."
      else
        ok "hay posicion valida"
      fi
    fi
  else
    bad "/fix no tiene NINGUN publicador"
    hint "gpsd_client es el unico que deberia publicarlo (ver 5a y 5b)."
    hint "Si el topico aparece en 'ros2 topic list' es solo porque fix_to_path y"
    hint "gnss_watchdog estan suscritos: un topico con 0 publishers igual se lista."
  fi
fi

# ---------------------------------------------------------------------------
step "6/6  ROS: TF map -> base_link (lo que RViz reclama)"
# ---------------------------------------------------------------------------
if have ros2; then
  if grep -q '/fix_to_path' <<<"${nodes:-}"; then
    ok "el nodo /fix_to_path corre"
    if timeout 8 ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
      ok "la TF map -> base_link existe"
    else
      warn "no hay TF map -> base_link todavia"
      hint "fix_to_path solo la publica DESPUES del primer fix con lat/lon validas."
      hint "Sin lock no hay pose, y RViz muestra 'Fixed Frame [map] does not exist'."
      hint "Para trabajar sin GNSS con lock, pone Fixed Frame = base_link en RViz."
    fi
  else
    bad "fix_to_path no esta corriendo"
    hint "Es un nodo DERIVADO: solo corre en mode:=live, nunca en record."
    hint "ros2 launch eod_av_launch gnss.launch.py mode:=live"
  fi
fi

# ---------------------------------------------------------------------------
echo
if (( fails == 0 )); then
  echo "${GREEN}${BOLD}Sin fallas en la cadena.${OFF} Si igual no ves nada en RViz, lo mas"
  echo "probable es que falte lock del receptor (paso 4): sin fix no hay pose"
  echo "map -> base_link y no hay trayectoria que dibujar."
else
  echo "${RED}${BOLD}$fails problema(s) detectado(s).${OFF} Arranca por el primero de la lista:"
  echo "los eslabones de abajo dependen de los de arriba."
fi
exit 0
