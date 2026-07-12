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
#   Un solo adaptador con las tres señales es la única forma correcta.
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
#
# ANTES DE CORRER: confirmá con `ip link show`, `ethtool -i <iface>` y `lspci -nn | grep -i eth`
# que los nombres de interfaz de abajo coinciden con tu PC real — pueden variar.
# Confirmá también el número de serie del adaptador USB-serial del GPS con:
#   udevadm info -q property -n /dev/ttyUSBx | grep ID_SERIAL_SHORT
#
# Uso: sudo bash setup_ptp_sync.sh

set -euo pipefail

# ============================================================
# CONFIGURACIÓN — EDITAR ESTOS VALORES ANTES DE EJECUTAR
# ============================================================
CAM1_IFACE="enp6s0"            # Triton 1 — Realtek RTL8125, sin PHC (software timestamping)
CAM2_IFACE="enp7s0"            # Triton 2 — Realtek RTL8125, sin PHC (software timestamping)
CAM3_IFACE="enp8s0"            # Triton 3 — Realtek RTL8125, sin PHC (software timestamping)
LIDAR_IFACE="enp11s0"          # Hesai — Intel I225-V onboard, CON PHC (hardware timestamping)
GPS_SERIAL="A5069RR4"          # ID_SERIAL_SHORT del adaptador FTDI FT232R (TPS->DCD, TX->RX, GND->GND)
GPS_DEV="/dev/gps_pps"         # symlink persistente (via udev, ver paso 3) -> ese adaptador
PTP_DOMAIN=0
# ============================================================

SW_IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE")
ALL_IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE" "$LIDAR_IFACE")

echo "==> [1/8] Instalando paquetes"
apt update
apt install -y linuxptp chrony pps-tools setserial gpsd gpsd-clients

echo "==> [2/8] Verificando hardware timestamping en cada interfaz"
for i in "${ALL_IFACES[@]}"; do
  echo "--- $i ---"
  ethtool -T "$i" 2>/dev/null | grep -E "PTP Hardware Clock|HWTSTAMP" \
    || echo "  ⚠ No se pudo leer $i — confirmá el nombre de interfaz con 'ip link show'"
done
echo "  (Esperado: $LIDAR_IFACE con 'PTP Hardware Clock: 0'; las 3 Triton con 'none')"

echo "==> [3/8] Regla udev para nombre persistente del adaptador GPS ($GPS_DEV)"
cat > /etc/udev/rules.d/99-eodav-gps.rules <<EOF
# EOD-AV: nombre persistente para el adaptador USB-serial FTDI FT232R que lleva
# NMEA (TX->RX) y PPS (TPS->DCD) del simpleRTK3B Compass. ttyUSBn no es estable
# entre reconexiones (el orden de enumeración USB puede cambiar), lo que rompe
# gpsd-eodav.service / gps-pps-ldattach.service en silencio. Fijar por número
# de serie evita ese problema de raíz.
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="$GPS_SERIAL", SYMLINK+="gps_pps"
EOF
udevadm control --reload-rules
udevadm trigger
sleep 1
if [[ ! -e "$GPS_DEV" ]]; then
  echo "  ⚠ $GPS_DEV no aparece todavía. Si el adaptador está conectado, revisá que" \
       "GPS_SERIAL coincida con 'udevadm info -q property -n /dev/ttyUSBx'."
fi

echo "==> [4/8] Servicio persistente para activar pps-ldisc sobre $GPS_DEV"
modprobe pps_ldisc
cat > /etc/systemd/system/gps-pps-ldattach.service <<EOF
[Unit]
Description=Activa pps-ldisc sobre $GPS_DEV (NMEA + PPS del simpleRTK3B Compass, mismo adaptador)
Before=gpsd-eodav.service chrony.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/ldattach 18 $GPS_DEV

[Install]
WantedBy=multi-user.target
EOF

echo "==> [5/8] Configurando gpsd (NMEA + PPS en $GPS_DEV)"
# El gpsd.service/gpsd.socket empaquetado por Ubuntu arma su ExecStart a partir de
# /etc/default/gpsd vía sustitución de variables de systemd, y en la práctica esa
# sustitución no aplica bien DEVICES (queda "Referenced but unset environment
# variable... OPTIONS" en el log). Se reemplaza por un servicio propio con el
# comando explícito, igual que ptp4l-hw/ptp4l-sw/phc2sys-eodav más abajo.
systemctl stop gpsd.service gpsd.socket 2>/dev/null || true
systemctl disable --now gpsd.service gpsd.socket 2>/dev/null || true
pkill gpsd 2>/dev/null || true

cat > /etc/systemd/system/gpsd-eodav.service <<EOF
[Unit]
Description=gpsd EOD-AV — NMEA + PPS en $GPS_DEV (un solo adaptador, gpsd correlaciona via sysfs)
After=gps-pps-ldattach.service
Requires=gps-pps-ldattach.service

[Service]
ExecStart=/usr/sbin/gpsd -N -n $GPS_DEV
Restart=always

[Install]
WantedBy=multi-user.target
EOF

echo "==> [6/8] Agregando refclocks GPS+PPS a chrony"
CHRONY_CONF="/etc/chrony/chrony.conf"
if [[ -f "$CHRONY_CONF" ]] && ! grep -q "agregado por setup_ptp_sync.sh" "$CHRONY_CONF"; then
  cat >> "$CHRONY_CONF" <<EOF

# --- agregado por setup_ptp_sync.sh ---
# NMEA y PPS llegan por el mismo adaptador ($GPS_DEV), así gpsd correlaciona el
# fix (SHM 0) con su propio PPS (SHM 1) vía sysfs — no hace falta offset extra
# en la línea de PPS (a diferencia de una config con dos adaptadores separados).
refclock SHM 0 offset 0.2 delay 0.2 refid NMEA
refclock SHM 1 refid PPS precision 1e-7 prefer
EOF
else
  echo "  (chrony.conf no encontrado en la ruta esperada, o ya tiene refclocks — revisar a mano)"
fi

echo "==> [7/8] Escribiendo /etc/linuxptp/ptp4l-hw.conf (Hesai) y ptp4l-sw.conf (3x Triton)"
mkdir -p /etc/linuxptp

{
  echo "[global]"
  echo "domainNumber        $PTP_DOMAIN"
  echo "priority1           128"
  echo "priority2           128"
  echo "delay_mechanism     E2E"
  echo "network_transport   UDPv4"
  echo "serverOnly          1"
  # uds_address va en [global]: si se agrega después de una sección de
  # interfaz (ej. [enp11s0]), ptp4l lo interpreta como opción de puerto y
  # falla en el arranque con "unknown option uds_address" (status 254).
  # Hace falta un socket propio por instancia porque corremos dos ptp4l en
  # simultáneo (hw y sw) y ambas compiten por el socket default /var/run/ptp4l.
  echo "uds_address         /var/run/ptp4l-hw"
  echo
  echo "[$LIDAR_IFACE]"
} > /etc/linuxptp/ptp4l-hw.conf

{
  echo "[global]"
  echo "domainNumber        $PTP_DOMAIN"
  echo "priority1           128"
  echo "priority2           128"
  echo "delay_mechanism     E2E"
  echo "network_transport   UDPv4"
  echo "serverOnly          1"
  echo "time_stamping       software"
  echo "uds_address         /var/run/ptp4l-sw"
  echo
  for i in "${SW_IFACES[@]}"; do
    echo "[$i]"
  done
} > /etc/linuxptp/ptp4l-sw.conf

echo "==> [8/8] Creando servicios systemd: ptp4l-hw, ptp4l-sw, phc2sys (un solo puerto con PHC)"

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
systemctl enable --now ptp4l-hw.service
systemctl enable --now ptp4l-sw.service
systemctl enable --now phc2sys-eodav.service

echo ""
echo "==> Listo. Verificá con:"
echo "   cat /sys/class/pps/pps*/name   # identificá cuál pps corresponde a $GPS_DEV (el nombre incluye 'usbserialN')"
echo "   sudo ppstest /dev/ppsN         # con el N correcto de arriba"
echo "   chronyc sources -v"
echo "   sudo pmc -u -b 0 -s /var/run/ptp4l-hw 'GET PARENT_DATA_SET'   # bus hw (Hesai)"
echo "   sudo pmc -u -b 0 -s /var/run/ptp4l-sw 'GET PARENT_DATA_SET'   # bus sw (3x Triton)"
echo "   # (el socket default /var/run/ptp4l ya no corresponde a ninguna instancia — ver paso 7)"
echo "   sudo phc_ctl $LIDAR_IFACE cmp   # el único puerto con PHC real que tiene sentido comparar"
