#!/usr/bin/env bash
#
# setup_ptp_sync.sh — EOD-AV
# Aprovisiona sincronización PTP Nivel A para 3x Triton + 1x Hesai,
# referenciados a GPS vía simpleRTK3B Compass (PPS + NMEA).
#
# Qué hace:
#   1. Instala linuxptp, chrony, gpsd y herramientas relacionadas.
#   2. Verifica hardware timestamping en cada interfaz.
#   3. Activa pps-ldisc sobre el puerto serial del GPS (como servicio persistente).
#   4. Configura gpsd para exponer NMEA + PPS a chrony.
#   5. Agrega los refclocks a chrony.
#   6. Genera ptp4l.conf con una sección por interfaz (grandmaster, serverOnly).
#   7. Crea servicios systemd: ptp4l-eodav.service y phc2sys@<iface>.service (uno por puerto).
#
# ANTES DE CORRER: completá la sección CONFIGURACIÓN de abajo con los valores
# reales de tu PC. Los nombres de interfaz y el dispositivo del GPS hay que
# confirmarlos en sitio con `ip link show` — ver PTP_SYNC_CONTEXT.md sección 5.
#
# Uso: sudo bash setup_ptp_sync.sh

set -euo pipefail

# ============================================================
# CONFIGURACIÓN — EDITAR ESTOS VALORES ANTES DE EJECUTAR
# ============================================================
CAM1_IFACE="enp6s0"            # Triton 1
CAM2_IFACE="enp7s0"            # Triton 2
CAM3_IFACE="enp8s0"            # Triton 3
LIDAR_IFACE="enp11s0"     # TODO: confirmar interfaz Linux del puerto del Hesai
GPS_SERIAL_DEV="/dev/ttyUSB0"  # adaptador USB-serial dedicado a TPS (DCD) + NMEA (RX)
PTP_DOMAIN=0
# ============================================================

IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE" "$LIDAR_IFACE")

if [[ "$LIDAR_IFACE" == "CAMBIAR_ESTO" ]]; then
  echo "⚠ Editá LIDAR_IFACE en la sección CONFIGURACIÓN antes de correr este script." >&2
  exit 1
fi

echo "==> [1/7] Instalando paquetes"
apt update
apt install -y linuxptp chrony pps-tools setserial gpsd gpsd-clients

echo "==> [2/7] Verificando hardware timestamping en cada interfaz"
for i in "${IFACES[@]}"; do
  echo "--- $i ---"
  ethtool -T "$i" 2>/dev/null | grep -E "PTP Hardware Clock|HWTSTAMP" \
    || echo "  ⚠ No se pudo leer $i — confirmá el nombre de interfaz con 'ip link show'"
done

echo "==> [3/7] Servicio persistente para activar pps-ldisc sobre $GPS_SERIAL_DEV"
modprobe pps_ldisc
cat > /etc/systemd/system/gps-pps-ldattach.service <<EOF
[Unit]
Description=Activa pps-ldisc sobre $GPS_SERIAL_DEV (PPS del simpleRTK3B Compass)
Before=gpsd.service chrony.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/ldattach 18 $GPS_SERIAL_DEV

[Install]
WantedBy=multi-user.target
EOF

echo "==> [4/7] Configurando gpsd (NMEA + PPS)"
cat > /etc/default/gpsd <<EOF
START_DAEMON="true"
USBAUTO="false"
DEVICES="$GPS_SERIAL_DEV /dev/pps0"
GPSD_OPTIONS="-n"
EOF

echo "==> [5/7] Agregando refclocks GPS+PPS a chrony"
CHRONY_CONF="/etc/chrony/chrony.conf"
if [[ -f "$CHRONY_CONF" ]] && ! grep -q "refid PPS" "$CHRONY_CONF"; then
  cat >> "$CHRONY_CONF" <<EOF

# --- agregado por setup_ptp_sync.sh ---
refclock SHM 0 offset 0.2 delay 0.2 refid NMEA
refclock SHM 1 refid PPS precision 1e-7 prefer
EOF
else
  echo "  (chrony.conf no encontrado en la ruta esperada, o ya tiene refclocks — revisar a mano)"
fi

echo "==> [6/7] Escribiendo /etc/linuxptp/ptp4l.conf"
mkdir -p /etc/linuxptp
{
  echo "[global]"
  echo "domainNumber        $PTP_DOMAIN"
  echo "priority1           128"
  echo "priority2           128"
  echo "delay_mechanism     E2E"
  echo "network_transport   UDPv4"
  echo "serverOnly          1"
  echo
  for i in "${IFACES[@]}"; do
    echo "[$i]"
  done
} > /etc/linuxptp/ptp4l.conf

echo "==> [7/7] Creando servicios systemd: ptp4l-eodav y phc2sys@ (una instancia por puerto)"
IFACE_ARGS=""
for i in "${IFACES[@]}"; do
  IFACE_ARGS="$IFACE_ARGS -i $i"
done

cat > /etc/systemd/system/ptp4l-eodav.service <<EOF
[Unit]
Description=ptp4l grandmaster EOD-AV (4 puertos, serverOnly)
After=network-online.target chronyd.service
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf $IFACE_ARGS -m
Restart=always

[Install]
WantedBy=multi-user.target
EOF

cat > /etc/systemd/system/phc2sys@.service <<'EOF'
[Unit]
Description=phc2sys: copia CLOCK_REALTIME al PHC de %i
After=chronyd.service ptp4l-eodav.service

[Service]
ExecStart=/usr/sbin/phc2sys -c %i -s CLOCK_REALTIME -O 0 -m
Restart=always

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now gps-pps-ldattach.service
systemctl restart gpsd
systemctl restart chrony
systemctl enable --now ptp4l-eodav.service
for i in "${IFACES[@]}"; do
  systemctl enable --now "phc2sys@$i.service"
done

echo ""
echo "==> Listo. Verificá con:"
echo "   sudo ppstest /dev/pps0"
echo "   chronyc sources -v"
echo "   sudo pmc -u -b 0 'GET PARENT_DATA_SET'"
for i in "${IFACES[@]}"; do
  echo "   sudo phc_ctl $i cmp"
done
