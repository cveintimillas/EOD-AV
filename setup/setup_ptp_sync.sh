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
# Qué hace:
#   1. Instala linuxptp, chrony, gpsd y herramientas relacionadas.
#   2. Verifica hardware timestamping en cada interfaz.
#   3. Activa pps-ldisc sobre el puerto serial del GPS (como servicio persistente).
#   4. Configura gpsd para exponer NMEA + PPS a chrony.
#   5. Agrega los refclocks a chrony.
#   6. Genera ptp4l-hw.conf (Hesai, hardware) y ptp4l-sw.conf (3x Triton, software).
#   7. Crea servicios systemd: ptp4l-hw, ptp4l-sw, y phc2sys (solo para el puerto con PHC).
#
# ANTES DE CORRER: confirmá con `ip link show`, `ethtool -i <iface>` y `lspci -nn | grep -i eth`
# que los nombres de interfaz de abajo coinciden con tu PC real — pueden variar.
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
GPS_SERIAL_DEV="/dev/ttyUSB0"  # adaptador USB-serial dedicado a TPS (DCD) + NMEA (RX)
PTP_DOMAIN=0
# ============================================================

SW_IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE")
ALL_IFACES=("$CAM1_IFACE" "$CAM2_IFACE" "$CAM3_IFACE" "$LIDAR_IFACE")

echo "==> [1/7] Instalando paquetes"
apt update
apt install -y linuxptp chrony pps-tools setserial gpsd gpsd-clients

echo "==> [2/7] Verificando hardware timestamping en cada interfaz"
for i in "${ALL_IFACES[@]}"; do
  echo "--- $i ---"
  ethtool -T "$i" 2>/dev/null | grep -E "PTP Hardware Clock|HWTSTAMP" \
    || echo "  ⚠ No se pudo leer $i — confirmá el nombre de interfaz con 'ip link show'"
done
echo "  (Esperado: $LIDAR_IFACE con 'PTP Hardware Clock: 0'; las 3 Triton con 'none')"

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

echo "==> [6/7] Escribiendo /etc/linuxptp/ptp4l-hw.conf (Hesai) y ptp4l-sw.conf (3x Triton)"
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
  echo
  for i in "${SW_IFACES[@]}"; do
    echo "[$i]"
  done
} > /etc/linuxptp/ptp4l-sw.conf

echo "==> [7/7] Creando servicios systemd: ptp4l-hw, ptp4l-sw, phc2sys (un solo puerto con PHC)"

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
systemctl restart gpsd
systemctl restart chrony
systemctl enable --now ptp4l-hw.service
systemctl enable --now ptp4l-sw.service
systemctl enable --now phc2sys-eodav.service

echo ""
echo "==> Listo. Verificá con:"
echo "   sudo ppstest /dev/pps0"
echo "   chronyc sources -v"
echo "   sudo pmc -u -b 0 'GET PARENT_DATA_SET'"
echo "   sudo phc_ctl $LIDAR_IFACE cmp   # el único puerto con PHC real que tiene sentido comparar"