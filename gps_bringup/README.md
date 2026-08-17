# gps_bringup

Bringup package that publishes GPS fixes to ROS 2 via `gpsd_client`, reading from the
system `gpsd` daemon instead of opening the GPS serial port directly.

`gpsd` is provisioned by `setup/setup_ptp_sync.sh` and already owns the simpleRTK3B's
serial port to feed chrony's GPS+PPS refclock. **NMEA and PPS share one single
adapter** (`/dev/gps_pps`, pinned by udev): `gpsd` can only correlate the fix with
the pulse when both come off the same device. The older two-adapter wiring
(NMEA on `ttyUSB0`, PPS on `ttyUSB1`) produced a fixed but spurious ~367 ms PPS
offset and was abandoned — see `setup/README.md`.

> **The adapter must have a DCD line.** `pps_ldisc` reads the pulse from DCD and
> nowhere else. An FT232R works; an **FT230X does not** (basic UART: TXD/RXD/RTS/
> CTS only). With the wrong chip, `/dev/ppsN` is still created and `ppstest` just
> times out forever — it looks like a loose wire and it is not.
If a ROS node also opened that same tty directly (as the old `nmea_serial_driver`
based launch file did), the two processes would race for the same serial device and
could corrupt each other's reads. `gpsd_client` avoids that by talking to `gpsd` over
its local TCP socket (port `2947`) instead.

Requirements
- ROS 2 installed and sourced
- `gpsd` running and configured (see `setup/setup_ptp_sync.sh` at the repo root)
- Runtime dependency `gpsd_client` (resolved via `rosdep`/apt: `ros-<distro>-gpsd-client`)

Quick setup
1. Update rosdep and install system deps for the workspace:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

2. Build the package and source the install overlay:

```bash
colcon build --packages-select gps_bringup
source install/setup.bash
```

Run
- Launch with defaults (connects to `gpsd` on `localhost:2947`):

```bash
ros2 launch gps_bringup gps.launch.py
```

- Point at a different gpsd host/port:

```bash
ros2 launch gps_bringup gps.launch.py gpsd_host:=localhost gpsd_port:=2947
```

Notes
- The launch file starts a `gpsd_client::GPSDClientComponent` (from the `gpsd_client`
  package) in a component container, and a `fix_to_path` node that subscribes to
  `/fix` and publishes `/gps_path` + the `map -> gps_link` TF.
- `use_gps_time` is enabled, so `NavSatFix` message stamps come from the GPS/PPS time
  gpsd reports (already PTP/GPS-disciplined by chrony), not from ROS node arrival time.
- Because `gpsd_client` only exposes parsed fixes (`sensor_msgs/NavSatFix` on `/fix`,
  `gps_msgs/GPSFix` on `/extended_fix`), raw NMEA sentences are no longer forwarded to
  ROS. If you need proprietary heading sentences (see below) that gpsd doesn't parse
  into `GPSFix`, you'll need a separate raw-NMEA tap (e.g. a small node reading
  gpsd's `?WATCH={"raw":1}` JSON stream) rather than a serial driver on the same tty.

Hardware & GNSS heading
- This setup uses a SimpleRTK3B receiver connected to Calibrated Survey Triple-band GNSS antennas (the antennas feed the RTK3B). The SimpleRTK3B outputs NMEA over serial; `gpsd` (not ROS) owns that serial port and does the NMEA parsing, and `gpsd_client` republishes gpsd's parsed fixes as ROS 2 topics.
- Important: multi-antenna heading (baseline/heading solution) may be provided by the RTK3B as specific NMEA sentences (or as vendor/proprietary sentences). Typical sentence identifiers that carry heading information include `PASHR`, `HDT`, `HDG`, or vendor-specific `$P...` sentences. Whether heading shows up depends on whether `gpsd` recognizes that sentence for this device — standard `HDT` is supported, proprietary `$P...`/`PASHR` sentences may not be. If the RTK3B is not configured to emit heading, or gpsd doesn't parse the specific sentence, you will not see heading on ROS topics.
- **Heading is now available**: see the sibling `um982_driver` package, which parses the UM982's proprietary `HPR`/`BESTNAVA`/`PVTSLNA` logs over a *second, dedicated* serial link (never `/dev/gps_pps` — that stays gpsd-only for the PTP pipeline) and publishes `/gnss/heading` and `/gnss/velocity`. Enable it via `gps.launch.py`'s `enable_um982_heading:=true` once that second link's udev rule and UM982 COM port are confirmed on the bench (see `um982_driver/README.md`).

How to verify the RTK3B output (serial checks)
- Do this only while `gpsd` (and thus the PTP setup) is stopped, since it already has the ttyUSB devices open — a second reader will race with it:

```bash
sudo systemctl stop gpsd
sudo usermod -a -G dialout $USER
newgrp dialout   # or log out and log in again

sudo stty -F /dev/ttyUSB0 115200 raw -echo -echoe -echok -crtscts
stdbuf -oL cat /dev/ttyUSB0 | sed -n '1,200p'

sudo systemctl start gpsd   # don't forget to bring it back up
```

- Look for NMEA sentences that contain heading information (`PASHR`, `HDT`, `HDG`, `VTG`, or vendor-specific `$P...`).

- To check what gpsd itself sees (works while gpsd is running, no conflict):

```bash
gpsmon           # or: cgps -s
```

How to verify in ROS 2
- After launching, list topics and inspect the fix:

```bash
ros2 topic list
ros2 topic echo /fix            # sensor_msgs/NavSatFix
ros2 topic echo /extended_fix   # gps_msgs/GPSFix (includes track/speed when available)
```

- If heading isn't in `/extended_fix.track`, it likely means gpsd isn't parsing the RTK3B's heading sentence — check with `gpsmon` first (see above) before assuming it's a ROS-side issue.

Expected topics (example output)
- `/fix` — `sensor_msgs/NavSatFix` (latitude/longitude/altitude), stamped with GPS/PPS time.
- `/extended_fix` — `gps_msgs/GPSFix` (adds track/speed/climb/DOP fields when gpsd reports them).
- `/gps_path`, TF `map -> gps_link` (or `path_child_frame`) — from `fix_to_path`.
- `/parameter_events` — ROS parameter change events (system-level).
- `/rosout` — ROS logging topic (system-level).

Troubleshooting
- No fixes in ROS: confirm `gpsd` itself is locked (`gpsmon`, `chronyc sources -v`) before looking at the ROS side — `gpsd_client` is just a passthrough.
- No heading: verify RTK3B configuration — some receivers require enabling specific message output for heading/baseline solutions — and confirm gpsd actually parses that sentence (see above).
- Can't connect to gpsd: confirm it's running (`systemctl status gpsd`) and listening on the port passed via `gpsd_port` (default `2947`).

License
- MIT

---

## Troubleshooting: `/fix` no publica nada y RViz dice que falta `map`

Los dos síntomas son **el mismo problema visto en dos lugares**: sin `/fix` no
hay pose, y `fix_to_path` sólo publica la TF `map -> base_link` después del
primer fix válido, así que el frame `map` nunca llega a existir.

Lo primero, siempre:

```bash
ros2 run gps_bringup diagnose_gnss.sh
```

Recorre los 6 eslabones de la cadena y dice en cuál se corta. Abajo, qué
significa cada corte.

### 0. El paquete `gpsd_client` no está instalado

```
[component_container-1] [ERROR] [gpsd_client_container]: Could not find requested resource in ament index
[ERROR] [launch_ros.actions.load_composable_nodes]: Failed to load node 'gpsd_client' ...
```

Ese mensaje **no** habla de parámetros ni de gpsd: dice que no hay ningún plugin
registrado con ese nombre, o sea que el paquete no está instalado. `gpsd_client`
viene aparte (no está vendorizado en este workspace):

```bash
sudo apt install ros-$ROS_DISTRO-gpsd-client
# o, desde el workspace (package.xml de gps_bringup ya lo declara como exec_depend):
rosdep install --from-paths src --ignore-src -r -y

ros2 component types | grep GPSDClientComponent   # verificación
```

`gps.launch.py` ahora chequea esto al arrancar y aborta con ese mensaje en vez
de dejar el error opaco del contenedor.

### 1. El componente `gpsd_client` no cargó

```bash
ros2 node list | grep gpsd
#   /gpsd_client_container   <- está
#   /gpsd_client             <- NO está  => el componente no cargó
```

`GPSDClientComponent` declara sus **7** parámetros con la sobrecarga *sólo-tipo*
de rclcpp:

```cpp
this->declare_parameter("override_augmentation_source", rclcpp::PARAMETER_BOOL);
```

Esa forma declara el parámetro **sin valor por defecto**, o sea que exige un
override. Si falta aunque sea uno, `declare_parameter` lanza
`NoParameterOverrideProvidedException`, el constructor del componente revienta
y el `component_container` **no lo carga pero sigue corriendo**. Resultado: el
launch arranca sin errores visibles y `/fix` no existe.

Los 7 obligatorios son `host`, `port`, `frame_id`, `publish_rate`,
`use_gps_time`, `check_fix_by_variance` y `override_augmentation_source`.
`gps.launch.py` los pasa todos.

```bash
ros2 param list /gpsd_client    # deben aparecer los 7
```

### 2. gpsd no tiene el device abierto

Es el caso más engañoso: el nodo ROS está vivo, `/fix` existe como tópico, y no
sale ni un mensaje. `gpsd_client::step()` hace:

```cpp
if (p == nullptr || !parser_->isOnline(*p))
  return;
```

y `isOnline()` es `data.online.tv_sec > 0 || data.online.tv_nsec > 0`. Mientras
gpsd no tenga un device activo, `online` es 0 y el nodo descarta todo **antes**
de publicar.

```bash
gpspipe -w -n 5 localhost:2947        # ¿responde? ¿aparece "class":"DEVICES"?
systemctl status gpsd-eodav.service
sudo systemctl restart gpsd-eodav.service
```

### 3. El receptor no tiene lock (esto **no** es una falla)

Bajo techo es lo normal. Con `check_fix_by_variance:=false` — que es como está
configurado — `/fix` **se publica igual**, con `status=-1` y `lat/lon` en `NaN`:

```bash
ros2 topic hz /fix                    # ~10 Hz
ros2 topic echo /fix --once           # latitude: nan
```

Eso ya demuestra que toda la cadena ROS funciona. Lo que falta es señal: la
antena necesita cielo despejado y el primer lock en frío puede tardar varios
minutos. Hasta entonces no hay TF `map -> base_link` y RViz sigue diciendo que
el frame `map` no existe — es correcto, no hay pose que dibujar.

El nodo `gnss_watchdog` (levantado por defecto, no publica ningún tópico)
distingue los tres estados por consola: **MUDO** (software), **SIN FIX** (falta
señal) y **CON FIX**.

### Por qué `gnss.rviz` usa `Fixed Frame: base_link`

Con `map` como Fixed Frame, sin lock RViz no dibuja **nada** y el único síntoma
es `Fixed Frame [map] does not exist`, que se lee como si el software estuviera
roto. Con `base_link` la ventana funciona siempre y `/gps_path` aparece cuando
hay posición. Para seguir la trayectoria en un marco fijo, cambiá a `map` desde
el desplegable una vez que haya fix.
