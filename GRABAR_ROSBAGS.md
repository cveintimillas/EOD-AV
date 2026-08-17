# Grabar y revisar datos — EOD-AV

Hay **DOS modos**, no tres:

| | Que hace | Comando |
|---|---|---|
| **1. RECORD** | Datos crudos, sin vista y sin nodos derivados. Unica excepcion: el radar publica ademas `/radar_pointcloud_99`. | `all_sensors.launch.py mode:=record` |
| **2. LIVE / PLAYBACK** | Procesa y visualiza. **Mismo pipeline**, cambia la fuente. | `all_sensors.launch.py` (sensores) o `processing.launch.py bag:=<bag>` (rosbag) |

Live y playback no son modos distintos: son `processing.launch.py` alimentado
por los sensores o por un bag. Por eso un bag reproducido se ve exactamente
igual que los sensores en vivo.

## TL;DR

```bash
# --- 1) GRABAR ----------------------------------------------------------
chronyc sources -v                                           # PTP con lock
ros2 launch eod_av_launch all_sensors.launch.py mode:=record # terminal 1
# terminal 2 -- OJO: sourcear el workspace o el bag sale sin radar
source ~/august/install/setup.bash
cd /media/<tu-ssd>/datasets && ros2 run eod_av_launch record_dataset.sh


# --- 2) VER UN BAG ------------------------------------------------------
ros2 launch eod_av_launch processing.launch.py bag:=<bag>    # una sola terminal
#   (equivalente, en dos terminales:)
#   ros2 bag play <bag> --clock
#   ros2 launch eod_av_launch processing.launch.py use_sim_time:=true

# --- 3) VER EN VIVO (mismo pipeline que 2, otra fuente) -----------------
ros2 launch eod_av_launch all_sensors.launch.py
```

Con `bag:=` el reloj se resuelve solo: `use_sim_time` viene en `auto`, o sea
que se pone en `true` cuando hay bag (y el `ros2 bag play` sale con `--clock`)
y en `false` con los sensores en vivo. Solo hay que tocarlo a mano si se
reproduce el bag desde otra terminal.

El sistema **nunca** inicia la grabación por su cuenta: la lanzás vos con
`ros2 bag record -a` desde la SSD. En modo record no se publica **nada**
derivado, así que `-a` produce un bag válido y sin datos redundantes (solo
agrega `/rosout` y `/parameter_events`, inocuos).

---

## Ver el bag en Lichtblick / Foxglove

El arbol de TF del sistema es el estandar: **una sola TF dinamica** del GNSS y
todo lo demas estatico colgando del vehiculo.

    map --(fix_to_path, dinamica)--> base_link --> hesai_lidar
                                              |--> radar_fixed
                                              |--> camera_enp6s0 / 7s0 / 8s0
                                              |--> gps_link

Cada sensor conserva SU frame en el `header.frame_id` de sus mensajes; nadie
publica en `map`. Lo que hace que todo se vea moverse por el mundo es elegir
`map` como frame de referencia EN EL VISOR: ahi el conjunto se desplaza como un
rigido y `/gps_path` va quedando dibujado detras.

Para que eso funcione el bag necesita las dos partes del arbol:

| | topico | como entra al bag |
|---|---|---|
| estaticas `base_link -> sensor` | `/tf_static` | siempre, en los dos modos |
| dinamica `map -> base_link` | `/tf` | **no en record**: se regenera en playback |

En modo record **`/tf` NO se graba**: `fix_to_path` es un nodo derivado y no
corre. Un bag de record se abre en Lichtblick con cada sensor en SU frame
(`hesai_lidar`, `camera_enp6s0`, `radar_fixed`), que es suficiente para mirar
los datos; lo que no hay es el mundo `map` bajo el vehiculo.

Para tener tambien la trayectoria y el frame `map`, se reproduce el bag con el
pipeline, que regenera la TF:

```bash
ros2 launch eod_av_launch processing.launch.py bag:=<bag>
```

Ahi RViz ya viene resuelto (`all_sensors.rviz` trae `Fixed Frame: map`). En
Lichtblick, panel 3D -> **Display frame = `map`**.

### El radar es la excepcion al "solo crudo"

Los demas sensores publican tipos ESTANDAR que Lichtblick dibuja solo desde el
bag: `sensor_msgs/Image`, `PointCloud2`, `NavSatFix`. El radar no:
`/unfiltered_radar_packet_1` es un tipo propio del paquete y queda como datos
opacos.

Por eso en record se agrega UN nodo que convierte ese mismo crudo en
**`/radar_pointcloud_99`** (nube SIN filtrar). Se graban los dos: el mensaje
propio es la fuente de verdad (lleva SNR, velocidad radial y campos que la nube
no), y la nube es lo unico dibujable sin conocer el paquete. No arrastra
filtrado, DBSCAN ni estadisticas.

Se apaga con `radar_cloud:=false`.

## Que se graba

| Topico | Contenido |
|---|---|
| `/arena_camera_node/enp{6,7,8}s0/image` | Bayer crudo, 3.15 MB/frame @12 fps |
| `/lidar_points` | nube del LiDAR (autocontenida) |
| `/lidar_ptp`, `/lidar_imu` | sincronizacion e IMU |
| `/unfiltered_radar_packet_1`, `/radar_objects_raw_1` | detecciones sin filtrar |
| `/fix`, `/extended_fix`, `/gnss/*` | GNSS |
| `/tf_static` | extrinsecas `base_link -> cada sensor` |

**Total: ~168 MB/s (605 GB/h)**, contra los ~400 MB/s (1.44 TB/h) del inicio.

## Que NO se graba (y se regenera solo)

| Omitido | Se reconstruye desde |
|---|---|
| `/radar_pointcloud_*`, `/filtered_radar_packet_*`, markers | `/unfiltered_radar_packet_1` |
| TF `map -> base_link` (pose) y `/gps_path` | `/fix` (via `fix_to_path`) |
| `.../image_color` | el Bayer |

**`fix_to_path` NO corre al grabar.** Todo lo que produce -- la TF
`map -> base_link` y `/gps_path` -- se deriva de `/fix`, asi que es
procesamiento y se reconstruye despues. El bag guarda `/fix` y, en
live/playback, el MISMO nodo `fix_to_path` regenera la pose y la trayectoria.
Es opcional en los dos ejes:

```bash
# sin trayectoria acumulada (solo la pose):
ros2 launch eod_av_launch processing.launch.py publish_path:=false
# sin fix_to_path en absoluto:
ros2 launch eod_av_launch processing.launch.py fix_to_path:=false
```

## Decisiones y por que

### Camaras en Bayer (no rgb8) — esto resolvio el lag

Medido en banco:

| | Antes (`rgb8`) | Ahora (`bayer_rggb8`) |
|---|---|---|
| Bytes/frame | 9.44 MB | **3.15 MB** |
| Uso del enlace GigE | **97 %** (922 de ~950 Mbps) | ~30 % |
| Rate | 9.5–11 Hz | **12.009 Hz** |
| Jitter (std dev) | 33 ms | **0.64 ms** |

`rgb8` hacia que el *firmware* interpolara el Bayer y transmitiera 3 B/px:
3x el ancho de banda por informacion que el sensor no captura. El enlace
saturado era la causa del lag — confirmado porque bajar la exposicion a 5 ms
**no** subio el rate.

No era el disco (escribia 360 MB/s con holgura) ni la CPU (68 % idle).

### LiDAR: se graba la nube, no los paquetes

Los paquetes UDP pesan 4x menos, pero reconstruir la nube exige el **archivo de
correccion angular** del sensor. Un dataset ilegible sin un archivo auxiliar
guardado aparte es fragil a anios vista, y los 41 MB/s de diferencia ya no
aprietan tras el ahorro de las camaras. Se prioriza que el bag sea
**autocontenido**.

### Las imagenes se ven en BLANCO Y NEGRO, y esta bien

RViz no demosaica Bayer: dibuja el mosaico como luminancia. Para **verificar**
la camara (encuadre, foco, exposicion) el B/N alcanza. Para ver color:

```bash
# recomendado: cero nodos extra, cv_bridge si demosaica
ros2 run rqt_image_view rqt_image_view /arena_camera_node/enp6s0/image

# alternativa: levanta un nodo por camara y apunta RViz a .../image_color
ros2 launch eod_av_launch all_sensors.launch.py debayer:=true
```

Si con `rqt_image_view` los colores salen cruzados, el patron nativo no es RGGB:
`pixelformat:=bayer_bggr8` (o `gbrg`/`grbg`).

---

## Recomendaciones de sistema

1. **Grabar headless.** `mode:=record` no abre RViz ni levanta nodos
   derivados; el unico extra es el visualizador de la nube del radar.
2. **Disco.** 168 MB/s: cualquier NVMe va sobrado. En `iostat` se vieron
   **rafagas** (`%util` 10 % con `aqu-sz` 17–26); el script usa
   `--max-cache-size 1GiB` para amortiguarlas.
3. **MCAP, no sqlite3.** Con `-a` hay que pedirlo a mano; da mucho mejor
   throughput para este caudal:
   ```bash
   ros2 bag record -a --storage mcap --max-cache-size 1073741824
   ```
4. **Usar `record_dataset.sh`, no `ros2 bag record -a`.** Con `-a`, si la
   terminal no tiene el workspace sourceado, los topicos de tipo propio
   (radar, `/lidar_ptp`) se SALTEAN en silencio y el bag sale incompleto sin
   que se note. El script aborta en ese caso, lista que esta publicando antes
   de arrancar, y deja fuera `/rosout`.
5. **Corte por TIEMPO, cada 4 min** (`--max-bag-duration 240`), no por tamanio.
   El caudal no es constante -- cambia con `processing:` y con cuantas camaras
   haya -- asi que un corte de 4 GiB cubria un intervalo desconocido. Ahora
   cada trozo cubre siempre el mismo intervalo de mundo. Se ajusta con
   `RECORD_SPLIT_S=<segundos>`.
6. **PTP primero**: sin el los timestamps no son comparables. Verificar con
   `chronyc sources -v` antes de grabar (el script tambien avisa).
7. Buffers de red: `sudo sysctl -w net.core.rmem_max=8388608 net.core.wmem_max=8388608`
8. Jumbo frames (MTU 9000) en las NIC de camara: complementario, ya no critico.

## Ajustes de camara (en caliente)

```bash
# Para el dataset conviene FIJO: radiometria estable entre las 3 camaras
ros2 param set /arena_cam_enp6s0 exposure_auto Off
ros2 param set /arena_cam_enp6s0 exposure_time 20000.0
ros2 param set /arena_cam_enp6s0 gain_auto Off
ros2 param set /arena_cam_enp6s0 gain 13.0

ros2 param set /arena_cam_enp6s0 frame_rate 12.0   # tope de fps
ros2 param set /arena_cam_enp6s0 width 1024        # resolucion (reinicia stream)
```

## Verificar la fidelidad de la grabacion

Lo unico que importa de verdad: **que no se pierdan frames**.

```bash
ros2 bag info <bag>     # 12 fps x 60 s = 720 mensajes por camara
ros2 topic hz /arena_camera_node/enp6s0/image
iostat -x 1
```

Si los counts cuadran, la grabacion es fiel.

---

# Procedimiento de prueba

## Prueba 1 — Adquisicion y grabacion

```bash
# Terminal 1: sensores en modo record (headless, solo crudo)
ros2 launch eod_av_launch all_sensors.launch.py mode:=record

# Terminal 2: comprobar que NO hay nada derivado publicandose
ros2 node list          # NO debe aparecer: fix_to_path, radar_processor,
                        # radar_visualizer, radar_clusters, debayer_*, rviz2
ros2 topic list         # NO debe aparecer: /gps_path, /radar_pointcloud_*,
                        # /filtered_radar_packet_*, /radar/cluster_markers,
                        # .../image_color

# Terminal 3: grabar DESDE LA SSD (unos segundos y Ctrl-C)
cd /media/<tu-ssd>/datasets
ros2 bag record -a

# Al terminar, verificar contenido y que no falten frames
ros2 bag info <bag>     # 12 fps x N s por camara
```

## Prueba 2 — Playback (sin sensores)

```bash
# Desconectar los sensores.

# Terminal 1: reproducir
ros2 bag play <bag>

# Terminal 2: el pipeline, sin ninguna fuente fisica
ros2 launch eod_av_launch processing.launch.py

# Terminal 3: comprobar que lo derivado se REGENERA
ros2 topic list         # ahora SI: /gps_path, /radar_pointcloud_1,
                        # /filtered_radar_packet_1, /radar/cluster_markers
ros2 node list          # fix_to_path, radar_processor, radar_visualizer,
                        # radar_clusters, rviz2
ros2 run tf2_ros tf2_echo map base_link    # la pose, regenerada desde /fix
```

En RViz: nube del LiDAR, nube y clusters del radar, trayectoria e imagenes.

## Prueba 3 — Live (con sensores)

```bash
# Reconectar los sensores.
ros2 launch eod_av_launch all_sensors.launch.py

# En otra terminal, la MISMA comprobacion que en la prueba 2:
ros2 node list
ros2 topic list
```

**Criterio de exito:** las listas de nodos y topicos de procesamiento de las
pruebas 2 y 3 deben ser IGUALES. Lo unico que cambia es quien publica los
topicos de entrada: los drivers (live) o `ros2 bag play` (playback).

Comparacion automatica:

```bash
# en la prueba 2
ros2 node list | sort > /tmp/nodos_playback.txt
# en la prueba 3
ros2 node list | sort > /tmp/nodos_live.txt
# los nodos de PROCESAMIENTO deben coincidir (los de driver solo en live)
diff /tmp/nodos_playback.txt /tmp/nodos_live.txt
```

## Color de las imagenes

En RViz salen en **blanco y negro** (no demosaica Bayer). Es esperado y sirve
para verificar encuadre/foco/exposicion. Para color:

```bash
ros2 run rqt_image_view rqt_image_view /arena_camera_node/enp6s0/image
# o, dentro de RViz:
ros2 launch eod_av_launch processing.launch.py debayer:=true
```
