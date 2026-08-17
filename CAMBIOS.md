# EOD-AV — copia modificada (para instalar manualmente)

Copia independiente del workspace EOD-AV con la reestructuración de launches
solicitada y los arreglos de la revisión previa. **El repositorio original no se
tocó**; esta carpeta es una copia para descargar, revisar y construir a mano.

## Cómo instalar

Los paquetes viven en la raíz, así que se construye como el `src/` de un
workspace colcon:

```bash
mkdir -p ~/eodav_ws/src
cp -r EOD-AV-mod ~/eodav_ws/src/EOD-AV      # esta carpeta
cd ~/eodav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Dependencias fuera de rosdep: Arena SDK (cámaras), `libpcap-dev` (radar),
`pip install um982-driver` (heading), `ros-<distro>-gpsd-client` (GPS).

> Nota: el C++ (`arena_camera_node`) no se pudo compilar en el entorno de
> revisión (requiere el Arena SDK). Los cambios son pequeños y acotados, pero
> **revisá/compilá antes de desplegar**.

---

## 1. Nueva arquitectura de launches (`eod_av_launch/launch/`)

Un launch **por sensor** (las 3 cámaras cuentan como uno) + un **master** que
corre todo sin duplicar ningún launch. Cada nodo se define en UN solo sitio.

| Launch | Qué hace |
|---|---|
| `cameras.launch.py` | 3x Triton (envuelve `arena_camera_node/three_cameras.launch.py`) |
| `lidar.launch.py` | Hesai (nodo del driver + `rviz:=true/false`) |
| `radar.launch.py` | ARS430 (envuelve `radar_live.launch.py`; `rviz`, `pcap_file`, `raw`, `iface`) |
| `gnss.launch.py` | GPS + UM982 opcional (`enable_um982_heading`, `path_child_frame=base_link`) |
| `all_sensors.launch.py` | **master**: los 4 sensores + TF estática, cada uno con flag `enable_*` |
| `static_transforms.launch.py` | (sin cambios) árbol TF `base_link -> sensores` |
| `camera_lidar_sync.launch.py` | reescrito como wrapper fino (cámaras+LiDAR+TF), sin duplicar nodos |
| `camera_lidar_gnss_sync.launch.py` | reescrito como wrapper fino (cámaras+LiDAR+GNSS+TF) |

Por qué el master fuerza `rviz:=false` en lidar y radar: tanto el `start.py` del
Hesai como el `radar_live.launch.py` arrancan su **propio** nodo `rviz2`;
correrlos juntos abría varias ventanas y colisionaba el nombre del nodo. Para
visualizar, corré un sensor individual (traen `rviz:=true` por defecto).

Ejemplos:

```bash
ros2 launch eod_av_launch all_sensors.launch.py
ros2 launch eod_av_launch all_sensors.launch.py enable_radar:=false enable_um982_heading:=true
ros2 launch eod_av_launch radar.launch.py raw:=true
```

## 2. Árbol TF conectado (bug)

`gps_bringup/launch/gps.launch.py`: `path_child_frame` default `gps_link` →
`base_link`. Antes `fix_to_path` publicaba `map -> gps_link` mientras las static
TF colgaban de `base_link`, dejando **dos árboles desconectados** (los sensores
no colgaban del GPS). Ahora `map -> base_link` y todo cuelga de ahí. El comentario
que decía que el default "ya era base_link" ahora coincide con el código.

`camera_lidar_sync.launch.py` ahora **sí** incluye las static transforms (antes
no, así que cámara+LiDAR sin GPS quedaba sin ningún TF).

## 3. Cámaras: gain/exposure auto|fijo y modificables en caliente

> **Ampliado en la v2 (ver abajo).** Antes `set_nodes_gain_()` fijaba
> `GainAuto=Continuous` e ignoraba el `gain` del launch. Ahora gain y exposure
> tienen modo auto/fijo por parámetro **y se cambian en caliente** con
> `ros2 param set`, además de resolución en runtime. Detalle en "Cambios v2".

## 4. Otros arreglos de código (bajo riesgo)

- **`ArenaCameraNode.cpp`**: `publish_an_image_on_trigger_` no hacía `return`
  tras el fallo "no está en trigger mode" y seguía intentando disparar → se
  agregó `return`.
- **`ArenaCameraNode.cpp`**: dos `throw;` sin excepción activa (rutas de QoS no
  soportada) llamaban a `std::terminate()` → reemplazados por
  `throw std::invalid_argument(...)`.
- **`gps_bringup/scripts/fix_to_path.py`**: el `Path` crecía sin límite y se
  republicaba entero a 10 Hz (memoria no acotada, ancho de banda O(n²)). Nuevo
  parámetro `max_path_len` (default 10000) con ventana deslizante. `0` = sin
  límite (comportamiento anterior).
- Eliminado `arena_camera_node/src/Untitled-2.cpp` (archivo muerto, duplicado de
  `ArenaCameraNode.cpp`, no referenciado en el `CMakeLists`).

## 5. Metadatos, dependencias y docs

- **`eod_av_launch/package.xml`**: `description`/`license` (MIT) rellenados y se
  declararon los `exec_depend` de los paquetes que orquesta
  (`arena_camera_node`, `hesai_ros_driver`, `ars430_ros_publisher`,
  `gps_bringup`, `um982_driver`, + `launch`/`launch_ros`/`tf2_ros`). Fija el
  orden de build en colcon y deja que rosdep verifique.
- **`eod_av_launch/setup.py`**: `description`/`license` rellenados.
- **`um982_driver`** (`package.xml` + `setup.py`): licencia `TODO` → `MIT`.
- **`README.md`** raíz: se agregaron radar y um982 (faltaban), la nueva sección
  de launches, y se corrigieron las instrucciones de build (los paquetes van
  bajo `src/` de un workspace; `--from-paths src` no funcionaba desde la raíz).
- **`.gitignore`**: se quitó el `*.md` global que ignoraba TODA la documentación
  salvo README (por eso `setup/README.md` enlazaba a `PTP_SYNC_CONTEXT.md` y
  `guia_ptp_sincronizacion.md`, que quedaban fuera del repo).

---

# Cambios v2 (segunda iteración)

## v2.1 — Cámaras: gain/exposure auto|fijo + resolución, todo en caliente

`arena_camera_node` (`ArenaCameraNode.h` + `.cpp`, `three_cameras.launch.py`).

Nuevos parámetros, **modificables en runtime** con `ros2 param set`:

| Parámetro | Valores | Efecto |
|---|---|---|
| `gain_auto` | `Off` / `Continuous` / `Once` | modo de ganancia |
| `gain` | double | ganancia fija (cuando `gain_auto=Off`) |
| `exposure_auto` | `Off` / `Continuous` / `Once` | modo de exposición |
| `exposure_time` | double (µs) | exposición fija (cuando `exposure_auto=Off`) |
| `width` / `height` | int | **resolución en caliente** (para/reinicia el stream) |

```bash
ros2 param set /arena_cam_enp6s0 gain 13.0            # ganancia fija (pasa a manual)
ros2 param set /arena_cam_enp6s0 gain_auto Continuous # auto-ganancia
ros2 param set /arena_cam_enp6s0 exposure_time 20000.0
ros2 param set /arena_cam_enp6s0 exposure_auto Continuous
ros2 param set /arena_cam_enp6s0 width 1024           # resolución (reinicia stream)
ros2 param set /arena_cam_enp6s0 height 768
```

Cómo funciona por dentro (relevante si vas a mantenerlo):
- La captura (`publish_images_`) pasó a correr en **su propio hilo**; antes era un
  `while` que bloqueaba el executor, así que `ros2 param set` nunca se atendía.
  Ahora `rclcpp::spin` queda libre para los servicios de parámetros.
- El callback `on_set_parameters_` solo **marca flags** (bajo mutex); el hilo de
  captura aplica los cambios entre frames (`apply_pending_reconfig_`), de modo
  que el SDK de Arena se toca desde **un solo hilo**. Gain/exposure se aplican en
  vivo; resolución hace `StopStream → set → StartStream`.
- Convención: setear `gain` (valor) implica pasar a manual (`gain_auto=Off`);
  setear `gain_auto Continuous/Once` vuelve a auto. Igual para exposure. (Nota:
  `ros2 param get gain_auto` puede quedar "desfasado" respecto al modo real tras
  un `set gain`; el comportamiento de la cámara es el correcto.)
- Default del launch: **fijo** (`gain_auto:Off gain:13.0`, `exposure_auto:Off
  exposure_time:28000`) para consistencia entre las 3 cámaras al grabar.

## v2.2 — Cada launch con o sin RViz

Todos los bring-up aceptan `rviz:=true/false` (por defecto **false**, para grabar
sin latencia):

```bash
ros2 launch eod_av_launch cameras.launch.py rviz:=true
ros2 launch eod_av_launch lidar.launch.py   rviz:=false
ros2 launch eod_av_launch all_sensors.launch.py rviz:=true   # UNA sola ventana combinada
```

- El master `all_sensors.launch.py` fuerza apagados los RViz internos de lidar y
  radar y, con `rviz:=true`, abre **una sola** ventana con una config combinada
  (`eod_av_launch/rviz/all_sensors.rviz`) → no más 3 ventanas ni choque del nodo
  `rviz2`.
- Configs nuevas en `eod_av_launch/rviz/`: `cameras.rviz` (3 imágenes),
  `gnss.rviz` (path + TF), `all_sensors.rviz` (grid + TF + lidar + radar + path +
  imágenes). Se instalan vía `setup.py`.

## v2.3 — Frame del radar configurable (comparte frame con el LiDAR si querés)

`ars430_ros_publisher/src/radar_visualizer_node.cpp` + su launch.

- El frame del radar (`radar_fixed`, antes constante en el código) es ahora el
  parámetro **`frame_id`**. Para superponer el radar sobre el LiDAR:
  `frame_id:=hesai_lidar`.
- Nuevo `publish_tf` (bool): el visualizer publicaba `base_link -> radar_fixed`
  por su cuenta, **duplicando** el TF que ya pone `static_transforms`. Ahora se
  puede apagar; el master lo pasa a `false` (static_transforms es el dueño del
  árbol). El visualizer "raw" nunca publica TF.
- **Recomendación:** para *ver* todos los sensores juntos, poné el Fixed Frame de
  RViz en `base_link` (o `map` con GPS) — ya están todos en un único árbol y, con
  las extrínsecas en 0, coinciden en el origen. Forzar `frame_id:=hesai_lidar`
  asume que radar y LiDAR están físicamente en el mismo punto; lo correcto a
  futuro es frames distintos con la extrínseca `base_link→sensor` medida.

## v2.4 — Guía de grabación

Nuevo **`GRABAR_ROSBAGS.md`** (raíz): recomendaciones para grabar sin lag
(headless, NVMe, MCAP, grabar packets crudos del lidar/radar, comprimir imagen,
buffers DDS, CPU performance, PTP, monitoreo de drops) + comando de referencia.

---

# Cambios v3 (tercera iteración)

## v3.1 — RViz por sensor + general, y radar/lidar mismo frame

`eod_av_launch/rviz/` ahora tiene **una config por sensor** + la general:

| Config | Muestra | Fixed Frame |
|---|---|---|
| `cameras.rviz` | 3 imágenes | base_link |
| `lidar.rviz` | nube `/lidar_points` | hesai_lidar |
| `radar.rviz` | nube `/radar_pointcloud_1` + clusters | radar_fixed |
| `gnss.rviz` | `/gps_path` + TF | map |
| `all_sensors.rviz` | **todo** (lidar + radar + 3 cámaras + path + TF) | base_link |

- Cada launch por-sensor usa su propia config (`lidar.launch.py` dejó de usar la
  de Hesai; `radar.launch.py` apaga el RViz interno del paquete y usa
  `radar.rviz`).
- **Radar y LiDAR con el mismo frame en la general:** en `all_sensors.launch.py`,
  `radar_frame_id` por defecto es ahora **`hesai_lidar`**, así el radar y el LiDAR
  comparten frame y se superponen en `all_sensors.rviz`. (Para separarlos:
  `radar_frame_id:=radar_fixed`.)

## v3.2 — `rviz:=true|false` en TODOS los launch

Regla: **standalone por-sensor abre RViz por defecto** (`rviz:=true`) para revisar
ese sensor; los **compuestos son headless** (`all_sensors` y los combos
compat: `rviz:=false`) para grabar sin latencia. `all_sensors.launch.py rviz:=true`
abre UNA sola ventana combinada. Los combos compat fuerzan `rviz:=false` en cada
sub-launch para no abrir/colisionar varios nodos `rviz2`.

## v3.3 — Cámaras en AUTO por defecto

`three_cameras.launch.py`: `gain_auto` y `exposure_auto` pasan a **`Continuous`**
(auto) por defecto en las 3 cámaras. Los valores `gain:13.0` / `exposure_time:28000`
quedan como el valor a usar si se cambia a modo fijo en caliente
(`ros2 param set <cam> gain_auto Off`). Las 3 cámaras siguen nombradas/frameadas
`enp6s0`/`enp7s0`/`enp8s0` (se identifican por `serial`).

## v3.4 — Radar en `enp9s0`

El nodo `radar_publisher` y ambos launch ya usaban `enp9s0`; se corrigió el
`ars430_ros_publisher/README.md` que todavía documentaba `enx00e04c3604d7`.

---

# Cambios v4 (cuarta iteración — tras probar en hardware)

## v4.1 — Exactamente 5 launch en `eod_av_launch/launch/`

```
cameras.launch.py  lidar.launch.py  radar.launch.py  gnss.launch.py  all_sensors.launch.py
```

**Borrados:** `camera_lidar_sync.launch.py`, `camera_lidar_gnss_sync.launch.py` y
`static_transforms.launch.py`.

Las TF estáticas no desaparecieron: se movieron al módulo Python
**`eod_av_launch/eod_av_launch/bringup.py`** (no es un launch), y **cada launch
publica las TF de su propio sensor**. Al componerlos en `all_sensors` el árbol
queda completo, sin transforms duplicadas y sin un launch de transforms aparte.
`bringup.py` también centraliza el nodo RViz.

## v4.2 — RViz: `rviz:=true|false` en los 5, cada uno con su config

| Launch | Config | Muestra | Fixed Frame |
|---|---|---|---|
| `cameras` | `cameras.rviz` | 3 × Image | base_link |
| `lidar` | `lidar.rviz` | PointCloud2 `/lidar_points` | hesai_lidar |
| `radar` | `radar.rviz` | PointCloud2 `/radar_pointcloud_1` + clusters + FOV | base_link |
| `gnss` | `gnss.rviz` | Path `/gps_path` + TF | map |
| `all_sensors` | `all_sensors.rviz` | **todo junto** | base_link |

- **Todos default `rviz:=true`** (antes `all_sensors` era `false`, que es por qué
  no se abría nada al correrlo).
- Las 5 configs se reescribieron con la **estructura canónica completa de RViz2**
  (Panels / Global Options / Tools / Transformation / Views / Window Geometry).
  Las anteriores eran mínimas y RViz podía rechazarlas.
- El nodo RViz ahora va con `output='screen'` y un `LogInfo` que imprime en
  consola la ruta del `.rviz` usado (o avisa si RViz quedó apagado). Así el log
  dice siempre si la visualización se activó — antes fallaba en silencio.

## v4.3 — Alineación radar/LiDAR: es rotación, no nombre de frame

Verificado por el usuario en RViz: el frente del radar caía **90° a la izquierda**
del frente del LiDAR.

**Corregido el enfoque de la v3.** Poner `frame_id:=hesai_lidar` en el radar (v3)
NO alineaba nada: sólo re-etiquetaba la nube. Lo que alinea es la **rotación** de
la TF. Ahora:

- El radar vuelve a su frame propio **`radar_fixed`**.
- `base_link -> radar_fixed` lleva **yaw = -90°** (`radar_yaw_deg`, en grados).
- Si en tu montaje queda girado al revés: `radar_yaw_deg:=90`.
- El visualizer del radar va siempre con `publish_tf:=false`: la TF la publica el
  launch (con el yaw), evitando que se publicara dos veces.

## v4.4 — Radar en `enp9s0` (confirmado en campo)

Confirmado funcionando: `Capturing live on enp9s0`, ~27,6 pkt/s, 3476
detecciones/5 s. Sólo faltaba el `setcap` (documentado en el README del radar).

---

# Cambios v5 (quinta iteración)

## v5.1 — BUG REAL: por qué RViz no abría en `radar` ni en `all_sensors`

**Causa (bug mío, no del entorno):** `IncludeLaunchDescription` aplica cada
`launch_arguments` como un `SetLaunchConfiguration` **en el scope actual y NO lo
revierte** al terminar el include. Como `radar.launch.py` y
`all_sensors.launch.py` son los únicos dos que pasan `rviz: 'false'` a un
sub-launch (para apagar el RViz interno del paquete), ese `false` **se filtraba
al scope propio** y apagaba el nodo `rviz2` declarado *después* del include.

Encaja exacto con lo observado: fallaban esos dos y sólo esos dos —
`cameras`/`lidar`/`gnss` no pasan `rviz` a ningún include y sí abrían.

**Fix:** helper `scoped()` en `bringup.py` = `GroupAction(scoped=True,
forwarding=True)`, que hace push/pop de las configuraciones. Aplicado a **todos**
los includes (`radar`, `all_sensors`, `cameras`, `gnss`).

## v5.2 — El radar sigue al LiDAR (frente atado)

El árbol pasa de hermanos a cadena:

```
antes:  base_link ─┬─> hesai_lidar
                   └─> radar_fixed          (independientes)

ahora:  base_link ──> hesai_lidar ──> radar_fixed   (yaw -90°)
```

TF compone las transformadas en cadena, así que **cualquier movimiento o
recalibración del frame del LiDAR arrastra al radar automáticamente**, sin tocar
nada del radar. Además el `-90°` pasa a significar exactamente lo que se midió:
la rotación del radar **respecto al LiDAR**.

- Nuevo argumento `radar_parent` (default `hesai_lidar`); `radar_parent:=base_link`
  vuelve al comportamiento anterior.
- `radar.rviz` pasa a Fixed Frame `radar_fixed`: corriendo `radar.launch.py` solo,
  el LiDAR no está y la cadena hacia `base_link` queda incompleta.
  `all_sensors.rviz` sigue en `base_link` (ahí la cadena sí existe).

---

## v5.3 — Frente del radar ajustable EN CALIENTE (`tf_tuner`)

Nuevo nodo **`eod_av_launch/tf_tuner.py`** (ejecutable `tf_tuner`): publica la TF
del radar leyendo sus parámetros en cada tick, así que `ros2 param set` se aplica
al instante y se ve en RViz sin reiniciar nada.

```bash
ros2 launch eod_av_launch all_sensors.launch.py      # radar_tf_live:=true por defecto
ros2 param set /radar_tf yaw_deg -45.0               # gira el frente en vivo
ros2 param set /radar_tf x 0.35                      # y tambien traslada
ros2 param get /radar_tf yaw_deg
```

Parámetros: `parent_frame`, `child_frame`, `x`, `y`, `z`, `roll_deg`,
`pitch_deg`, `yaw_deg`, `rate_hz`. Cada cambio se loguea para poder copiar el
valor final a `RADAR_YAW_DEG` en `bringup.py`.

Nuevo argumento **`radar_tf_live`** (default `true`) en `radar.launch.py` y
`all_sensors.launch.py`:

- `true` → nodo `tf_tuner`, TF **dinámica** ajustable (calibración).
- `false` → `static_transform_publisher`, TF **estática** a `/tf_static`
  (preferible para grabar el dataset: sin interpolación/extrapolación al
  reproducir el bag).

Verificado numéricamente que `yaw_deg=-90` rota el frente (+X) 90° en sentido
horario — es decir, lleva a "adelante" lo que se veía a la izquierda.

---

## v5.4 — Radar en yaw 0 y UNA sola referencia común

**`RADAR_YAW_DEG = 0.0`** (antes -90): el radar arranca sin rotación aplicada y
el ángulo se busca en caliente con `ros2 param set /radar_tf yaw_deg <grados>`.

**`RADAR_PARENT_FRAME = base_link`** (antes `hesai_lidar`): todos los sensores
cuelgan directamente de la referencia común, cada uno con su TF propia:

```
map --(GNSS fix_to_path, dinamica)--> base_link --+--> hesai_lidar
                                                  |--> radar_fixed
                                                  +--> camera_enp{6,7,8}s0
```

Motivo: colgar el radar del LiDAR lo dejaba **huérfano** con
`enable_lidar:=false`. Con todo bajo `base_link`, `map -> base_link` mueve el
conjunto como un rígido mientras se traza el recorrido. Verificado que el árbol
queda conectado y con **una sola raíz** en todas las combinaciones
(`all_sensors` completo, sin LiDAR, sin GNSS, y cada launch por separado).
Para atar el radar al LiDAR igualmente: `radar_parent:=hesai_lidar`.

**`all_sensors.rviz` pasa a Fixed Frame `map`** (antes `base_link`), con la
cámara apuntando a `base_link`. Con `base_link` como Fixed Frame el vehículo
quedaba clavado en el centro y era el mundo el que se movía; con `map` el rig
avanza por la escena y **`/gps_path` va quedando dibujado detrás** — que es el
comportamiento buscado al trazar la trayectoria con el GNSS. Sin GNSS, el frame
`map` no existe: cambiar Fixed Frame a `base_link` en el desplegable de RViz.

`radar.rviz` vuelve a Fixed Frame `base_link` (ahora la cadena
`base_link -> radar_fixed` existe aunque se corra el radar solo).

---

## v5.5 — Fix: `radar_fixed` salía girado 90° en `all_sensors`

**Bug introducido en la v5.4.** Al bajar `RADAR_YAW_DEG` a `0.0` quedó un valor
**hardcodeado** en `all_sensors.launch.py`:

| Sitio | valor |
|---|---|
| `bringup.py` (`RADAR_YAW_DEG`) | `0.0` ✅ |
| `radar.launch.py` | `str(RADAR_YAW_DEG)` → `0.0` ✅ |
| `all_sensors.launch.py` | `'-90.0'` **hardcodeado** ❌ |

Síntoma exacto: `base_link` y `hesai_lidar` idénticos (ambos identidad) pero
`radar_fixed` girado 90° — y sólo al correr `all_sensors`, no `radar` solo.

Corregido: `all_sensors.launch.py` hereda `str(RADAR_YAW_DEG)`, así el valor
vive en **un solo sitio**. De paso, `gnss.launch.py` usa `BASE_FRAME` en vez de
la cadena `'base_link'` literal, por el mismo motivo.

---

## v5.6 — Todos los sensores con la orientación por defecto (identidad)

Ningún sensor lleva rotación aplicada: las 5 TF son **identidad**, así que cada
frame queda con la orientación por defecto de RViz (+X adelante, +Y izquierda,
+Z arriba, REP-103):

```
base_link -> hesai_lidar      (0,0,0) (0,0,0)
base_link -> radar_fixed      (0,0,0) (0,0,0)
base_link -> camera_enp6s0    (0,0,0) (0,0,0)
base_link -> camera_enp7s0    (0,0,0) (0,0,0)
base_link -> camera_enp8s0    (0,0,0) (0,0,0)
```

Además **`radar_tf_live` pasa a `false` por defecto**: el radar usa
`static_transform_publisher` igual que el resto (TF estática en `/tf_static`),
sin trato especial. El ajuste en caliente sigue disponible a un flag:

```bash
ros2 launch eod_av_launch all_sensors.launch.py radar_tf_live:=true
ros2 param set /radar_tf yaw_deg <grados>
```

---

# Cambios v6 — optimización de grabación (Live / Record / Playback)

Basado en el diagnóstico medido en tu banco: `Speed: 1000Mb/s`, `bw` pico
**115.21 MB/s = 922 Mbps** (techo de GigE), rate ~9.5–11 Hz con jitter de
±33 ms, disco escribiendo 340–372 MB/s al 10 % de `%util`, CPU 68 % idle.

**Causa raíz confirmada:** las cámaras pedían `rgb8`, o sea que el *firmware*
interpolaba el Bayer y mandaba 3 B/px (9.44 MB/frame), **saturando el enlace
GigE**. El framerate y el jitter los imponía la red, no el sensor (lo probó tu
test: bajar la exposición a 5 ms no subió el rate).

## v6.1 — Cámaras: `bayer_rggb8` + tope de framerate

- **`three_cameras.launch.py` pasa a `bayer_rggb8`** (1 B/px, **3.15 MB/frame**).
  El nodo ya soportaba Bayer (`pixelformat_translation.h` mapea
  `bayer_rggb8 ↔ BayerRG8`); solo faltaba usarlo. Enlace del **97 % → ~30 %**.
  Es *más* crudo que antes: sin interpolación del firmware.
- **Nuevo parámetro `frame_rate`** (`AcquisitionFrameRate` + `Enable`) en
  `ArenaCameraNode`, modificable en caliente. **Es imprescindible**: al liberar
  ancho de banda la cámara subiría sola hacia ~36 fps y se comería el margen
  recién ganado. Default 12 fps.
- El launch de las 3 cámaras se reescribió con un bucle sobre una lista
  `(iface, serial)`: eran 3 bloques idénticos copiados.

## v6.2 — Tres modos de operación

`mode:=live|record` en `all_sensors` y en cada launch por sensor, más
`playback.launch.py`:

| | live | record | playback |
|---|---|---|---|
| cámaras | bayer @12fps | **igual** | debayer opcional |
| LiDAR | `/lidar_points` | **`/lidar_packets`** (4× menos) | packets → nube |
| radar | pipeline completo | **solo el crudo** | pipeline completo |
| GNSS | `/fix` + trayectoria | **solo `/fix`** | reconstruye trayectoria |
| RViz | 1 ventana | apagado | 1 ventana |

Las cámaras **no cambian entre live y record**: se verifica exactamente lo que
se graba (evita la divergencia clásica entre "lo que vi" y "lo que quedó").

## v6.3 — Cambios por paquete

- **`hesai_ros_driver`**: nuevos `config_record.yaml` (`send_packet_ros: true`,
  `send_point_cloud_ros: false`) y `config_playback.yaml` (`source_type: 3`).
  Se pasan por el parámetro `config_path`, que el nodo ya soportaba y el
  `start.py` tenía comentado. El `config.yaml` original queda intacto.
- **`ars430_ros_publisher`**: `processing:=false` deja solo `radar_publisher`
  (grabación); `capture:=false` corre solo el procesamiento alimentado desde un
  bag (playback). `raw_cloud` y `stats` pasan a ser opcionales.
- **`gps_bringup`**: `enable_path:=false` no lanza `fix_to_path`. Era el único
  componente con **costo creciente en el tiempo** (republica el `Path` entero a
  10 Hz → ~50 GB en una sesión de una hora), y es 100 % reconstruible desde `/fix`.

## v6.4 — Script de grabación

Nuevo `eod_av_launch/scripts/record_dataset.sh`: lista **explícita** de tópicos
crudos, `--storage mcap`, caché de 1 GiB para amortiguar las ráfagas de
escritura que se vieron en `iostat`, troceado en 4 GiB, y aviso previo si chrony
no está sincronizado.

## Resultado esperado

| | antes | después |
|---|---|---|
| 3× cámaras | 345 MB/s | **113 MB/s** |
| LiDAR | 55 MB/s (nube) | **13.6 MB/s** (packets) |
| `/gps_path` | hasta 20 MB/s creciente | **0** |
| **Total** | **~400 MB/s — 1.44 TB/h** | **~127 MB/s — 457 GB/h** |

**3.1× menos**, sin perder información. Y el enlace GigE pasa de saturado a
~30 %, que es lo que debería eliminar el jitter.

---

# Cambios v7 — imagen en B/N y unificación Live+Playback

## v7.1 — Por qué las cámaras se veían en blanco y negro

**No era el driver ni el `encoding`.** Se verificó que el nodo pone
`encoding = "bayer_rggb8"` correctamente y que el dato es 1 B/px real.

**Causa: RViz no demosaica.** Trata cualquier encoding `bayer_*` como
luminancia de un solo canal, así que dibuja el mosaico crudo en escala de
grises. La información de color **está** en el patrón; falta interpolarla.

*(Corrección: en la v6 afirmé que RViz mostraría Bayer en color vía cv_bridge.
Es falso.)*

**Solución:** el demosaico es *procesamiento*, así que vive en la capa de
procesamiento — `image_proc/debayer_node` produce `.../image_color`, y las
configs de RViz apuntan ahí en vez de al tópico crudo. Nueva dependencia
`image_proc` en `package.xml`.

## v7.2 — Live y Playback unificados: dos capas, no tres modos

Tenías razón: eran el mismo trabajo con distinta fuente. Ahora la separación es
**fuente vs procesamiento**, no tres modos paralelos:

```
FUENTE (crudo)                   PROCESAMIENTO (derivado + vista)
------------------------------   ---------------------------------
cameras -> .../image (bayer)  -> debayer     -> .../image_color
lidar   -> /lidar_packets     -> driver ST3  -> /lidar_points
radar   -> /unfiltered_...    -> filtros     -> nubes + clusters
gnss    -> /fix               -> fix_to_path -> /gps_path + TF
                              -> RViz

live     = FUENTE(sensores) + PROCESAMIENTO
record   = FUENTE(sensores)                    -> rosbag
playback = FUENTE(rosbag)   + PROCESAMIENTO
```

- **Nuevo `processing.launch.py`**: todo el procesamiento y la visualización,
  **agnóstico de la fuente**. Con `bag:=<ruta>` es el "playback".
- **`playback.launch.py` eliminado**: era `processing.launch.py bag:=...`.
- **Los 4 launch por sensor son ahora solo fuentes de crudo.** El LiDAR publica
  únicamente packets (ya no la nube), el radar solo el publisher, el GNSS solo
  `/fix`, las cámaras solo Bayer.
- Cuando un launch por sensor se abre con `rviz:=true`, **incluye
  `processing.launch.py`** restringido a ese sensor: cero lógica duplicada, y la
  vista es idéntica a la de playback.
- `all_sensors.launch.py` queda en dos modos: `live` (fuentes + procesamiento) y
  `record` (solo fuentes).

**Consecuencia buscada:** un bag reproducido y un sensor conectado atraviesan
exactamente el mismo grafo de nodos, así que producen la misma salida.

## v7.3 — Nota sobre el archivo de corrección del LiDAR

Al pasar la expansión packets→nube a la capa de procesamiento, ese nodo necesita
el **archivo de corrección angular**:

- **En live** el LiDAR está conectado y el config conserva `use_ptc_connected`,
  así que puede bajarlo del sensor como siempre.
- **En playback no hay sensor**: hay que pasarlo con
  `processing.launch.py lidar_correction:=/ruta/archivo`. El launch avisa en el
  log si falta.

Es un requisito real del dataset de todos modos: **sin ese archivo, un bag de
packets no se puede reconstruir**. Guardalo junto a las grabaciones.

---

## v7.4 — Fix: `image_proc` ausente tumbaba TODO el launch

Un `Node(package='image_proc', ...)` cuyo paquete no está instalado lanza
`PackageNotFoundError` **al construir el LaunchDescription**, y eso **aborta el
launch entero** — se caían las cámaras, el LiDAR, el radar y el GPS, no solo el
debayer. Fallo de robustez mío.

Corregido con degradación en vez de aborto:

- `processing.launch.py` comprueba con `get_package_share_directory` si
  `image_proc` (y `rviz2`) existen **antes** de crear los nodos.
- Si falta `image_proc`: avisa en el log con el comando de instalación y sigue
  sin debayer.
- Además, como las configs de RViz apuntan a `.../image_color` (que no
  existiría), se genera una copia temporal que mira el tópico crudo — se verá en
  blanco y negro, pero **se ve**, en vez de quedar la vista muda.

Para tener color:

```bash
sudo apt install ros-jazzy-image-proc
```

---

## v7.5 — Debayer propio, sin depender de `image_proc`

`image_proc` demostró ser un punto frágil (no instalado → launch abortado). Se
reemplaza por un nodo del propio paquete: **`eod_av_launch/debayer_node.py`**
(OpenCV + numpy, ambos ya presentes con ROS desktop).

Ventajas sobre la dependencia externa:

- **No puede faltar**: se instala con el paquete.
- **QoS explícito RELIABLE**, igual que el publisher del driver, así no hay
  incompatibilidad silenciosa de QoS.
- **Limita la tasa de conversión** (`debayer_rate_hz`, default 5 Hz). Demosaicar
  los 12 fps de las 3 cámaras generaría **340 MB/s de RGB solo para la
  pantalla** — justo el tráfico que se quiso evitar. Para verificar que la
  cámara ve bien, 5 fps sobran. Medido: 3,4 ms/frame → **~5 % de un core**.
- **Passthrough** si la entrada no es Bayer, así la vista sigue funcionando con
  `pixelformat:=rgb8`.

Mapeo ROS→OpenCV verificado con mosaicos sintéticos en los **4 patrones**
(`rggb`/`bggr`/`gbrg`/`grbg`): en todos, un fotosito rojo encendido produce un
píxel dominante en R. (El mapeo parece cruzado — `bayer_rggb8` →
`COLOR_BayerBG2RGB` — porque OpenCV nombra el patrón desplazado respecto a ROS;
es el mismo criterio que usa image_proc.)

---

# Cambios v8 — dos modos y simplificación

Tras el análisis: el lag lo resolvió **Bayer** (cambio de driver), no la
separación raw/procesamiento. Se conserva lo que funcionó y se simplifica lo que
daba problemas.

## v8.1 — Dos modos

```
1. RECORD          = FUENTES -> rosbag           (all_sensors.launch.py mode:=record)
2. LIVE / PLAYBACK = FUENTES o BAG + PROCESAMIENTO
                     live     -> all_sensors.launch.py
                     playback -> processing.launch.py bag:=<bag>
```

Live y playback son el **mismo** pipeline con distinta fuente.

## v8.2 — LiDAR: se graba la nube (bag autocontenido)

Revertido a `/lidar_points`. Los packets pesaban 4× menos pero exigían el
**archivo de corrección angular** para reconstruirse: una dependencia externa
que vuelve frágil al dataset. Los 41 MB/s de diferencia ya no aprietan.

**Eliminados** `config_record.yaml` y `config_playback.yaml`, el nodo de
expansión en processing y el argumento `lidar_correction`. El driver usa el
`config.yaml` de siempre.

## v8.3 — La TF sí se graba, el Path no

`fix_to_path` gana el parámetro **`publish_path`** y separa dos cosas de costo
muy distinto:

| | Costo | ¿Se graba? |
|---|---|---|
| TF `map -> base_link` | constante (1 msg/fix) | **Sí** — es la pose del vehículo |
| `/gps_path` | **creciente** (~50 GB/h) | No — se regenera desde `/fix` |

Así el bag lleva la pose sin pagar el crecimiento O(n²).

## v8.4 — Debayer desactivado por defecto

RViz no demosaica Bayer: muestra B/N, y **eso alcanza para verificar** encuadre,
foco y exposición. El color es análisis, no verificación.

- `debayer` pasa a `false` por defecto; las configs de RViz apuntan al tópico
  crudo. Con `debayer:=true` el launch genera al vuelo una copia del `.rviz` que
  mira `.../image_color`.
- Camino recomendado para color: **`rqt_image_view`**, que usa cv_bridge y sí
  demosaica — cero nodos, cero tópicos, cero tráfico extra.

Esto elimina 3 nodos, 3 tópicos y ~142 MB/s locales del camino por defecto, que
era la única parte del diseño que venía dando problemas.

## Presupuesto final

| | Original | Ahora (record) |
|---|---|---|
| 3× cámaras | 345 MB/s | **113 MB/s** |
| LiDAR | 55 MB/s | 55 MB/s |
| Radar + GNSS | ~0.05 MB/s | ~0.05 MB/s |
| `/gps_path` | hasta 20 MB/s creciente | **0** |
| **Total** | **~400 MB/s — 1.44 TB/h** | **~168 MB/s — 605 GB/h** |

**2.4×**, con jitter de 33 ms → 0.64 ms y sin dependencias externas.

---

# Cambios v9 — cumplimiento estricto del requerimiento de dos modos

Cierra los dos incumplimientos detectados en la revisión de la v8.

## v9.1 — `fix_to_path` fuera del modo Record

**Antes:** `gps.launch.py` lanzaba `fix_to_path` **siempre** (en record iba con
`publish_path:=false`, pero seguía publicando la TF `map → base_link` en `/tf`).
Eso incumplía "no debe formar parte del flujo mínimo de adquisición" — y con
`ros2 bag record -a` ese `/tf` derivado se habría grabado.

**Ahora:** el nodo tiene `condition=IfCondition(enable_path)` y **`enable_path`
por defecto es `false`**. En el flujo de `eod_av_launch` nunca se activa: quien
lanza `fix_to_path` es siempre `processing.launch.py`.

`gps_bringup` queda como **fuente pura**: publica `/fix` y `/extended_fix`, nada
derivado. (`enable_path:=true` sigue disponible para usar ese paquete por su
cuenta, fuera de `eod_av_launch`.)

## v9.2 — Misma cadena exacta en live y playback

**Antes:** en live, `all_sensors` pasaba `enable_gnss:='false'` a `processing`,
así que el Path lo publicaba `fix_to_path` (del bring-up) mientras que en
playback lo publicaba `fix_to_path_view` (de processing): **dos nodos distintos
con nombres distintos** para lo mismo.

**Ahora:** `processing` recibe el `enable_gnss` real y el nodo se llama
`fix_to_path` en ambos casos. **Un solo nodo, un solo nombre, un solo grafo**,
tanto con sensores como con un bag.

## v9.3 — `fix_to_path` opcional en dos ejes

En `processing.launch.py`:

| Argumento | Default | Efecto |
|---|---|---|
| `fix_to_path` | `true` | lanzar o no el nodo (TF `map → base_link`) |
| `publish_path` | `true` | además de la TF, publicar `/gps_path` |

Cumple "debe ser opcional y utilizarse únicamente cuando se requiera visualizar
la trayectoria".

## Resumen de nodos por modo

| | RECORD | LIVE | PLAYBACK |
|---|---|---|---|
| `arena_camera_node` ×3 | ✔ | ✔ | — (bag) |
| `hesai_ros_driver_node` | ✔ | ✔ | — (bag) |
| `radar_publisher` | ✔ | ✔ | — (bag) |
| `gpsd_client` | ✔ | ✔ | — (bag) |
| `static_transform_publisher` ×5 | ✔ | ✔ | — (`/tf_static` del bag) |
| `radar_processor`/`visualizer`/`clusters` | — | ✔ | ✔ |
| `fix_to_path` | **—** | ✔ | ✔ |
| `debayer_node` ×3 (opcional) | — | ✔ | ✔ |
| `rviz2` | — | ✔ | ✔ |

Las columnas **LIVE y PLAYBACK son idénticas** en la mitad de procesamiento.

---

## Pendiente (no incluido — requiere hardware/decisión)

- **Extrínsecas reales** en `static_transforms.launch.py` (hoy todo en 0, a
  propósito). Es lo que más valor agrega antes de la próxima grabación calibrada.
- **Compresión de imagen / `image_transport`** para grabar (3×2048×1536 rgb8
  ≈ 9,4 MB/frame por cámara). Ver `GRABAR_ROSBAGS.md` §5 — necesita elegir
  JPEG vs Bayer-offline y tocar el nodo/pixelformat.
- CI (colcon build + lint) — no se agregó.

> **Recordá:** el C++ (`arena_camera_node`, `ars430_ros_publisher`) NO se pudo
> compilar en el entorno de revisión (falta el Arena SDK / ROS). Se validó
> balance de llaves y sintaxis Python/YAML, pero **compilá y probá en hardware**
> antes de grabar en serio, sobre todo el cambio de resolución en caliente
> (stop/start del stream) y el callback de parámetros.

---

# Cambios v10 — fusión con la rama alternativa (mode explícito + arreglos)

Esta iteración toma v9 tal cual y le aplica 8 cambios que salieron de comparar
esta copia contra una reimplementación independiente hecha desde el repo
original. **No se tocó nada del driver de cámaras, ni `tf_tuner`, ni
`record_dataset.sh`, ni la cadena de radar**: todo eso es de esta rama y se
conserva.

## v10.1 — `mode:=live|record` explícito en los 4 bring-up

Antes `mode` existía solo en `all_sensors.launch.py`, y los launches por sensor
usaban `rviz:=false` como sinónimo de "modo grabación". Funcionaba, pero
mezclaba dos cosas distintas: `rviz` decidía además si corría el procesamiento,
así que no había manera de tener el debayer o los filtros del radar sin abrir
una ventana.

Ahora los dos ejes son independientes:

```bash
ros2 launch eod_av_launch cameras.launch.py mode:=record      # solo lo crudo
ros2 launch eod_av_launch cameras.launch.py mode:=live rviz:=false  # sin ventana
```

`mode` decide si corre el procesamiento; `rviz`, si se abre la ventana. En
`mode:=record` no corre ningún nodo derivado aunque se pase `rviz:=true`.

Implementación: `bringup.py` gana `if_live()`, `if_record()` y `validate_mode()`.
Las dos primeras son condiciones (`IfCondition` sobre un `PythonExpression`); la
tercera es una acción que aborta el launch si `mode` está mal escrito, porque
una condición que no matchea no avisa: `mode:=recrod` arrancaría a medias en
silencio.

`all_sensors.launch.py` ahora incluye los bring-up con `mode:='record'` — o sea,
como fuentes puras — y agrega el pipeline una sola vez él mismo. Si les pasara
`mode:='live'`, cada uno incluiría su propio `processing.launch.py` y
terminarían corriendo cuatro RViz.

## v10.2 — BUG: `gnss.launch.py` en vivo no publicaba nada derivado

`gnss.launch.py` pasaba `enable_path:='false'` al bring-up **y**
`enable_gnss:='false'` a `processing.launch.py`. Resultado: `fix_to_path` no
corría en ningún lado. `ros2 launch eod_av_launch gnss.launch.py` abría
`gnss.rviz` — que muestra `/gps_path` y usa `Fixed Frame: map` — y la ventana
quedaba vacía, sin TF `map -> base_link` y sin trayectoria. El comentario del
código decía "el bring-up de arriba ya publica el Path en este caso", que era
justo lo contrario de lo que hacían las dos líneas de arriba.

Ahora el include de processing va con `enable_gnss:='true'` y le pasa además
`fix_to_path` y `publish_path`, que antes ni llegaban.

## v10.3 — TF estática `base_link -> gps_link`

`gpsd_client` estampa `/fix` con `frame_id: gps_link`, pero ese frame no existía
en el árbol: cualquier consumidor que quisiera llevar el fix a `base_link`
fallaba con *"Missing transform from frame gps_link"*. Se agrega `gnss_tf()` en
`bringup.py` y lo publica `gnss.launch.py`.

Va colgado de `base_link` y **no** al revés: la TF dinámica de `fix_to_path` es
`map -> base_link`, y un frame no puede tener dos padres. Identidad y sin medir,
como el resto de las extrínsecas.

## v10.4 — `fix_to_path` estampa con el tiempo del fix

Usaba `self.get_clock().now()`. Ahora usa `msg.header.stamp`, que con
`use_gps_time` en `gpsd_client` es tiempo GPS/PPS: el mismo reloj disciplinado
que usan las cámaras y el LiDAR. Dos consecuencias: la pose queda alineada con
el resto del dataset, y reproducir un bag da el resultado idéntico a la corrida
en vivo sin depender de `use_sim_time`, de `--clock` ni del rate del playback.

## v10.5 — Tests del mapeo Bayer (`test/test_debayer.py`)

Diez tests que corren sin ROS ni hardware. Verifican que la tabla
`_BAYER_TO_CV` ponga cada canal donde corresponde en los 4 patrones, con la
contraprueba de que leer un mosaico con el patrón equivocado da otro color.

El nombre del patrón en ROS y en OpenCV no coinciden (ROS nombra el bloque 2x2
desde arriba a la izquierda; OpenCV, desde el segundo píxel de la segunda fila),
así que la tabla correcta parece cruzada — `bayer_rggb8 -> COLOR_BayerBG2RGB` —
y es facilísimo "corregirla" mal. El síntoma sería una imagen que se ve perfecta
salvo que los rojos y los azules están intercambiados. Confirmado: la tabla que
ya estaba es la correcta en los 4 patrones.

## v10.6 — Se elimina `MODE_PLAYBACK`

Quedaba declarado en `MODES` desde la etapa de tres modos (v6), aunque desde v7
nadie lo distinguía. `resolve_mode` lo aceptaba y `all_sensors` lo trataba como
"no es record", así que `mode:=playback` **levantaba los sensores físicos** —
exactamente lo contrario de lo que promete el nombre. `MODES` ahora es
`('live', 'record')`.

## v10.7 — `use_sim_time:=auto`

Era un booleano en `false` que había que acordarse de poner junto con `--clock`;
olvidarlo dejaba a los nodos derivados y a RViz mirando un reloj distinto al de
los datos. Ahora el default es `auto`: `true` cuando hay `bag:=`, `false` con
sensores en vivo. `true|false` sigue forzando el valor.

## v10.8 — El debayer se suscribe BEST_EFFORT

Se suscribía RELIABLE. Un suscriptor RELIABLE solo matchea con publicadores
RELIABLE; uno BEST_EFFORT matchea con los dos. Suscribirse RELIABLE ataba el
nodo a que la cámara y el `ros2 bag play` de turno ofrecieran exactamente esa
política: si alguna vez no lo hacen, el nodo arranca, no recibe nada y no hay
error que lo explique. Publica RELIABLE, por el mismo motivo del otro lado.

No se pierde nada: este nodo ya descarta frames a propósito (`max_rate_hz`), es
para mirar, no para grabar.

## Extras de esta pasada

- `scoped()` en el include de `radar_live.launch.py` dentro de
  `processing.launch.py`. No rompía nada por el orden de evaluación, pero es la
  regla del proyecto y es exactamente el patrón que causó el bug v5.1.
- Lint limpio: `flake8` y `pep257` (con los ignores por defecto de ament) no
  reportan nada en `eod_av_launch`, `gps_bringup/scripts`, `um982_driver` ni en
  los launch de `arena_camera_node` y `ars430_ros_publisher`.
- `fix_to_path`: se saca la variable `z` que se calculaba y no se usaba, y el
  docstring pasa a describir la TF como salida principal.

## Lo que NO se cambió (a propósito)

- **`radar_visualizer` sigue con `publish_tf` default `true`.** Corriendo
  `radar_live.launch.py` suelto vuelve a publicar `base_link -> radar_fixed`, y
  si en algún momento hay extrínsecas medidas en `bringup.py`, ese nodo las
  pisaría (el buffer estático de tf2 se queda con el último que llegó). Dentro
  de `eod_av_launch` no pasa: `processing.launch.py` pasa `publish_tf:='false'`.
- **`all_sensors.rviz` sigue con `Fixed Frame: map`.** Sin fix GPS no dibuja
  nada; el propio archivo lo comenta y sugiere cambiarlo a `base_link` desde el
  desplegable.
- **La reescritura de la config de RViz a `/tmp`** cuando `debayer:=true`.

---

# Cambios v11 — GNSS: `/fix` mudo y `map` inexistente en RViz

Síntoma reportado: con **sólo el GNSS conectado**, RViz dice *"Fixed Frame [map]
does not exist"* y `/fix` no muestra nada. Cámaras, LiDAR y radar funcionan.

Los dos síntomas son el mismo problema visto en dos lugares: sin `/fix` no hay
pose, y `fix_to_path` publica la TF `map -> base_link` recién después del primer
fix válido, así que el frame `map` nunca llega a existir.

## v11.1 — CAUSA RAÍZ: faltaba un parámetro obligatorio

`GPSDClientComponent` declara sus **7** parámetros con la sobrecarga *sólo-tipo*
de rclcpp:

```cpp
this->declare_parameter("use_gps_time",                 rclcpp::PARAMETER_BOOL);
this->declare_parameter("check_fix_by_variance",        rclcpp::PARAMETER_BOOL);
this->declare_parameter("override_augmentation_source", rclcpp::PARAMETER_BOOL);
this->declare_parameter("frame_id",                     rclcpp::PARAMETER_STRING);
this->declare_parameter("publish_rate",                 rclcpp::PARAMETER_INTEGER);
this->declare_parameter("host",                         rclcpp::PARAMETER_STRING);
this->declare_parameter("port",                         rclcpp::PARAMETER_INTEGER);
```

Esa sobrecarga declara el parámetro **sin valor por defecto**: exige un
override, y si falta lanza `NoParameterOverrideProvidedException`. El
constructor del componente revienta, el `component_container` **no lo carga
pero sigue corriendo**, y el launch parece haber arrancado bien.

`gps.launch.py` pasaba 6 de 7. Faltaba **`override_augmentation_source`**.

Arreglo: se pasan los 7, con los mismos valores que el nodo usa internamente
como default (`get_parameter_or`), así que no cambia ningún comportamiento —
sólo satisface el requisito de que existan.

> Verificación en la máquina: `ros2 param list /gpsd_client` tiene que listar
> los 7. Si `ros2 node list` muestra `/gpsd_client_container` pero **no**
> `/gpsd_client`, el componente sigue sin cargar.

## v11.2 — El segundo modo de falla silenciosa: gpsd sin device

Aunque el componente cargue, `step()` hace:

```cpp
if (p == nullptr || !parser_->isOnline(*p))
  return;
```

con `isOnline()` = `data.online.tv_sec > 0 || data.online.tv_nsec > 0`. Mientras
gpsd no tenga un device activo, `online` es 0 y el nodo **descarta todo antes de
publicar**: el tópico `/fix` existe y no sale un solo mensaje.

No se cambia código por esto (es correcto), pero ahora se diagnostica.

## v11.3 — `diagnose_gnss.sh`

`ros2 run gps_bringup diagnose_gnss.sh` recorre los 6 eslabones —adaptador USB,
servicio gpsd, gpsd ve al receptor, lock del receptor, nodo ROS + `/fix`, TF
`map -> base_link`— y dice en cuál se corta, con el comando concreto para
seguir. Separa explícitamente "está roto" de "falta señal".

## v11.4 — `gnss_watchdog`

Nodo nuevo en `gps_bringup` que vigila `/fix` y reporta por consola en cuál de
los tres estados está el GNSS:

| Estado | Significa |
|---|---|
| **MUDO** | no llega nada → software/gpsd (v11.1 o v11.2) |
| **SIN FIX** | llegan mensajes con `NaN` → la cadena ROS está bien, falta señal |
| **CON FIX** | hay posición → ya existe `map -> base_link` |

Sin esto los tres se ven igual: RViz vacío y ningún error.

Corre en los **dos modos** (`gnss_watchdog:=false` lo apaga). **No publica
ningún tópico**: aparece en `ros2 node list` pero `ros2 node info` no le lista
publishers más allá de `/rosout`, así que no ensucia el bag. Durante una
grabación larga, enterarse de que el GNSS perdió el lock vale más que el ruido
en la lista de nodos.

## v11.5 — `fix_to_path` explica por qué no hay `map`

Cuando llegan mensajes con `lat/lon` en NaN, ahora avisa (throttled a 10 s) que
no va a publicar la TF hasta el primer fix válido y que el error de RViz es
consecuencia de eso, no de una falla. Al primer fix válido loguea la posición y
que el frame `map` ya existe.

## v11.6 — `gnss.rviz` con `Fixed Frame: base_link`

Con `map`, sin lock RViz no dibuja **nada** y el único síntoma es *"Fixed Frame
[map] does not exist"* — que se lee como software roto. Con `base_link` la
ventana funciona siempre (se ve el árbol TF del vehículo) y `/gps_path` aparece
cuando hay posición. Para seguir la trayectoria en un marco fijo, cambiar a
`map` desde el desplegable una vez que haya fix.

## Lo que NO se cambió

`check_fix_by_variance` sigue en `false`, y es a propósito: con `true`,
`parseNavSatFix()` devuelve `nullopt` cuando la varianza no es válida y `/fix`
queda mudo justo mientras no hay fix — que es cuando más falta hace ver que el
receptor responde.

---

# Cambios v12 — GNSS: el serial del adaptador estaba hardcodeado

Salida de `diagnose_gnss.sh` en la máquina de pruebas (sólo el GNSS conectado):

```
1/6  Adaptador USB-serial (/dev/gps_pps)
  [FALLA] /dev/gps_pps NO existe
2/6  Servicio gpsd
  [FALLA] no hay ningun gpsd corriendo
3/6  gpsd ve al receptor
  [ OK ]  gpsd responde en localhost:2947      <-- contradice al paso 2
  [FALLA] gpsd corre pero NO tiene ningun device abierto
```

Y `udevadm`: el adaptador conectado tiene `ID_SERIAL_SHORT=D30HIC4U`, mientras
que `GPS_SERIAL` valía `"A5069RR4"`.

## v12.1 — CAUSA RAÍZ: `GPS_SERIAL` fijo en el script

La regla udev era `ATTRS{serial}=="A5069RR4"`. Con otro adaptador **no matchea
nunca**, `/dev/gps_pps` no se crea, y de ahí cae todo en cascada:
`gps-pps-ldattach.service` hace `ldattach` sobre un device inexistente,
`gpsd-eodav.service` lo tiene como `Requires=` y tampoco arranca. Aguas abajo el
único síntoma es "`/fix` no publica nada", que parece un problema de ROS.

El script sólo imprimía un `⚠` y **seguía**, construyendo servicios que no
podían funcionar.

Ahora:

* `GPS_SERIAL` vacío = **autodetectar**. Con un solo `ttyUSB*`/`ttyACM*`
  conectado lo toma; con varios, aborta y los lista.
* Si se fija a mano y no coincide con ningún adaptador conectado, **aborta**
  explicando que ése es el modo de falla silencioso.
* `idVendor` sale del adaptador detectado en vez de estar fijo en `0403` (FTDI),
  así que funciona con adaptadores de otro fabricante.
* Si tras `udevadm trigger` el symlink sigue sin aparecer, **aborta** en vez de
  avisar y continuar.
* Override por entorno: `sudo GPS_SERIAL=D30HIC4U ./setup_ptp_sync.sh`

## v12.2 — La contradicción entre los pasos 2 y 3: `gpsd.socket`

El paso 2 decía "no hay ningún gpsd corriendo" y el 3, dos líneas después,
"gpsd responde en localhost:2947". No es un bug del diagnóstico: es el
`gpsd.socket` de Ubuntu, que escucha en 2947 por activación de socket y levanta
un gpsd **sin ningún device** en cuanto alguien se conecta. `setup_ptp_sync.sh`
lo deshabilita, así que verlo activo significa que el setup nunca corrió.

Ese gpsd responde pero tiene `online == 0`, y `gpsd_client` descarta todo antes
de publicar → `/fix` mudo. El diagnóstico ahora detecta el caso y lo nombra.

## v12.3 — `GNSS_ONLY=1`

El paso `[2/8]` lee `ethtool -T` de las 4 interfaces de cámaras y LiDAR. Con
`set -euo pipefail` y esos sensores desconectados, el script **abortaba antes de
llegar a configurar el GPS** — justo en el escenario de probar un sensor solo en
el banco.

```bash
sudo GNSS_ONLY=1 ./setup_ptp_sync.sh
```

Configura sólo udev + ldattach + gpsd + refclocks de chrony, y saltea los pasos
7 y 8 (PTP sobre las interfaces de red).

## v12.4 — El paso 1 del diagnóstico ahora dice el porqué

Cuando falta `/dev/gps_pps`, lista los adaptadores conectados con su serial e
`idVendor`, lee el serial que espera la regla udev instalada, y marca si
coinciden o no. En este caso habría respondido en una línea.

---

# Cambios v13 — el paquete `gpsd_client` no estaba instalado (y dos bugs del diagnóstico)

Con el adaptador ya resuelto (v12), el launch mostró el error real:

```
[component_container-1] [ERROR] [gpsd_client_container]: Could not find requested resource in ament index
[ERROR] [launch_ros.actions.load_composable_nodes]: Failed to load node 'gpsd_client' ...
```

## v13.1 — CAUSA: `gpsd_client` no instalado

"Could not find requested resource in ament index" significa que **no hay
ningún plugin registrado con ese nombre**: el paquete no está en el sistema.
`gpsd_client` no está vendorizado en este workspace; se instala aparte
(`ros-<distro>-gpsd-client`, liberado para Jazzy) o vía
`rosdep install --from-paths src --ignore-src -r -y`, ya que el `package.xml`
de `gps_bringup` lo declara como `exec_depend`.

No tenía nada que ver con los parámetros (v11) ni con gpsd (v12).

`gps.launch.py` ahora verifica el paquete al construir el LaunchDescription y
aborta con un mensaje que dice qué falta y cómo instalarlo, en lugar de dejar
que el contenedor arranque vacío con un error opaco en medio del log.

## v13.2 — BUG del diagnóstico: match por subcadena

El paso 5 hacía `grep -q '/gpsd_client'` sobre la salida de `ros2 node list`.
Como `/gpsd_client_container` **contiene** `/gpsd_client`, el chequeo daba
`[ OK ] el nodo /gpsd_client existe (el componente cargo)` justo en el caso en
que el componente NO había cargado — precisamente el error que la comprobación
existía para detectar.

Ahora usa match exacto (`grep -qx`), y distingue el contenedor huérfano.

## v13.3 — BUG del diagnóstico: "el tópico /fix existe"

Se comprobaba con `ros2 topic list`, que lista un tópico aunque sólo tenga
**suscriptores** — y aquí siempre hay dos (`fix_to_path` y `gnss_watchdog`).
O sea que "el tópico existe" no decía absolutamente nada sobre si el GNSS
publicaba.

Ahora cuenta publicadores con `ros2 topic info /fix` y reporta
"`/fix` no tiene NINGÚN publicador" cuando corresponde.

## v13.4 — Chequeo del paquete en el paso 5

El paso 5 arranca comprobando `ros2 pkg prefix gpsd_client` y
`ros2 component types | grep GPSDClientComponent`, antes de mirar nodos. Es la
causa más básica y la que da el error más opaco, así que va primero.

---

# Cambios v14 — `GNSS_ONLY=1` escribía los servicios pero no los arrancaba

Reportado desde la máquina de pruebas: el script termina con "Cadena del GNSS
lista" y acto seguido

```
$ systemctl is-active gpsd-eodav.service
inactive
$ gpspipe -w -n 5 localhost:2947
gpspipe: could not connect to gpsd localhost:2947
```

## v14.1 — CAUSA: la salida temprana saltaba el paso [8/8]

Todos los `systemctl daemon-reload` / `enable --now` del script viven juntos en
el paso **[8/8]**, no en el paso donde se escribe cada unit file. `GNSS_ONLY=1`
hacía `exit 0` antes del paso 7, así que dejaba
`/etc/systemd/system/gps-pps-ldattach.service` y
`/etc/systemd/system/gpsd-eodav.service` escritos en disco y **sin arrancar
nunca** — y encima imprimía un mensaje de éxito.

Es el mismo patrón que veníamos persiguiendo en el GNSS: un paso que falla en
silencio y deja el síntoma tres eslabones más abajo.

Ahora la rama `GNSS_ONLY` hace su propio paso [8/8] acotado:

```bash
systemctl daemon-reload
systemctl enable --now gps-pps-ldattach.service
systemctl enable --now gpsd-eodav.service
systemctl restart chrony
```

y después **verifica**: si `gpsd-eodav.service` no quedó activo, sale con
código 1 y los comandos para ver por qué (`systemctl status`, `journalctl`), en
vez de declarar éxito.

## v14.2 — El mensaje de chrony era ambiguo

`(chrony.conf no encontrado en la ruta esperada, o ya tiene refclocks — revisar
a mano)` mezclaba dos situaciones muy distintas en una sola línea. Ahora
distingue "el archivo no existe" (con cómo encontrarlo) de "ya estaba
configurado por este script, no se toca".

---

# Cambios v15 — el Path se dibujaba estando quieto

Reportado con el GNSS ya funcionando: RViz dibuja una maraña de varios metros
sin que nada se mueva, y el frame `map` aparece a decenas de metros de
`base_link`.

**No era un error del software: es el ruido del receptor.** Un GNSS quieto no
devuelve dos veces la misma posición, y `fix_to_path` acumulaba *todos* los
fixes. Pero sí había dos cosas que arreglar, ambas en la parte DERIVADA — el
`/fix` que se graba sigue intacto, con todo su ruido, que es lo correcto para el
dataset.

## v15.1 — Filtro de movimiento del Path

Sólo se agrega un punto si hubo desplazamiento real. Tres piezas:

* **Promediado** (`smooth_window`, 10 fixes): comparar el fix crudo contra el
  último punto dibujado **no alcanza** — la nube de ruido es más grande que el
  umbral, así que en cuanto un fix cae en el borde opuesto supera la distancia
  y se dibuja igual. Medido: seguían pasando **211 de 600** puntos. Promediando
  las últimas N muestras la nube se encoge por √N y el gate recién funciona.
* **Umbral adaptativo** (`distance_sigma_k`, 2.0): el umbral sale de la
  covarianza que reporta el propio receptor —
  `max(min_distance_m, k · sigma_horizontal)`. Sin RTK sigma es de metros y el
  umbral sube solo; con RTK fijo baja a centímetros y el detalle fino se
  conserva.
* **Fallback** (`fallback_sigma_m`, 1.0): sigma a asumir cuando el receptor no
  reporta covarianza. Sin esto el filtro se caía al piso de 0,2 m y quitaba
  sólo el 9 % de los puntos.

Medido sobre 30 semillas, 600 fixes (60 s a 10 Hz) con ruido σ=1 m, quieto:

| | puntos en el Path | extensión del dibujo |
|---|---|---|
| antes | 600 | 6,0 – 7,7 m |
| ahora | 1 – 3 (mediana 2) | < 3,2 m |

Y moviéndose 60 m: 29 puntos, recorrido 58,8 m. Con RTK (σ=2 cm) moviéndose
30 m: 131 puntos, recorrido 28,8 m — o sea que el detalle fino **no** se pierde.

No queda en exactamente 1 punto: el promedio móvil sigue haciendo un random
walk lento. Lo que cambia es el orden de magnitud.

## v15.2 — El origen se fija promediando, no con el primer fix

El primer fix es sistemáticamente el peor (el receptor todavía no convergió).
Fijar el origen ahí deja el frame `map` desplazado — es la línea amarilla larga
entre `map` y `base_link` de la captura. Y como todo se mide contra el origen,
el error se arrastra toda la sesión.

Ahora se promedian los primeros `origin_settle_fixes` (20 ≈ 2 s) fixes válidos.
Test: con un primer fix a 50 m + 19 buenos, el origen queda a **2,5 m** en vez
de a 50 m.

## v15.3 — Tests de regresión (`gps_bringup/test/test_fix_to_path.py`)

8 tests con fixes sintéticos de ruido conocido, sin hardware. Verifican las dos
propiedades juntas — **quieto no dibuja** y **moviéndose sí** — porque un filtro
que borra la maraña pero también el movimiento real es fácil de escribir sin
darse cuenta. Incluye una contraprueba con el filtro apagado, para que los demás
no puedan pasar por el motivo equivocado.

Los umbrales de los asserts salen de medir 30 semillas, no al revés.

Wire a `colcon test` vía `ament_cmake_pytest` (dos líneas en el CMakeLists; si
dieran problema se borran sin afectar el build).

## Nota: esto no mejora la precisión

El filtro es de **visualización**. Si necesitás precisión real, el camino son
las correcciones **RTK** (NTRIP o base propia): el simpleRTK3B las soporta y
llevan el ruido de ~1–2 m a ~1–2 cm. Con RTK activo el umbral se adapta solo y
la trayectoria queda con todo el detalle.

---

# Cambios v16 — el adaptador USB-serial no puede entregar PPS (y nadie lo decía)

Diagnóstico en la PC principal: `ppstest /dev/pps1` daba `Connection timed out`
indefinidamente, con fix 3D confirmado (27 satélites) y el cable presente.

```
$ udevadm info -q property -n /dev/gps_pps | grep ID_MODEL
ID_MODEL=FT230X_Basic_UART
ID_MODEL_ID=6015
```

## v16.1 — CAUSA: el FT230X no tiene línea DCD

`pps_ldisc` lee el pulso **únicamente** por DCD. El FT230X es un UART básico —
TXD, RXD, RTS, CTS y unos CBUS — y **no tiene DCD ni en el chip**. El pulso no
podía llegar por más que el cable TPS estuviera bien puesto.

Lo peor del modo de falla: `/dev/ppsN` **se crea igual** (la línea de disciplina
se engancha sin problema) y `ppstest` se queda esperando para siempre. Parece un
cable suelto, y no lo es.

El repo ya tenía la información, en `um982_driver/README.md:42`:

> an FTDI **FT230X** (`ID_SERIAL_SHORT=D30HIC4U`, **distinct from the FT232R
> used by `/dev/gps_pps`**)

O sea que el proyecto usa dos adaptadores distintos — `A5069RR4` (FT232R, con
DCD, para NMEA+PPS) y `D30HIC4U` (FT230X, sin DCD, para el passthrough del
UM982) — y se estaba usando el segundo en el rol del primero.

## v16.2 — El script ahora lo detecta y aborta

`setup_ptp_sync.sh` lee `ID_MODEL` de cada adaptador, lo muestra en el listado y
**aborta** si el elegido es un chip sin DCD, explicando por qué y qué hace falta.

```
Adaptadores USB-serial detectados:
  /dev/ttyUSB0   serial=D30HIC4U    FT230X_Basic_UART     <-- SIN linea DCD: no puede entregar PPS
```

Escape deliberado para seguir sin PPS (reloj a ~10-100 ms, suficiente para
probar PTP entre sensores pero no para hora absoluta):

```bash
sudo ALLOW_NO_PPS=1 GNSS_ONLY=1 ./setup_ptp_sync.sh
```

Esto también tapa un agujero que había abierto la autodetección de v12: elegía
"el único adaptador conectado", que es correcto para el NMEA pero no verificaba
que sirviera para PPS.

## v16.3 — El diagnóstico lo dice en el paso 1

`diagnose_gnss.sh` reporta el modelo del chip y marca `[FALLA]` cuando no puede
hacer PPS, en vez de mandar a revisar cables.

## v16.4 — Documentación contradictoria corregida

`gps_bringup/README.md` seguía describiendo el esquema viejo de dos adaptadores
(*"NMEA on `ttyUSB0`, PPS on `ttyUSB1`"*) que el resto de la documentación
descarta explícitamente. Ahora describe el esquema de un solo adaptador y
advierte del requisito de DCD.

---

## v17 — PTP sobre puertos de bridge: el `Delay_Req` no llega (MEDIDO)

### El hallazgo

Las tres interfaces de cámara de la máquina de pruebas son esclavas de un
bridge Linux, **de forma deliberada** (un solo segmento L2 / una sola IP para
que el descubrimiento GigE Vision de Arena vea las tres cámaras):

    2: enp6s0: <...> master bridge0 state forwarding
    3: enp7s0: <...> master bridge0 state forwarding
    4: enp8s0: <...> master bridge0 state forwarding
    8: bridge0: <...> inet 168.254.1.1/16

`setup_ptp_sync.sh` escribía `ptp4l-sw.conf` con `network_transport UDPv4` y
lanzaba `ptp4l -i enp6s0 -i enp7s0 -i enp8s0`. Eso **no funciona sobre puertos
de bridge**, y falla de una forma que parece que anduviera.

### La medición

Reproducción con `veth` + `bridge` en un network namespace, linuxptp 4.0,
grandmaster (`serverOnly 1`) de un lado y cliente (`clientOnly 1`) del otro:

| Configuración | `path delay` | Resultado |
|---|---|---|
| Interfaz suelta, UDPv4 | 1395–1552 ns | OK |
| **Esclavo de bridge, UDPv4** (lo que hacíamos) | **0** | **ROTO** |
| El bridge mismo (`br0`), UDPv4 | no arranca | ROTO |
| **Esclavo de bridge, L2** | 1507–1696 ns | OK |

Sobre `br0` directamente, `ptp4l` ni siquiera levanta:

    interface 'br0' does not support requested timestamping mode
    failed to create a clock

### Por qué

En un puerto de bridge, `br_handle_frame` (el `rx_handler`) se queda con la
trama **antes** de que llegue a los *protocol handlers* de IP. Un socket UDP
con `SO_BINDTODEVICE` sobre el esclavo nunca ve el `Delay_Req` que manda la
cámara.

El sentido contrario sí funciona —`ptp4l` transmite directo por el netdev, sin
pasar por el bridge— y por eso el síntoma engaña: la cámara **ve** el
`Announce` y el `Sync`, loguea `new foreign master`, pasa a `UNCALIBRATED` y
reporta `master offset`. Todo parece bien. Pero el `path delay` queda en `0`
para siempre: nunca se cierra el intercambio E2E y nunca se compensa la
latencia del enlace.

Con transporte L2 el problema desaparece, porque los taps de `AF_PACKET`
corren en `__netif_receive_skb_core` **antes** del `rx_handler` del bridge.

Se descartó STP y `mcast_snooping` como causa: el mismo test con
`stp_state 0 mcast_snooping 0` sigue dando `path delay 0`.

### Cambios

`setup/setup_ptp_sync.sh`:

* Nueva variable **`CAM_PTP_TRANSPORT`** (`auto` | `L2` | `UDPv4`, default
  `auto`). En `auto` se usa L2 si las interfaces de cámara son esclavas de un
  bridge, y UDPv4 si no — o sea que el comportamiento previo se conserva
  intacto en una máquina sin bridge.
* Nueva función `bridge_master_of()`; el paso `[2/8]` ahora reporta qué
  interfaces están en un bridge.
* Aviso si **`LIDAR_IFACE`** está en un bridge: el timestamping por hardware se
  toma en la NIC, así que el PHC sigue sirviendo, pero el `Delay_Req` del
  Hesai sufre el mismo problema.
* Aviso si el bridge tiene **STP activo**: con tres equipos terminales no puede
  haber bucles, y el `forward_delay` de 15 s hace que tras cada replug el
  puerto tarde ~30 s en reenviar.
* Los comandos de verificación finales ahora incluyen `GET CURRENT_DATA_SET` /
  `GET PORT_DATA_SET` para mirar el `path delay`, y dos `tcpdump` para
  determinar qué transporte PTP habla realmente la cámara.

`ptp4l-hw.conf` (Hesai) **no cambia**: sigue en UDPv4, que es lo que pide la
web UI del Hesai (`Transport: UDP/IP`), y `enp11s0` no está en el bridge.

### Sin verificar

**Que las Triton acepten PTP sobre L2.** Está medido que L2 arregla el
problema del bridge en linuxptp; no está probado contra una cámara real. Los
dos `tcpdump` del final del script resuelven la duda en 15 segundos:

    sudo timeout 15 tcpdump -ni enp6s0 -e 'ether proto 0x88f7'          # L2
    sudo timeout 15 tcpdump -ni enp6s0 'udp port 319 or udp port 320'   # UDPv4

Si la cámara sólo habla UDPv4, entonces el bridge y el PTP son incompatibles y
hay que sacar las cámaras del bridge.

---

## v18 — Documento de arquitectura de sincronización + escala de tiempo

### `setup/ARQUITECTURA_SINCRONIZACION.md` (nuevo)

Diseño de referencia completo de la cadena de tiempo: principio de un solo
reloj central (`CLOCK_REALTIME` disciplinado por PPS, todo lo demás colgando de
ahí), por qué hacen falta dos instancias de `ptp4l`, plan de configuración por
bloques (A: GNSS→reloj, B: PTP, C: radar), tabla de precisión esperable por
eslabón, y criterios de aceptación por paso.

### `setup/setup_ptp_sync.sh` — escala de tiempo TAI/UTC

Se agregó a **las dos** instancias:

    utc_offset          0
    timeSource          0x20

`ptp4l` **siempre** anuncia escala PTP (TAI) con `currentUtcOffset`. En
linuxptp 4.0 **no existe** la opción `ptp_timescale` — verificado:

    unknown option ptp_timescale at line 5 in global section
    failed to parse configuration file

Lo que se controla es `utc_offset`, cuyo default es **37**. Como en este diseño
tanto `CLOCK_REALTIME` como el PHC están en UTC (`phc2sys -O 0`), dejar el
default habría puesto al LiDAR **37 s corrido** respecto de las cámaras. Con
`utc_offset 0`, un esclavo que calcula `UTC = PTP − offset` obtiene UTC, y uno
que ignora el offset también.

Verificado que el `ptp4l-sw.conf` generado arranca y toma el rol de grandmaster
en los tres puertos de un bridge.

### Hallazgo documentado, no corregido: timestamp del radar

`ars430_ros_publisher/src/radar_publisher_node.cpp` líneas 238 y 292:

```cpp
msg.header.stamp = now();     // reloj del nodo al parsear
```

`now()` incluye el bufferizado de libpcap y la latencia de scheduling. En la
**misma función** ya está disponible el timestamp de recepción del kernel, que
se usa para el replay offline (línea 157) pero no para el vivo:

```cpp
const int64_t ts = (int64_t)h->ts.tv_sec * 1000000 + h->ts.tv_usec;
```

Cambiar `now()` por `h->ts` mejora el timestamp del radar en dos o tres órdenes
de magnitud. No se aplicó: es un cambio de comportamiento en la fuente de datos
y corresponde decidirlo, no colarlo.

---

## v19 — `setup/diagnose_ptp.sh`

Un solo comando que recolecta todo el estado de la cadena de sincronización, en
cinco niveles con dependencia estricta (un eslabón roto se disfraza de
"sincronización mala" en el siguiente):

| Nivel | Qué |
|---|---|
| 0 | Inventario: versión desplegada, interfaces, bridge, capacidades de NIC |
| 1 | Referencia: PPS → chrony → `CLOCK_REALTIME` |
| 2 | Distribución: `ptp4l` ×2, `phc2sys`, PHC |
| 3 | Tráfico real en el cable (qué transporte PTP habla cada equipo) |
| 4 | ROS 2: `ros2 topic delay` por tópico |

    sudo ./diagnose_ptp.sh        # 0..3
    sudo ./diagnose_ptp.sh 4      # además el nivel 4 (necesita nodos ROS vivos)

Deliberadamente **sin `set -e`**: un comando que falla es un dato, no un motivo
para abortar. Todo va con `timeout` para que `ppstest`, `tcpdump` y
`ros2 topic delay` no bloqueen.

Cada bloque imprime un "CÓMO LEERLO" con los valores esperados y qué significa
cada desvío (`path delay 0`, ±37 s, delay negativo, delay creciendo lineal).

### Detecta específicamente

* Que el `/dev/pps*` elegido sea el del **adaptador serie** y no el del **PHC de
  la Intel** — los dos existen y confundirlos hace que el PPS "funcione" cuando
  en realidad no se está midiendo el GNSS.
* Que la versión desplegada sea la nueva (v17/v18), antes de creerle a nada más.
* Incoherencia entre "cámaras en un bridge" y `network_transport` distinto de
  `L2`.
* `utc_offset` ausente o distinto de 0.
* Daemons NTP en conflicto con chrony (`systemd-timesyncd`, `ntp`, …).

### Bug encontrado y corregido durante su propia prueba

La primera versión imprimía `[ OK ] enp11s0 no está en ningún bridge` en una
máquina donde `enp11s0` **no existe**: un `ip link show` vacío se lee igual que
"no está en ningún bridge". Es la misma clase de falso OK que ya había aparecido
en `diagnose_gnss.sh`. Ahora se comprueba `/sys/class/net/<iface>` primero.

Verificado en dos escenarios: máquina pelada (ningún OK falso; todo lo ausente
sale como FALLA) y un bridge real en un network namespace (detecta la
pertenencia y no marca falsos positivos sobre la interfaz suelta).

---

## v20 — Tres correcciones, dos de ellas a errores míos

### 1. BUG: `setup_ptp_sync.sh` abortaba en silencio en el paso [2/8]

Introducido por mí en v17. `bridge_master_of()` terminaba con el estado del
`[[ ... ]] && echo`, o sea **1** cuando la interfaz NO está en un bridge. Con
`set -euo pipefail`, la asignación

```bash
b=$(bridge_master_of "$LIDAR_IFACE")
```

toma ese estado y **aborta el script**. Reproducido:

```
$ bash repro.sh
antes de la linea del LiDAR
exit=1                      # "DESPUES" nunca se imprime
```

Síntoma en campo, exactamente: el paso `[2/8]` imprimía las tres líneas
`ℹ enpXsY es esclava del bridge 'bridge0'` y el script moría antes de `[3/8]`,
**sin un solo mensaje de error**, dejando los `.conf` viejos en
`/etc/linuxptp/`. O sea que `utc_offset 0` y el transporte de v17/v18 nunca se
llegaron a escribir, y todo lo que se midió después era la configuración vieja.

Corregido con un `return 0` explícito. Test de regresión incluido: el bloque
real del paso `[2/8]`, extraído del script, corriendo bajo `set -e` contra un
bridge de verdad en un network namespace, ahora llega a `[3/8]`.

### 2. Las Triton hablan UDPv4, no L2 — `auto` estaba mal

Dato de campo: se capturaron los `Delay_Req` de las tres cámaras en
`udp port 319`, con clock identity `1c:0f:af:...` (OUI de LUCID), y **cero**
paquetes en `ether proto 0x88f7`.

O sea que la lógica de v17 —"cámaras en bridge ⇒ L2"— las habría dejado **sin
grandmaster**. Con cámaras GigE Vision, UDPv4 no es opcional.

`auto` ahora usa siempre UDPv4 y, si detecta el bridge, imprime la consecuencia
en vez de intentar un arreglo que rompe otra cosa.

**Y la consecuencia es menor de lo que yo había planteado:** quedarse en UDPv4
sobre un bridge significa que el path delay no se mide y el esclavo arrastra un
sesgo igual al retardo de ida del enlace — **medido: 1400–1700 ns**. Eso está
muy por debajo del piso de ruido del timestamping por software (20–100 µs), así
que no es el limitante. Lo decisivo es si la cámara igual llega a
`PtpStatus == Slave`.

Se descartaron los arreglos por software, todos medidos:

| Intento | Resultado |
|---|---|
| `ptp4l` sobre `bridge0`, `time_stamping software` | `does not support requested timestamping mode` |
| `ptp4l` sobre `bridge0`, `time_stamping legacy` | idem |
| IP en el esclavo del bridge + UDPv4 | `path delay 0` igual |

Si las cámaras no enganchan, la única salida es sacarlas del bridge.

### 3. `meanPathDelay` en un grandmaster vale 0 SIEMPRE

Criterio de aceptación que yo había dado mal. `meanPathDelay`,
`offsetFromMaster` y `peerMeanPathDelay` son 0 en un grandmaster por
definición: no tiene master del cual medirse (`stepsRemoved 0`). Leerlos por
`pmc` desde el lado del grandmaster **no verifica nada**.

`diagnose_ptp.sh` ahora lo dice, y en su lugar propone la prueba correcta sobre
la captura del nivel 3: buscar un `Delay_Req` que venga del sensor y ver si la
PC contesta en el **puerto 320** dentro del milisegundo. Eso sí distingue un
enlace que cierra de uno que no:

```
# Hesai (enp11s0) — CIERRA
12:53:48.427152 IP 192.168.1.201.319 > 224.0.1.129.319: Delay_Req
12:53:48.427226 IP 192.168.1.100.320 > 224.0.1.129.320: <- Delay_Resp, 74 us

# Camara (enp6s0) — NO CIERRA
12:51:37.720773 IP 168.254.87.15.319 > 224.0.1.129.319: Delay_Req
12:51:38.382502 IP 10.1.1.10.319 > ...                  <- 660 ms despues, es
                                                           el Sync periodico
```

`tcpdump` ve el `Delay_Req` de la cámara porque `AF_PACKET` corre antes del
`rx_handler` del bridge; `ptp4l` no lo ve. Es la confirmación en hardware real
de lo que se había medido en la reproducción.

---

## v21 — Modo de dos adaptadores (NMEA y PPS en devices USB distintos)

### El dato que lo motiva

Con las dos puntas conectadas al mismo tiempo, el paso `[3/8]` reveló la
topología real, que no es la que el script asumía:

```
/dev/ttyUSB0   serial=D30HIC4U   FT230X_Basic_UART   <-- SIN linea DCD
/dev/ttyUSB1   serial=A5069RR4   FT232R_USB_UART
```

* `ttyUSB0` = USB nativo del simpleRTK3B → **NMEA**, y es un FT230X sin DCD.
* `ttyUSB1` = el FTDI232 externo → **PPS por DCD**, y es un FT232R que sí lo
  tiene.

Esto cierra la contradicción que se venía arrastrando: el diagnóstico anterior
veía un solo adaptador (el FT230X) y concluía "sin DCD, PPS imposible". El
FT232R estaba, pero no conectado en ese momento. `/dev/pps1 -> usbserial1`
confirma que la disciplina de línea ya está sobre el adaptador correcto.

### El problema de la topología de dos adaptadores

gpsd **no puede correlacionar** el fix de un device USB con el pulso de otro
—está documentado en la cabecera de este mismo script, con el corrimiento
espurio de ~370 ms que se midió—. Y de hecho reaparece en la salida de chrony:

```
#? PPS   0 4 0  46m  +452ms[ +477ms] +/- 156ms
```

### La solución correcta

En modo dos adaptadores el PPS **no pasa por gpsd**. Se usa el driver PPS
nativo de chrony, que existe justamente para esto:

```
refclock SHM 0 refid NMEA offset 0.2 delay 0.2 noselect
refclock PPS /dev/gps_pps_src lock NMEA refid PPS precision 1e-7 prefer
```

`lock NMEA` es lo que los une: el pulso da la **fracción** de segundo (precisa)
y el NMEA da el **segundo entero** (grosero). Sin `lock`, chrony no sabe a qué
segundo pertenece cada pulso y el refclock PPS queda inutilizable.

### Cambios

* Nueva variable **`GPS_NMEA_SERIAL`**. Vacía = modo un adaptador (sin cambios).
* Tres symlinks udev: `/dev/gps_pps` (tty del PPS), `/dev/gps_nmea` (tty del
  NMEA) y `/dev/gps_pps_src` (el `/dev/ppsN`, sobre el subsistema `pps`, para
  que chrony tenga una ruta fija).
* `gpsd-eodav.service` abre **sólo** el adaptador de NMEA en modo dos
  adaptadores.
* Validación: aborta si `GPS_NMEA_SERIAL` no existe o coincide con `GPS_SERIAL`.
* Comandos de verificación específicos del modo, incluida la corrección manual
  de `chrony.conf` si el symlink `/dev/gps_pps_src` no llegara a crearse.

Uso:

```bash
sudo GPS_SERIAL=A5069RR4 GPS_NMEA_SERIAL=D30HIC4U ./setup_ptp_sync.sh
```

### Bug de comillas encontrado al probarlo

Los comentarios del bloque nuevo llevaban backticks dentro de un heredoc **sin
comillas** (`<<EOF`), así que bash los ejecutaba como sustitución de comandos:

```
/tmp/t.sh: line 8: lock: command not found
```

El `chrony.conf` quedaba con los comentarios mutilados. Corregido y agregado un
barrido que verifica que no queden backticks dentro de ningún heredoc sin
comillas en todo el script.

### Sin verificar

La regla udev sobre el subsistema `pps` (`SUBSYSTEM=="pps", ATTRS{serial}==...`)
no se pudo probar acá: depende de que udev encuentre el atributo `serial`
subiendo la cadena de padres desde el device PPS. Si `/dev/gps_pps_src` no
aparece, el script imprime cómo resolver el `/dev/ppsN` real y corregir
`chrony.conf` a mano.

---

## v22 — CAUSA RAÍZ del PPS: la ldisc estaba sobre el adaptador equivocado

### El dato

```
/dev/gps_pps  ->  ttyUSB1        (FT232R, el que SÍ tiene DCD)   ✓
/dev/pps1     ->  'usbserial0'   (= ttyUSB0, FT230X, SIN DCD)    ✗
```

`usbserialN` corresponde a `ttyUSBN`. El symlink apuntaba al adaptador correcto,
pero **`ldattach` seguía enganchado al otro**, de una corrida anterior. Con la
disciplina de línea sobre un chip que no tiene DCD, `ppstest` iba a dar
`Connection timed out` para siempre — y no había un solo error en ningún log.

### Por qué sobrevivía

`gps-pps-ldattach.service` era `Type=oneshot` + `RemainAfterExit=yes`, y
`ldattach` **se demoniza por defecto**. Resultado: systemd marcaba la unidad
`active (exited)` sin ser dueño del proceso real. Dos consecuencias:

* `systemctl enable --now` (paso 8) **no hace nada** si la unidad ya figura
  activa → al cambiar de adaptador, la ldisc vieja seguía puesta.
* Un `restart` dejaba **dos** `ldattach` corriendo sobre ttys distintos.

### Corrección

```ini
[Service]
Type=simple
ExecStart=/usr/sbin/ldattach -d 18 /dev/gps_pps
Restart=always
```

`-d` mantiene `ldattach` en primer plano, así systemd pasa a ser el dueño real
y `restart`/`stop` funcionan de verdad. Además:

* El paso `[4/8]` ahora mata `ldattach` huérfanos (`pkill -x ldattach`) antes de
  arrancar, hace `restart` (no `enable --now`) y **verifica** que la fuente PPS
  resultante corresponda al tty al que apunta `/dev/gps_pps`. Si no coincide,
  aborta y lista las fuentes PPS presentes.
* El paso `[6/8]` ya no se limita a "ya lo tenía, no se toca": si el bloque
  existe pero el `refclock PPS` apunta a otro `/dev/ppsN`, **corrige esa línea**.
  Es justo el valor que cambia al mover la ldisc, y dejarlo viejo da `Reach 0`
  para siempre sin ningún error visible.
* `diagnose_ptp.sh` gana los dos chequeos correspondientes: desajuste
  symlink↔ldisc, y desajuste `chrony.conf`↔dispositivo PPS real.

### Nota operativa: `noselect` en NMEA

En modo dos adaptadores el NMEA va con `noselect` (sólo desambigua el segundo).
Mientras el PPS no funcione, eso deja a chrony **sin ninguna fuente
seleccionable** — `Leap status: Not synchronised` y el reloj en deriva libre
(se vio `Frequency: 508 ppm fast`, contra los ±50 ppm de un cristal normal;
conviene borrar `/var/lib/chrony/chrony.drift` una vez que el PPS enganche).

Si hace falta trabajar con el PPS todavía roto, sacar `noselect` de la línea del
NMEA deja el reloj dentro de ~100 ms en vez de a la deriva.

---

## v23 — Dos bugs del nivel 4 de `diagnose_ptp.sh`

### 1. Bajo `sudo`, ros2 no arranca

```
importlib.metadata.PackageNotFoundError: No package metadata was found for ros2cli
```

`sudo` borra **`PYTHONPATH`** por seguridad (está en su `env_delete`), así que ni
`sudo -E env "PATH=$PATH"` lo conserva y `ros2cli` no se encuentra a sí mismo.

El nivel 4 **no necesita root**: `ros2 topic list` y `ros2 topic delay` corren
como usuario normal. Sólo los niveles 0–3 (pmc, ppstest, tcpdump) lo necesitan.

Ahora el script detecta el fallo, dice exactamente por qué y da el comando:

```
./diagnose_ptp.sh 4        # SIN sudo
```

### 2. Los nombres de tópico estaban hardcodeados y no coincidían

La lista fija daba `(no publicado)` para las tres cámaras y para el radar,
cuando en realidad estaban publicando:

| Se buscaba | Se publica |
|---|---|
| `/camera_enp6s0/image_raw` | `/arena_camera_node/enp6s0/image` |
| `/radar/packets` | `/filtered_radar_packet_1` |

Ahora se descubren de `ros2 topic list` por patrón, uno por familia de sensor.
Verificado contra la lista real de la máquina: elige los siete correctos
(`/lidar_points`, `/lidar_imu`, las tres `/arena_camera_node/<iface>/image`,
`/fix`, `/filtered_radar_packet_1`).

---

## v24 — El sistema queda sincronizado con o sin PPS

### Resultado de campo: las cámaras SÍ están sincronizadas a través del bridge

La pregunta que quedaba abierta desde v17 tiene respuesta, y es la buena:

```
14:13:50.721505  IP 168.254.198.183.319 > 224.0.1.129.319: Delay_Req
                 clock identity: 0x1c0faffffe2ad2eb
                 originTimeStamp: 1785784430.719700304
```

`originTimeStamp` = **19:13:50.7197 UTC**. El paquete se capturó a
**19:13:50.7215 UTC**. Diferencia: **1,8 ms**, casi todo tránsito. La cámara
conoce la hora real, o sea que su reloj PTP está enganchado.

Confirmado por `ros2 topic delay` sobre las tres:

| Tópico | delay medio | std dev |
|---|---|---|
| `/arena_camera_node/enp6s0/image` | 0,118 s | 0,00051 s |
| `/arena_camera_node/enp7s0/image` | 0,118 s | 0,00070 s |
| `/arena_camera_node/enp8s0/image` | 0,119 s | 0,00081 s |

Las tres coinciden dentro de **~1 ms** y el desvío no crece. Si el PTP no
estuviera enganchado, el timestamp saldría del contador interno de la cámara y
el delay sería de horas. **El bridge se queda.**

### Lo que faltaba: el reloj del host

`/fix` da delay **negativo** (−0,26 a −0,42 s). Eso significa que el timestamp
del GNSS (que es UTC correcto, `use_gps_time: True`) está **adelantado** al
reloj del host. O sea: el GNSS tiene razón y **el reloj de la PC está atrasado**.

Causa: `chrony` sin ninguna fuente seleccionable. El PPS no pulsa y el NMEA
estaba con `noselect`.

### Cambios

**1. El paso `[4/8]` ahora prueba si el PPS realmente pulsa** (`ppstest` con
timeout de 4 s) y guarda el resultado. Si no hay flancos, lo dice y explica que
la cadena de software está bien y lo que falta es físico (cable en DCD, no en
DTR; TIMEPULSE activo; GND compartido).

**2. El paso `[6/8]` adapta la config de chrony a ese resultado:**

| PPS | NMEA | Precisión |
|---|---|---|
| pulsa | `noselect` (sólo desambigua el segundo) | ~1 µs |
| no pulsa | **seleccionable** | ~100 ms |

Sin esto, `noselect` + PPS muerto deja a chrony **sin ninguna fuente** y el
reloj en deriva libre — se midió `Frequency: 508 ppm fast`, contra los ±50 ppm
de un cristal normal. 100 ms es mucho peor que 1 µs, pero es infinitamente
mejor que nada, y **la coherencia entre sensores se mantiene igual**, que es lo
que importa para fusión.

El ajuste se aplica también sobre un bloque de chrony ya existente, en las dos
direcciones.

**3. Validación de `chrony.conf` antes de reiniciar**, con `chronyd -Q`. Un
refclock apuntando a un device inexistente hacía que el servicio no arrancara
y el reloj quedara sin disciplinar — peor que antes de correr el script.

**4. Quitado un pie de bala de la ayuda impresa.** El script sugería:

```
sudo sed -i 's#refclock PPS [^ ]*#refclock PPS /dev/ppsN#' /etc/chrony/chrony.conf
```

con `/dev/ppsN` como marcador. Copiado y pegado literalmente, escribe un device
inexistente y **chrony deja de arrancar** — pasó exactamente eso en campo. Ahora
imprime el device concreto que el propio script dejó configurado.

---

## v25 — El script se colgaba en la validación de chrony (bandera equivocada)

La validación agregada en v24 usaba `chronyd -Q`. Mal:

```
  -p    Print configuration and exit      <- parsea y sale al instante
  -Q    Log offset and exit               <- ARRANCA y espera una medicion
  -q    Set clock and exit                <- idem, ademas toca el reloj
```

`-Q` no valida nada: levanta chronyd y se queda esperando que alguna fuente
conteste. Con el NMEA en `poll 16` y los servidores de red inalcanzables, eso no
pasa nunca — **el script quedaba colgado en el paso `[6/8]` indefinidamente.**

Corregido a `-p`, con un `timeout 15` de refuerzo. Verificado con el binario
real de chrony 4.5: sale en **11 ms**, `rc=0` con config válida y `rc=1` con un
directiva inválida, imprimiendo el error.

Además, `-p` valida **sintaxis**, no que los devices existan — un
`refclock PPS /dev/ppsN` inexistente pasa el parseo y después hace fallar el
arranque del servicio. Se agregó ese chequeo aparte, que lista los devices PPS
presentes cuando falla.

---

## v26 — Las tres mejoras pendientes

### 1. Timestamp del radar: `now()` → timestamp de recepción del kernel

`ars430_ros_publisher/src/radar_publisher_node.cpp`. El ARS430 no habla PTP, así
que la única palanca de sincronización es **de dónde** sale el timestamp.

Antes, las dos rutas de parseo (`parsePacket` y `parseObjectList`) estampaban con
`now()`: el reloj del nodo en el momento de **parsear**, que incluye el
bufferizado de libpcap (hasta 100 ms, el timeout de `pcap_open_live`) y la
latencia de scheduling del hilo de captura.

Ahora usan `h->ts`, el instante en que el **driver de red** recibió la trama —
del orden de microsegundos contra `CLOCK_REALTIME`, el mismo reloj que
disciplina chrony y del que cuelgan cámaras y LiDAR vía PTP.

Implementación: `handleFrame()` guarda `frame_stamp_` (es donde `h` está
disponible), y un helper `frameStamp()` decide la fuente. Las dos rutas de
parseo se llaman sincrónicamente desde `handleFrame`, así que siempre
corresponde a la trama en curso.

Parámetro nuevo **`use_kernel_timestamp`**, con default **dependiente del modo**:

| Modo | Default | Por qué |
|---|---|---|
| vivo (`pcap_file` vacío) | `true` | es el punto del cambio |
| replay de pcap | `false` | los timestamps grabados están en el pasado; estampar con ellos rompe la visualización en vivo (RViz descarta lo que quedó fuera de la ventana de TF) |

O sea que **el flujo offline existente no cambia de comportamiento**. Para
reprocesar un pcap conservando su línea de tiempo original, ponerlo en `true`.

### 2. STP del bridge, detrás de un flag

`FIX_BRIDGE_STP=1` en `setup_ptp_sync.sh`. Con tres equipos terminales no puede
haber bucles, y el `forward_delay` hace que tras cada replug el puerto pase
~30 s en LISTENING/LEARNING: las cámaras "desaparecen" ese rato.

Aplica en dos capas: sysfs (efecto inmediato) y `nmcli con mod ... bridge.stp no`
(persistente). Si el bridge no lo maneja NetworkManager, avisa explícitamente
que el cambio **no** sobrevive a un reinicio en vez de dejarlo pasar. Verifica el
resultado leyendo `stp_state` de vuelta.

Va detrás de un flag y no por defecto porque toca la configuración de red del
host.

### 3. `eod_av_tools` — nodo `sync_check` (paquete nuevo)

Mide el desfase real de `header.stamp` de cada tópico.

**Por qué no alcanzaba `ros2 topic delay`:** es Python y usa cola RELIABLE.
Sobre `/lidar_points` (5 MB por mensaje a 10 Hz = 50 MB/s) no da abasto y lo que
reporta como *delay* es su propio atraso — se lo vio subir de 3,98 s a 4,42 s en
12 s, o sea 78000 ppm, imposible para un reloj.

`sync_check` evita las dos causas: C++ (sin GIL) y **BEST_EFFORT con depth 1**,
así que si no llega a procesar **descarta** en vez de encolar. Un mensaje
perdido no sesga la medición; uno encolado sí.

**Lee el timestamp sin conocer el tipo:** `GenericSubscription` + los 12 primeros
bytes del CDR. Todo mensaje cuyo primer campo sea un `std_msgs/Header` los tiene
en un lugar fijo, así que funciona igual con `PointCloud2`, `Image`, `Imu`,
`NavSatFix` y con los mensajes propios del radar, sin depender de sus paquetes
ni recompilar al agregar uno. Los tópicos que no empiezan con `Header` se
detectan solos (el timestamp no da plausible) y se cuentan aparte.

**Reporta** `n`, media, min, max, **desvío estándar** (la estabilidad, que es lo
que dice si el reloj está enganchado) y **deriva en ppm** por mínimos cuadrados
(la que distingue "enganchado" de "corriendo con su propio cristal").

Wireado en `all_sensors.launch.py` como `sync_check:=true` (default `false`),
corriendo en **los dos modos** igual que `gnss_watchdog`: no publica ningún
tópico, así que no ensucia el bag.

```bash
ros2 launch eod_av_launch all_sensors.launch.py mode:=record sync_check:=true
# o suelto:
ros2 run eod_av_tools sync_check
```

#### Verificación

La lógica vive en `include/eod_av_tools/sync_stats.hpp`, sin dependencias de
ROS, justamente para poder testearla sin middleware. `test/test_sync_stats.cpp`
cubre 9 casos y **pasa**, compilado con `-Wall -Wextra -Wpedantic` sin warnings
y ejecutado por `ctest`:

```
1) CDR little-endian ................ OK
2) CDR big-endian ................... OK
3) buffer <12 B rechazado ........... OK
4) topico sin Header rechazado ...... OK
5) nanosec >= 1e9 rechazado ......... OK
6) enganchado: media=118.00 ms sd=284.4 us deriva=0.3 ppm
7) libre 50 ppm: deriva medida = 50.00 ppm
8) n<3 no inventa deriva ............ OK
9) fmt: 4.420 s | 118.00 ms | 1.8 us | -262.00 ms
```

El caso 7 es el que importa: la regresión recupera **50,00 ppm** de una rampa
sintética de 50 ppm exactos.

#### Sin verificar

**Nada de esto se compiló contra ROS 2** — no hay un entorno ROS donde lo
preparé. Lo que sí está verificado:

* `sync_stats.hpp` y su test compilan y pasan con g++ 13 real.
* El bloque `BUILD_TESTING` del `CMakeLists.txt` configura, compila y corre bajo
  `cmake` + `ctest` reales.
* Sintaxis de todos los `.launch.py` (`ast.parse`) y de todos los `.sh`
  (`bash -n`).
* `package.xml` bien formado.

Lo que **no** está verificado y hay que mirar en el primer `colcon build`: que
el nodo compile contra `rclcpp` (la firma de `create_generic_subscription` y el
uso de `GenericSubscription`), y el cambio del radar contra `rclcpp::Time`.

---

## v27 — Un nodo de diagnóstico opcional no puede tumbar el bringup

### El fallo

```
[ERROR] [launch]: Caught exception in launch: "package 'eod_av_tools' not found"
[INFO] [start-1]: sending signal 'SIGINT' to process[start-1]
...
```

Con `sync_check:=true` y `eod_av_tools` sin compilar, launch levanta
`PackageNotFoundError` **al construir la descripción** y se cae **todo** el
bringup: las tres cámaras, el LiDAR, el radar y el GNSS reciben SIGINT por culpa
de un nodo de diagnóstico opcional.

Es un error de diseño mío en v26: puse el `Node` con `IfCondition`, pero la
condición se evalúa *después* de que launch ya resolvió el paquete.

### La corrección

La disponibilidad se resuelve dentro del `OpaqueFunction`, donde se puede
capturar la excepción:

```python
if context.perform_substitution(
        LaunchConfiguration('sync_check')).lower() in ('true', '1'):
    try:
        get_package_share_directory('eod_av_tools')
        actions.append(Node(...))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='... sigo SIN el ...'))
```

Los sensores no dependen de `sync_check`, así que la ausencia del paquete pasa a
ser un aviso, no un aborto. Verificado en los tres casos:

| `sync_check` | paquete | resultado |
|---|---|---|
| `true` | **no** | sensores + aviso con el comando para compilarlo |
| `true` | sí | sensores + `sync_check` |
| `false` | no | sensores, sin ruido |

Misma clase de defensa que `_require_gpsd_client()` en `gps.launch.py`.

---

## v28 — `cap_net_raw` para el radar, como paso del setup

El script pasa de 8 a **9 pasos**. El nuevo `[9/9]` le da `cap_net_raw` al
binario del radar, que es lo que necesita `pcap_open_live()` para capturar sobre
`enp9s0`. Sin eso el nodo **arranca igual y no publica una sola detección** — la
alternativa sería correr el nodo como root, que es peor: todo el árbol de launch
quedaría con privilegios.

### `ros2 pkg prefix` no sirve acá

El comando suelto era:

```bash
sudo setcap 'cap_net_raw=pe' "$(ros2 pkg prefix ars430_ros_publisher)/lib/ars430_ros_publisher/radar_publisher"
```

Dentro del script eso **no funciona**: corre bajo `sudo`, y `sudo` borra
`PYTHONPATH` (está en su `env_delete`), así que `ros2cli` ni arranca. Es
exactamente el mismo problema que rompía el nivel 4 de `diagnose_ptp.sh`.

Por eso el binario se busca por filesystem, en dos intentos:

1. Subir desde el directorio del propio script hasta encontrar un
   `install/ars430_ros_publisher/lib/...`. Funciona **sin ninguna variable de
   entorno**, que es el caso real bajo `sudo`.
2. Recorrer `AMENT_PREFIX_PATH`, por si se invocó con `sudo -E`.

Override manual: `sudo RADAR_BIN=<ruta> ./setup_ptp_sync.sh`.

Los tres caminos verificados contra un workspace sintético: lo encuentra por
filesystem, no se cuelga cuando no existe, y respeta `RADAR_BIN`.

### La capacidad se pierde en cada build

`colcon build` reemplaza el binario y la capacidad se va con él. El script lo
avisa, y **`diagnose_ptp.sh` nivel 0 lo verifica** — es el tipo de cosa que se
cae sin que nadie lo note, porque el nodo arranca igual. Los tres estados
probados:

```
[FALLA] el binario del radar NO tiene cap_net_raw: pcap_open_live va a fallar
[ OK ]  .../radar_publisher cap_net_raw=ep
[ ? ]   no encontre el binario del radar (¿workspace sin compilar?)
```

Si `setcap` falla, el mensaje apunta a la causa habitual: no se pueden poner
capabilities en overlayfs, NFS ni en un montaje `nosuid`.

`libcap2-bin` se agregó al `apt install` del paso `[1/9]`.

### Warning de compilación corregido

```
sync_check_node.cpp:42:1: warning: multi-line comment [-Wcomment]
```

Una barra de continuación al final de un comentario `//` en el bloque de uso.
gcc avisa porque la línea siguiente queda comentada sin que se vea.

---

## v29 — Corrección de la guía del propio `sync_check`

### La línea que estaba mal

El nodo imprimía:

> `Comparar sensores ENTRE SI: la diferencia de medias es el desfase.`

**Falso.** `media = now − header.stamp` es la suma de dos cosas: el offset de
reloj **y** la latencia real del pipeline. Restar dos medias no las separa —
arrastra la diferencia de latencia, que entre una cámara (exposición +
transferencia de 3 MB) y un radar (un paquete UDP) es de decenas de ms aunque
los relojes estén perfectos.

Se ve en los datos reales: cámaras 114,9 ms contra radar 47,3 ms. Esa diferencia
de 67,6 ms es casi toda latencia de pipeline, no desfase de reloj.

Ahora el nodo lo dice explícitamente y remite al evento físico común, que es lo
único que los separa.

### Distinguir "reloj libre" de "nodo atrasado"

Las dos cosas dan `deriva` grande. Se agregó el criterio que las separa, porque
apareció en campo con `/lidar_points`: mirar **`min`**.

* `min` estable y sólo media/max crecen → el nodo acumula backlog. Los
  timestamps son correctos; la entrega es tardía.
* `min` también crece → el reloj del sensor se separa de verdad.

### Tópicos de RViz fuera de la lista

`/clicked_point`, `/goal_pose` e `/initialpose` llevan `Header` pero no son
sensores: aparecían con `n=0` ensuciando la tabla.

---

## v30 — El PPS funciona; el falso negativo que dejaba a chrony sin sincronizar

### El dato

```
source 0 - assert 1785853852.000008344, sequence: 110
source 0 - assert 1785853853.000009838, sequence: 111
source 0 - assert 1785853854.000011101, sequence: 112
```

Parte fraccionaria: **8 µs** del segundo exacto. Es un PPS real disciplinado por
GNSS. Y chrony lo confirma:

```
#x PPS   0 4 177 11  +8673ns[+8673ns] +/- 2566ns
```

**±2,5 µs de error estimado.** El cable quedó bien.

### El problema: los dos refclocks se contradicen

```
#x NMEA   +177ms[ +177ms] +/-  119ms
#x PPS   +8673ns[+8673ns] +/- 2566ns
```

Las dos con **`x`** = *may be in error*. chrony marca así a las fuentes cuyos
intervalos de confianza **no se solapan**: NMEA dice `[58, 296] ms`, PPS dice
`[6,1, 11,3] µs`. Como no puede decidir cuál miente, **no selecciona ninguna** y
el reloj queda sin disciplinar (`Leap status: Not synchronised`).

Es exactamente lo que el diseño evita poniendo `noselect` en el NMEA cuando hay
PPS: el pulso da la fracción de segundo y el NMEA sólo desambigua el segundo,
sin votar.

### La causa: falso negativo en la detección de PPS

El paso `[4/9]` probaba con `timeout 4 ppstest` **dos segundos después** de
reenganchar la ldisc. Dio "NO llegan flancos" — y un minuto más tarde el mismo
`ppstest` a mano mostraba los pulsos. Con la ldisc recién reenganchada el
dispositivo tarda en asentarse.

Ese falso negativo no es cosmético: hace que el paso `[6/9]` deje el NMEA
**seleccionable**, y con un PPS bueno presente los dos se contradicen y chrony
no sincroniza nada.

**Corregido:** ventana de 8 s y un reintento con 3 s de espera entre medio.

### `NMEA_OFFSET` configurable

El residuo de +177 ms del NMEA sale de que el retardo real del sentence es
`0,2 + 0,177 ≈ 0,38 s`, y el script compensaba 0,2 fijo. Ahora es una variable
con instrucciones de calibración:

```bash
sudo NMEA_OFFSET=0.38 ./setup_ptp_sync.sh
```

Con PPS vivo es cosmético (el NMEA va con `noselect` y para desambiguar el
segundo sobra medio segundo de margen). Importa cuando **no** hay PPS: ahí el
NMEA es la única fuente y su residuo es el error del reloj.

---

## v31 — La causa del falso negativo: buffering de stdout

### Reproducido

```
=== por un pipe SIN stdbuf (lo que hacia el script) ===
Terminated
grep NO encontro
=== con stdbuf -oL ===
grep ENCONTRO assert
```

`timeout 8 ppstest "$dev" | grep -q assert`: con stdout hacia un **pipe**, libc
pasa de line-buffering a **buffer de bloque de 4 KB**. Cada línea de `ppstest`
son ~80 bytes, así que hacen falta ~50 pulsos —**50 segundos**— antes de que el
buffer se vacíe y `grep` vea algo. El `timeout` mataba el proceso mucho antes.

Interactivamente stdout es un tty → line-buffered → se ve enseguida. Por eso
`ppstest` a mano funcionaba y dentro del script no. Subir la ventana de 4 a 8 s
(v30) no podía arreglarlo: el problema no era el tiempo.

### Consecuencia real

El paso `[6/9]` dejaba el NMEA **seleccionable**, y con el PPS bueno presente
los dos refclocks se contradicen:

```
#x NMEA   +202ms[ +202ms] +/-  119ms
#x PPS   -3927ns[-3927ns] +/- 4228ns
```

Los intervalos no se solapan (`[83, 321] ms` contra `[-8,2, +0,3] µs`), chrony
marca las dos `x` = *may be in error*, no selecciona ninguna, y el reloj queda
**sin disciplinar**.

### Corrección: detección por sysfs

`/sys/class/pps/ppsN/assert` trae `<sec>.<nsec>#<seq>` y se actualiza en cada
flanco. Se lee dos veces separadas 3 s y se compara. Exacto, instantáneo, sin
dependencias y sin buffering de por medio. Probados los tres estados: pulsando,
muerto, y sin dispositivo.

### `NMEA_OFFSET` no se aplicaba sobre un bloque existente

Correr con `NMEA_OFFSET=0.38` sobre un `chrony.conf` que ya tenía el bloque de
este script **no hacía nada**: el camino "ya lo tenía" sólo corregía el device
del PPS y el `noselect`. Ahora también corrige el `offset`.

### Confirmado sano en esta corrida

`phc_ctl enp11s0 cmp` volvió a **−285 ns** (venía de +3688 s tras el salto de
reloj) y `phc2sys` reporta `s2 freq +1990` con offsets de decenas de ns: la
cadena `CLOCK_REALTIME → PHC → Hesai` reconvergió sola, como correspondía.

---

## v32 — El propio setup podía matar un PPS que funcionaba

### El síntoma

PPS funcionando (`sequence: 278, 279, 280`), se corre `setup_ptp_sync.sh`, PPS
muerto con `assert` en `0.000000000#0` — cero pulsos desde que se creó el
dispositivo.

### La causa

El paso `[4/9]` hacía **incondicionalmente**, en cada corrida:

```bash
pkill -x ldattach
systemctl restart gps-pps-ldattach.service
```

incluso cuando la ldisc ya estaba sobre el tty correcto y pulsando bien. Eso
destruye un `/dev/ppsN` que funciona y crea uno nuevo con el contador en cero.
Si el reenganche no prende —el FTDI puede quedar sin reportar cambios de modem
status tras cerrar y reabrir el tty— **un PPS que andaba deja de andar por
correr el setup**.

### Corrección

Ahora el paso `[4/9]` primero comprueba si la ldisc **ya** está sobre el tty
correcto **y** el dispositivo pulsa. Si es así, no lo toca:

```
✓ la ldisc YA esta sobre ttyUSB1 y /dev/pps1 pulsa -- no la toco
```

Sólo cuando falta o no pulsa hace el `pkill` + `restart`. El script pasa a ser
idempotente en ese paso, que es lo que tenía que haber sido desde el principio.

### Test de flancos de DCD en `diagnose_ptp.sh`

`ppstest` mide el PPS **después** de `pps_ldisc`: si da timeout no distingue
entre "no llega señal por el cable" y "la ldisc no engancha los flancos". El
nivel 1 ahora lee el bit DCD directo del chip con `TIOCMGET`, sin pasar por la
ldisc:

| flancos en 10 s | significa |
|---|---|
| ~20 | el cable y el TIMEPULSE están **bien**; el problema es la ldisc |
| 0 | el pulso no llega al chip: cable en el pad equivocado (DTR está al lado de DCD), TIMEPULSE apagado, o GND sin compartir. Ningún software lo arregla |

Es la prueba que separa hardware de software en un solo comando.

---

## v33 — Sincronización lograda; dos bugs del diagnóstico

### El objetivo, cumplido

```
Reference ID : 50505300 (PPS)
Stratum      : 1
System time  : 0.000001492 seconds fast of NTP time
RMS offset   : 0.000004223 seconds
Skew         : 0.109 ppm
Root dispersion : 0.000024251 seconds
Leap status  : Normal
#* PPS   +1889ns[+2633ns] +/- 4082ns
#? NMEA   -26ms[  -26ms] +/-  122ms
```

**El reloj del host queda a 4,2 µs del GNSS** — contra los 19,7 ms que daba con
NMEA solo. Casi 4700× mejor. El `#*` en PPS y el `#?` en NMEA son exactamente la
configuración que buscaba el diseño: el pulso pone la fracción de segundo, el
NMEA sólo desambigua cuál segundo es.

### BUG: el diagnóstico elegía el `/dev/ppsN` equivocado

```
/dev/pps1 -> 'usbserial1'  = adaptador serie, ESTE es el del GNSS
/dev/pps2 -> 'usbserial0'  = adaptador serie, ESTE es el del GNSS
[ OK ] PPS del GNSS = /dev/pps2          <- MAL
[FALLA] DESAJUSTE: /dev/gps_pps -> ttyUSB1 pero la fuente se llama 'usbserial0'
[FALLA] chrony usa /dev/pps1 pero el PPS del GNSS es /dev/pps2
```

**Las dos FALLA eran falsas.** El bucle asignaba `GPS_PPS` en cada fuente serie,
así que ganaba **la última**: `/dev/pps2` = `usbserial0` = el FT230X sin DCD,
sobrante de un `ldattach` viejo. La correcta es la que corresponde al tty al que
apunta `/dev/gps_pps`, que es lo que ya hacía bien el `setup`.

Corregido y reproducido contra el caso exacto de campo (pps0=ptp0,
pps1=usbserial1, pps2=usbserial0 → elige pps1).

### `publish_rate` del GNSS ahora es un argumento de launch

Estaba fijo en 10. Con el receptor a 1 Hz eso republica **el mismo fix diez
veces con el mismo `header.stamp`**, y el retardo medido rampa de ~0 a ~1 s en
bucle: media 424 ms, sd 130 ms, min 131 ms, max 731 ms. Todo artefacto — se ve
comparando con `/extended_fix`, que da 390 µs de media.

```bash
ros2 launch eod_av_launch all_sensors.launch.py publish_rate:=1
```

El default sigue en 10 para no cambiar comportamiento. La alternativa mejor para
un dataset de vehículo es subir el receptor a 10 Hz: son 10 posiciones
**reales** por segundo en vez de una repetida diez veces.

---

## v34 — La carrera que rompía la ldisc al reengancharla

### El dato que lo cierra

El test de flancos de DCD del nivel 1 dio **20 flancos en 10 s** — el cable, el
TIMEPULSE y el GND están perfectos. Y al mismo tiempo:

```
ppstest /dev/pps1 -> Connection timed out
/sys/class/pps/pps1/assert -> '0.000000000#0'
```

Hardware bien, `pps_ldisc` sin capturar. Eso descarta todo lo físico y localiza
el problema en el reenganche.

### La causa

La unidad `gps-pps-ldattach.service` tiene `Restart=always` / `RestartSec=2`.
El paso `[4/9]` hacía:

```bash
pkill -x ldattach          # mata TAMBIEN el proceso gestionado por systemd
systemctl restart gps-pps-ldattach.service
```

El `pkill` mata el proceso que systemd administra → systemd programa un
auto-restart a los 2 s → el `systemctl restart` inmediato arranca otro. Quedan
**dos ciclos de attach/detach sobre el mismo tty en un par de segundos**, y el
resultado es una ldisc puesta pero con la interrupción de DCD sin armar:
`/dev/ppsN` existe y su contador se queda en `0.000000000#0` para siempre.

### Corrección

Orden correcto — `stop` **primero**, que cancela la intención de auto-restart, y
recién ahí el `pkill` toca únicamente huérfanos de corridas viejas:

```bash
systemctl stop gps-pps-ldattach.service
pkill -x ldattach || true
sleep 1
systemctl start gps-pps-ldattach.service
```

Más un **reintento acotado**: si tras el primer attach el dispositivo no captura
flancos, se hace un ciclo limpio de `stop` + `start` y se vuelve a comprobar. Una
sola vez.

Se conserva el guard de v32 (no tocar la ldisc si ya está bien puesta y
pulsando), que es lo que protege un PPS que funciona de morir por correr el
setup.

---

## v35 — PPS confirmado a 3,1 µs; modo de paquetes crudos para el LiDAR

### El PPS quedó, y el guard hizo su trabajo

```
[4/9] ✓ la ldisc YA esta sobre ttyUSB1 y /dev/pps1 pulsa -- no la toco
[6/9] ✓ NMEA -> noselect (hay PPS, el NMEA solo desambigua el segundo)

Reference ID : 50505300 (PPS)
RMS offset   : 0.000003131 seconds
Leap status  : Normal
```

**3,1 µs.** Y lo importante: el script **no tocó** la ldisc, así que correr el
setup ya no mata un PPS que funciona. Es exactamente para lo que estaba el guard
de v32.

Con el reloj disciplinado por PPS, las cámaras mejoraron otro escalón:

| tópico | media | sd | deriva |
|---|---|---|---|
| `enp6s0` | 114,99 ms | **331 µs** | **−0,0 ppm** |
| `enp7s0` | 116,35 ms | 1,10 ms | 2,0 ppm |
| `/extended_fix` | 228 µs | **54,7 µs** | −0,5 ppm |

### BUG: `publish_rate` no se propagaba

Se agregó el argumento a `gps.launch.py` y a `gnss.launch.py` (v33), pero
**`all_sensors.launch.py` nunca se lo pasaba** al include del GNSS. O sea que
`publish_rate:=1` no llegaba a ningún lado y `/fix` seguía con la rampa de
205→705 ms. Corregido.

### `lidar_config`: modo de paquetes crudos

`/lidar_points` sigue saliendo con 3–4 s de atraso y creciendo. Ya está
establecido que **no** es error de reloj (`use_timestamp_type: 0` = hora del
barrido) sino backlog: el driver parsea con 4 hilos sin GPU y no da abasto,
mientras el buffer UDP de 64 MB acumula.

`lidar.launch.py` ahora acepta qué YAML usar, y se agregó
`HesaiLidar_ROS_2.0/config/config_raw_packets.yaml`:

```yaml
send_packet_ros: true         # publica /lidar_packets (UDP crudo)
send_point_cloud_ros: false   # NO arma la nube en vivo
send_imu_ros: true
```

```bash
ros2 launch eod_av_launch all_sensors.launch.py \
     mode:=record lidar_config:=config_raw_packets.yaml
```

El driver pasa a reenviar lo que recibe sin parsear. La nube se reconstruye
offline con `source_type: 3` (packet rosbag) y `ros_recv_packet_topic:
/lidar_packets`.

Para un dataset es además lo correcto: se guarda el dato tal como salió del
sensor, sin transformación intermedia. El `config` entero se instala en
`share/`, así que el archivo nuevo se toma sin tocar el CMakeLists.

---

## v36 — El backlog del LiDAR no era backlog

### La medición que lo descarta

Con `lidar_config:=config_raw_packets.yaml` el driver **no parsea nada**: sólo
reenvía los paquetes UDP. Y el atraso no cambió:

```
/lidar_packets   media 3,177 s -> 3,858 s
                 min   2,273 s -> 0,910 s
                 max   4,378 s -> 6,479 s
```

La hipótesis del parseo queda descartada. Y la del backlog también, por
aritmética:

```
tasa: 245/25 s = 9,8 Hz   y   543/55 s = 9,9 Hz    -> entra = sale
```

**Una cola que crece exige salida < entrada.** Acá la tasa es la nominal del
LiDAR mientras el retardo oscila entre 0,9 y 6,5 s. No hay cola: **son los
timestamps los que se mueven varios segundos**.

Eso apunta al reloj del propio Hesai, no al driver ni a la CPU.

### `raw_topics`: volcado hexadecimal para leer `/lidar_ptp`

El único lugar donde el LiDAR reporta si su PTP está enganchado es
`/lidar_ptp`, y no se puede leer:

```
ros2 topic echo /lidar_ptp  ->  The message type 'hesai_ros_driver/msg/Ptp' is invalid
```

(faltan los bindings de Python del driver). Además `Ptp.msg` no empieza con
`Header`, así que `sync_check` lo descartaba.

Nuevo parámetro `raw_topics`: en vez de medirlos, vuelca los primeros 64 bytes
en hexadecimal. Funciona con cualquier tipo, sin bindings y sin conocer el
esquema.

```bash
ros2 run eod_av_tools sync_check --ros-args \
  -p raw_topics:="['/lidar_ptp']" -p report_period:=20.0
```

`Ptp.msg` es:

```
uint8  ptp_lock_offset
uint32 ptp_status_size
uint8[] ptp_status
```

que en CDR queda: `[0..3]` encapsulación, `[4]` `ptp_lock_offset`, `[8..11]`
`ptp_status_size`, `[12..15]` largo del array, `[16...]` los bytes de estado.
Con eso se decodifica a mano.

---

## v37 — Por qué el Hesai se quedaba en "Free Run"

### El diagnóstico

Web UI del LiDAR:

```
GPS    Unlock
NMEA   Unlock
PTP    Free Run          <-- no engancha
```

con la configuración **correcta**: `Clock Source: PTP`, `Profile: 1588v2`,
`Transport: UDP/IP`, `Domain: 0`. Y el intercambio E2E cerrando (su `Delay_Req`
contestado en 36 µs).

Volcado de `/lidar_ptp` decodificado con `raw_topics`:

```
00 01 00 00   encapsulacion CDR little endian
14 ...        ptp_lock_offset = 20   <- el umbral configurado, no una medicion
10 00 00 00   ptp_status_size = 16
10 00 00 00   largo del array = 16
ff ff ff fe bd c1 3f 24 00 00 00 08 00 00 03 c1
```

Eso confirma que el LiDAR **no** está reportando un offset medido: está en free
run.

### Dos causas, las dos de una línea

**1. `clockClass 248`.** Es el default de linuxptp y en IEEE-1588 significa
literalmente *"reloj libre, no trazable a una referencia primaria"*. Un esclavo
que busca una fuente buena puede negarse a engancharse a eso — y el mensaje
`gm.ClockClass 248` estaba en todas las salidas de `pmc` desde el principio.

Acá el grandmaster **sí** es trazable: chrony lo disciplina con el PPS del GNSS
(4,3 µs de RMS medidos) y `phc2sys` lo pasa al PHC. `clockClass 6` =
*sincronizado a una referencia primaria*, que es la verdad.

**2. `logSyncInterval`.** El Hesai está configurado con `1` (un Sync cada 2 s) y
linuxptp manda con `0` (uno por segundo). Varias implementaciones exigen que
coincidan.

### Cambio

Tres variables nuevas, aplicadas **sólo** a `ptp4l-hw.conf` (el Hesai). La
instancia de las cámaras se deja como está: engancha bien y no hay motivo para
arriesgar lo único que ya funciona.

```bash
PTP_CLOCK_CLASS="${PTP_CLOCK_CLASS:-6}"
PTP_LOG_SYNC_INTERVAL="${PTP_LOG_SYNC_INTERVAL:-1}"
PTP_LOG_ANNOUNCE_INTERVAL="${PTP_LOG_ANNOUNCE_INTERVAL:-1}"
```

Verificado con ptp4l real: anuncia `gm.ClockClass 6`, `logSyncInterval 1`,
`logAnnounceInterval 1`.

**Salvedad honesta sobre `clockClass 6`:** si el PPS se cae, chrony pasa a NMEA
y el 6 deja de ser cierto — linuxptp no lo degrada solo. Para volver al
comportamiento conservador: `PTP_CLOCK_CLASS=248`.

### Sin verificar

Que esto haga enganchar al Hesai. Son las dos causas más probables y ambas están
mal hoy, pero no tengo el LiDAR. Si con las dos sigue en Free Run, hay que
bisecar (`PTP_LOG_SYNC_INTERVAL=0` deja sólo el cambio de `clockClass`).

---

## v38 — Los cambios de configuración de ptp4l nunca se aplicaban

### El síntoma

Se agregó `clockClass 6` al conf, el script terminó sin errores, y:

```
$ sudo pmc -u -b 0 -s /var/run/ptp4l-hw 'GET PARENT_DATA_SET' | grep ClockClass
		gm.ClockClass                         248
```

El valor del conf **anterior**.

### La causa

El paso `[7/9]` reescribe `/etc/linuxptp/ptp4l-hw.conf` y `ptp4l-sw.conf`. El
paso `[8/9]` los arrancaba con:

```bash
systemctl enable --now ptp4l-hw.service
```

**`--now` no reinicia una unidad que ya está activa**: sólo la arranca si estaba
parada. Así que el demonio seguía corriendo con la configuración vieja y el
cambio no se veía hasta el próximo reboot.

Es la misma trampa que ya había mordido con `gps-pps-ldattach` en v22 — el
comentario que lo explica está en el paso `[4/9]` de este mismo script, y no
apliqué la lección a las instancias de `ptp4l`. **Todo cambio hecho a los .conf
desde v18 (`utc_offset`, `timeSource`, el transporte) sólo se aplicó por
casualidad**, cuando el servicio se reinició por otro motivo.

### Corrección

Se toma la huella `sha256` de cada conf **antes** de reescribirlo, y en el paso
`[8/9]` se reinicia **sólo lo que cambió**:

```bash
reiniciar_si_cambio ptp4l-hw.service "$SHA_HW_ANTES" /etc/linuxptp/ptp4l-hw.conf
reiniciar_si_cambio ptp4l-sw.service "$SHA_SW_ANTES" /etc/linuxptp/ptp4l-sw.conf
```

Reiniciar `ptp4l-sw` sin motivo obliga a las tres cámaras a reenganchar el PTP;
no hay razón para molestarlas si su configuración no cambió. `phc2sys` se
reinicia siempre porque su configuración vive en el `ExecStart` de la unidad,
que el paso `[8/9]` acaba de reescribir.

### Y una verificación, para que no se vuelva a esconder

Al final del paso `[8/9]` se consulta al demonio **que está corriendo**:

```
✓ ptp4l-hw esta anunciando clockClass 6 (lo configurado)
```

Si no coincide, lo dice y da el comando para forzarlo. Un cambio de
configuración que no se aplica es invisible: el script reporta éxito y el sensor
sigue viendo lo viejo.

---

## v39 — El arreglo de v38 tenía un agujero: comparaba lo que no debía

### El síntoma

```
----- conf en disco -----
clockClass          6
----- lo que anuncia el demonio -----
		gm.ClockClass                         248
```

y **sin** la línea `cambio -> reiniciando ptp4l-hw.service`.

### La causa

v38 comparaba el `sha256` del `.conf` antes y después de reescribirlo, o sea
respondía a *"¿cambió el archivo **en esta corrida**?"*. Esa es la pregunta
equivocada.

Secuencia real:

1. **v37** escribió `clockClass 6` en el `.conf` y **no** reinició (ese era el
   bug de v38).
2. **v38** reescribió el mismo contenido, el sha dio igual → no reinició.
3. El proceso quedó con la configuración vieja **para siempre**, con el archivo
   en disco diciendo otra cosa.

Un `.conf` que ya está actualizado de una corrida anterior fallida deja el
sistema permanentemente inconsistente, y cada corrida siguiente lo confirma
como "sin cambios".

### Corrección

La pregunta correcta es *"¿lo que **anuncia** el demonio coincide con lo
configurado?"*. Se le pregunta por `pmc`, y si no coincide **se reinicia y se
vuelve a verificar**. Se auto-corrige sin importar el historial.

Los cuatro estados, probados:

| caso | resultado |
|---|---|
| ya coincide | `✓ anuncia 6` — no toca nada |
| no coincide y el restart lo arregla | `-> reiniciando` + `✓ anuncia 6` |
| no coincide y persiste | `✖ sigue anunciando 248 tras reiniciar` + qué revisar |
| `pmc` no responde | `⚠ no pude consultar` + `systemctl status ptp4l-hw` |

Se conserva el restart-por-sha de v38 (evita reiniciar `ptp4l-sw` sin motivo y
obligar a las tres cámaras a reenganchar); la verificación es la red de
seguridad que lo cubre cuando el sha miente.

---

## v40 — Calidad declarada del grandmaster

### Lo que quedó descartado

Con el cable reparado y `clockClass 6` confirmado, la cadena PTP hacia el Hesai
es **impecable** y eso descarta varias hipótesis:

```
ptp4l-hw: active (running) since ... 20min ago     -> 0 reinicios, sin flapping
Announce + Sync + Follow_Up cada 2 s               -> cadencia correcta
Delay_Req del LiDAR contestado en ~50 us           -> intercambio E2E cierra
```

El `Time Offset for LiDAR Lock` ya está en 100 µs (el máximo) y sigue en
`Free Run`. O sea que no es el umbral, ni el reinicio, ni la cadencia.

### Lo que se puede cambiar, medido

Se enumeraron **todas** las claves de configuración de ptp4l 4.0. De las que
afectan al Announce, esto es lo que anuncia hoy el grandmaster:

| campo | valor | significado |
|---|---|---|
| `gm.ClockClass` | 6 | sincronizado a referencia primaria ✓ |
| `gm.ClockAccuracy` | **0xfe** | **DESCONOCIDA** |
| `gm.OffsetScaledLogVariance` | **0xffff** | **DESCONOCIDA** |
| `currentUtcOffsetValid` | 0 | no válido |
| `timeTraceable` | 0 | no trazable |

Anunciar `clockClass 6` junto con accuracy y varianza **desconocidas** es
contradictorio: un reloj que dice estar sincronizado a una referencia primaria
debería saber su exactitud. Un esclavo que valida la calidad del maestro puede
rechazarlo por eso.

**Medido:** ni `utc_offset` ni `leapfile` consiguen poner
`currentUtcOffsetValid` ni `timeTraceable` en 1 — linuxptp 4.0 **no expone esos
flags como opción de configuración**. Se probaron las cuatro combinaciones.

Así que las únicas dos palancas que quedan son:

```bash
PTP_CLOCK_ACCURACY="${PTP_CLOCK_ACCURACY:-0x21}"      # mejor que 25 ns
PTP_OFFSET_VARIANCE="${PTP_OFFSET_VARIANCE:-0x4E5D}"  # tipica de un GM bueno
```

`0x21` es realista, no optimista: `phc_ctl enp11s0 cmp` da decenas de ns.

Verificado con ptp4l real: anuncia `gm.ClockAccuracy 0x21` y
`gm.OffsetScaledLogVariance 0x4e5d`.

### Sin verificar

Que esto haga enganchar al Hesai. Es la última palanca disponible del lado del
maestro; si con esto sigue en Free Run, lo que queda es un requisito de firmware
que linuxptp no puede satisfacer (probablemente `timeTraceable`).

**Y hay una pregunta previa que importa más que la etiqueta de la web UI:** si
los timestamps del LiDAR ya siguen al reloj del host. `Free Run` en la UI y
timestamps correctos no son incompatibles — hay que medirlo con `sync_check`, no
leerlo.
