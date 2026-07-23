# ars430_ros_publisher — Driver ROS2 Jazzy para el radar ARS430 (protocolo RDI v2)

> ✅ Verificado en Ubuntu 24.04 + ROS2 Jazzy: `colcon build` limpio y pipeline
> completo probado reproduciendo una captura real del radar (`pcap_file`):
> decodificación v2, nube PointCloud2 con campos `intensity`/`velocity` a
> ~25 Hz, y cambio de parámetros en caliente comprobado
> (`velocity_min` 0→5 hizo caer las detecciones publicadas del 97 % al 1 %,
> y `raw:=true` las devolvió al 100 %, sin reiniciar nada).
> También verificado `radar_objects`: 6-8 cubos con IDs persistentes (1-8) y
> etiquetas, posiciones estables al centímetro entre scans, y `prob_min`
> filtrando en caliente.

Port a ROS2 Jazzy del driver del radar Continental ARS430 con el **protocolo
RDI v2** (nuestro radar: multicast `239.0.0.1:40000`, ingeniería inversa
julio 2026). Tres nodos:

| Nodo | Función | Topics |
|---|---|---|
| `radar_publisher` | Sniffer libpcap + decoder v2 (deduplica las 6 repeticiones por ciclo) + decoder de la **lista de objetos del tracker interno** (service 230). También puede reproducir un pcap. | → `/unfiltered_radar_packet_<id>`, `/radar_objects_raw_<id>` |
| `radar_processor` | Filtros de detecciones, **todos ajustables en caliente** | → `/filtered_radar_packet_<id>` |
| `radar_visualizer` | RadarPacket → PointCloud2 (campos `intensity`=RCS y `velocity`) + líneas FOV + TF | → `/radar_pointcloud_<id>` |
| `radar_objects` | Objetos del tracker → **MarkerArray**: un cubo tamaño persona por objeto con etiqueta "ID n prob%", color estable por ID y opacidad proporcional a la probabilidad | → `/radar/markers` |
| `radar_clusters` | **Objetos propios desde los puntos**: gate de velocidad + DBSCAN + persistencia → cubo con etiqueta "P n  v m/s" exactamente sobre el cluster de detecciones. El recomendado para personas en interior. | → `/radar/cluster_markers` |

## 1. Compilar

```bash
# Dependencias
sudo apt install ros-jazzy-desktop libpcap-dev

# Copiar SOLO esta carpeta al workspace (no todo el repo)
mkdir -p ~/ros2_ws/src
cp -r ros2/ars430_ros_publisher ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select ars430_ros_publisher
source install/setup.bash

# Permiso de captura (repetir tras cada colcon build)
sudo setcap 'cap_net_raw=pe' install/ars430_ros_publisher/lib/ars430_ros_publisher/radar_publisher
```

## 2. Ejecutar

```bash
# Radar en vivo (defaults: iface enx00e04c3604d7, puerto 40000, RViz incluido)
ros2 launch ars430_ros_publisher radar_live.launch.py

# Sin filtros (diagnóstico: ver TODO lo que reporta el radar)
ros2 launch ars430_ros_publisher radar_live.launch.py raw:=true

# Validar el pipeline SIN radar, reproduciendo una captura tcpdump
ros2 launch ars430_ros_publisher radar_live.launch.py pcap_file:=/ruta/radar_raw.pcap
```

Recordad preparar la red antes (igual que en Noetic):

```bash
sudo ip addr add 10.1.1.10/24 dev enx00e04c3604d7 && sudo ip link set enx00e04c3604d7 up
sudo tcpdump -i enx00e04c3604d7 -c 5 'udp port 40000'   # debe verse 10.1.1.20 -> 239.0.0.1
```

## 3. Parámetros

### `radar_publisher` (se fijan al arrancar)

| Parámetro | Default | Descripción |
|---|---|---|
| `id` | 1 | Sufijo de los topics |
| `iface` | `enx00e04c3604d7` | Interfaz de red |
| `port` | 40000 | Puerto UDP del radar |
| `bpf_extra` | "" | Filtro BPF adicional |
| `pcap_file` | "" | Si se indica, reproduce ese pcap en vez de capturar en vivo |
| `pcap_loop` | true | Repetir el pcap en bucle |
| `pcap_realtime` | true | Reproducir el pcap al ritmo real de sus timestamps (false = a máxima velocidad) |

### `radar_processor` (⚡ TODOS modificables en caliente)

| Parámetro | Default | Unidad | Qué filtra |
|---|---|---|---|
| `raw` | false | — | `true` = reenvía todo sin filtrar |
| `snr_min_near` | 0.0 | dB | SNR mínimo del near scan (evento 4) |
| `snr_min_far` | 0.0 | dB | SNR mínimo del far scan (evento 2) |
| `velocity_min` | 0.0 | m/s | Descarta \|vrel\| menor (0 = desactivado). **El filtro más potente para aislar personas.** |
| `velocity_max` | 100.0 | m/s | Descarta \|vrel\| mayor |
| `range_min` | 0.25 | m | Rango mínimo |
| `range_max` | 100.0 | m | Rango máximo |
| `az_max_deg` | 90.0 | grados | \|acimut\| máximo |
| `rcs_min` | −100.0 | dBsm | RCS mínimo |

**Cambiarlos en vivo** (sin recompilar ni relanzar, efecto inmediato en RViz):

```bash
ros2 param set /radar_processor velocity_min 0.4
ros2 param set /radar_processor range_max 15.0
ros2 param set /radar_processor raw true          # bypass total
ros2 param get /radar_processor velocity_min      # consultar
ros2 param dump /radar_processor                  # volcar la config actual a YAML
```

El processor imprime cada 5 s cuántas detecciones entran, cuántas salen y qué
filtro descarta cada una — usadlo como feedback mientras ajustáis. También se
puede usar `rqt` (plugin *Dynamic Reconfigure / Parameters*) para moverlos con
sliders.

Para dejar fijada una configuración buena, guardadla y cargadla en el launch:

```bash
ros2 param dump /radar_processor > mi_filtro.yaml
ros2 run ars430_ros_publisher radar_processor --ros-args --params-file mi_filtro.yaml
```

### `radar_visualizer` (se fijan al arrancar)

| Parámetro | Default |
|---|---|
| `input_topic` | `/filtered_radar_packet_1` |
| `output_topic` | `/radar_pointcloud_1` |

### `radar_objects` (⚡ TODOS modificables en caliente)

Los objetos NO se calculan agrupando puntos: vienen ya seguidos (tracked) del
firmware del radar por el service 230, con ID persistente y probabilidad de
existencia. El "umbral de objeto" es esa probabilidad:

| Parámetro | Default | Qué hace |
|---|---|---|
| `prob_min` | 50 | Probabilidad de existencia mínima (0–100). **El umbral de "¿esto es un objeto de verdad?"** |
| `min_seen_scans` | 3 | Persistencia: mostrar el track solo tras verlo en N scans consecutivos (~0,2 s con N=3) |
| `range_min` / `range_max` | 0.5 / 100.0 | Puerta radial [m] |
| `invert_y` | false | Invertir el eje lateral para casar con la nube (ver calibración abajo) |
| `offset_x` / `offset_y` | 0.0 | Corrección constante del origen [m] — el tracker puede reportar en coordenadas de vehículo con montaje configurado en firmware |
| `marker_size_xy` / `marker_size_z` | 0.5 / 1.7 | Dimensiones del cubo [m] (tamaño persona) |
| `marker_alpha` | 0.8 | Opacidad base (se multiplica por prob/100) |
| `marker_lifetime_s` | 0.5 | Cuánto tarda en desvanecerse un cubo sin refrescar |
| `show_labels` | true | Etiquetas de texto "ID n prob%" |

```bash
ros2 param set /radar_objects prob_min 80        # más estricto
ros2 param set /radar_objects min_seen_scans 1   # mostrar tracks al instante
ros2 param set /radar_objects invert_y true      # calibración del eje lateral
```

**Calibración de `invert_y`** (una sola vez): con RViz abierto, camina hacia
TU izquierda mirando al radar; si el cubo se mueve al lado contrario que los
puntos de la nube, pon `invert_y` a `true`.

**Valores recomendados**: para demo/operación `prob_min 80` y
`min_seen_scans 3` (solo objetos consolidados); para depurar el tracker
`prob_min 0` y `min_seen_scans 1` (todo lo que el radar cree ver); para
seguimiento de personas en interior añade `range_max 20`.

**Aviso de campos tentativos**: del slot de objeto solo están verificadas
posición (x, y), ID y probabilidad. El campo `rcs` del mensaje es una
hipótesis y la velocidad del track aún no está identificada (el nodo no la
usa). La lista viaja en un datagrama fragmentado: se ven los primeros 30 de
62 slots — de sobra salvo escenas con >30 objetos simultáneos.

### `radar_clusters` (⚡ TODOS modificables en caliente)

Objetos construidos por nosotros desde las detecciones: gate de velocidad →
DBSCAN → asociación entre ciclos → cubo etiquetado `"P n  v m/s (k pts)"` en
`/radar/cluster_markers`. Como usan las mismas detecciones que la nube, **el
cubo cae exactamente sobre el cluster de puntos**, sin calibración. Con el
gate por defecto (0.3 m/s) una escena estática no muestra ningún cubo; al
caminar alguien, aparece el suyo.

| Parámetro | Default | Qué hace |
|---|---|---|
| `velocity_min` | 0.3 | Gate de velocidad [m/s]. 0 = clusteriza también lo estático (diagnóstico) |
| `eps` | 1.0 | Radio de vecindad DBSCAN [m] |
| `min_points` | 4 | Detecciones mínimas para formar cluster |
| `min_cycles` | 2 | Persistencia: ciclos consecutivos antes de mostrar el cubo |
| `window_s` | 0.2 | Ventana de acumulación de detecciones [s] (~2 ciclos de radar) |
| `assoc_dist` | 1.0 | Salto máximo del centroide para mantener el mismo ID [m] |
| `marker_size_z` / `marker_alpha` / `marker_lifetime_s` / `show_labels` | 1.7 / 0.7 / 0.4 / true | Apariencia (la base del cubo se ajusta sola a la extensión del cluster, 0.4–1.5 m) |

Recetas: persona única en interior → defaults tal cual. Si el cubo parpadea,
`window_s 0.3` y `min_points 3`. Si dos personas cercanas se funden en un
cubo, `eps 0.7`. Si salen cubos espurios durante el movimiento (fantasmas de
multipath agrupados), sube `min_points` a 6 y limita `range_max` en el
`radar_processor` al fondo real de la sala.

### ¿Tracker del radar (`/radar/markers`) o clusters propios (`/radar/cluster_markers`)?

**Veredicto tras el experimento de calibración (jul 2026, 2 capturas con
movimiento controlado): usad los clusters.** El tracker (service 230) queda
**desactivado por defecto** (`enable_tracker:=true` para reactivarlo).

Lo que mostró el experimento:

1. **La nube de puntos está bien orientada**: caminando hacia la izquierda
   del radar, la Y del cluster sube (+Y = izquierda, convenio ROS). No hace
   falta `invert_y` en la nube ni cambiar nada.
2. **El service 230 no es utilizable en interior con lo que sabemos del
   formato**: con escena dinámica, ~28 de 30 filas visibles llevan el magic
   pero el ~70 % tiene posiciones físicamente imposibles (hasta >300 m), el
   byte 0 resultó ser índice de fila (no ID persistente), y ningún campo
   decodificado (probabilidad, estado, clase) separa las filas reales de la
   basura — la probabilidad marca 100 % también en las filas imposibles. Sin
   la especificación del firmware no hay criterio de validez fiable.
3. **Los fantasmas de movimiento NO son débiles**: SNR mediana ~20 dB (más
   que la propia persona, ~18 dB). El filtro de SNR no los elimina (1 %).
   Sí funcionan: el **clustering con persistencia** (los fantasmas son
   dispersos y transitorios; la persona es un cluster compacto que se
   repite) y el **recorte de rango** (63 % de los fantasmas aparecen más
   allá de 8 m con la persona a ~5 m — poned `range_max` al fondo real de
   la sala).

Receta recomendada para persona en interior tras el experimento:

```bash
ros2 param set /radar_processor range_max 10.0     # fondo real de la sala
ros2 param set /radar_clusters min_points 5        # robustez ante fantasmas agrupados
ros2 param set /radar_clusters min_cycles 3
```

## 4. Parámetros recomendados por escenario de prueba

**Escenario A — Primer arranque / diagnóstico** (ver todo):

```bash
ros2 launch ars430_ros_publisher radar_live.launch.py raw:=true
```

**Escenario B — Persona moviéndose en interior** (el caso actual). El truco
es el filtro de velocidad: el clutter estático (paredes, mesas, multipath) es
el >90 % de los puntos y tiene v≈0; la persona camina a 0,5–2 m/s:

```bash
ros2 param set /radar_processor velocity_min 0.4
ros2 param set /radar_processor range_min 0.5
ros2 param set /radar_processor range_max 20.0
ros2 param set /radar_processor snr_min_near 5.0
ros2 param set /radar_processor snr_min_far 0.0     # el far scan en interior tiene SNR 0-9: no lo filtréis
```

Con esto en RViz debería quedar prácticamente solo la persona. Si desaparece
al pararse (v→0), bajad `velocity_min` a 0,2–0,3 o alternad con `raw`.

**Escenario C — Exterior, persona a 10–20 m** (réplica de los bags de
WATonomous):

```bash
ros2 param set /radar_processor velocity_min 0.3
ros2 param set /radar_processor range_min 1.0
ros2 param set /radar_processor range_max 40.0
ros2 param set /radar_processor snr_min_near 5.0
ros2 param set /radar_processor snr_min_far 3.0
```

**Escenario D — Mapa estático del entorno** (lo contrario: solo clutter):

```bash
ros2 param set /radar_processor velocity_min 0.0
ros2 param set /radar_processor velocity_max 0.1
ros2 param set /radar_processor snr_min_near 8.0
```

**Método de ajuste recomendado**: empezad SIEMPRE en `raw:=true` para ver la
línea base; activad los filtros de uno en uno (primero `velocity_min`, después
rango, después SNR) mirando el informe del processor y RViz; anotad con
`ros2 param dump` la configuración final por escenario.

En la config de RViz incluida, la nube FILTRADA se colorea por **velocity**
(blanco = estático, color = en movimiento) y la RAW por **intensity** (RCS,
verde = débil → rojo = fuerte). Ambas con Decay Time 1 s.

## 5. Verificación con estadísticas

```bash
ros2 run ars430_ros_publisher radar_stats.py -t /unfiltered_radar_packet_1
ros2 run ars430_ros_publisher radar_stats.py -t /filtered_radar_packet_1
```

Referencia sana en interior con el radar v2: ~33 paquetes/s tras deduplicar
(16,7 ciclos/s × 2 eventos), 6–9 det/paquete en far y 19–28 en near, SNR near
mediana ~11 dB, y % en movimiento >0 cuando alguien camina delante.

## 6. Diferencias respecto al driver ROS1 / port antiguo

1. **Decoder v2 integrado** (little-endian, registros de 27 B, deduplicación
   de las 6 repeticiones). El port ROS1→ROS2 antiguo usaba el layout de 2019 y
   decodificaba basura — sustituidlo por este paquete.
2. Los nombres de campo de los mensajes son `snake_case` (obligatorio en
   ROS2): `EventID→event_id`, `posX→pos_x`, `VrelRad→vrel_rad`, `AzAng→az_ang`,
   `RCS→rcs`, `SNR→snr`… y `RadarDetection` gana `range` y `flags`.
3. Los filtros dejan de ser `#define` (recompilar) y pasan a parámetros
   dinámicos.
4. La nube incluye el campo `velocity` para colorear por movimiento.
5. Sensor Status v2 y el service 230 (¿lista de objetos?) aún sin layout
   conocido: se ignoran. `vrel_rad_var` no identificado (no publicado);
   `vambig` es tentativo.
6. Limitación de fragmentación: del near scan solo llegan los primeros 52
   registros por el filtro BPF (aviso `tail lost` si se supera; en interior
   no ocurre).

## 7. Problemas típicos

| Síntoma | Causa/solución |
|---|---|
| `radar_publisher` muere con `exit code 127` y `error while loading shared libraries: librclcpp.so` | Al ponerle capabilities (`setcap`), el linker entra en modo seguro e ignora `LD_LIBRARY_PATH`. **Ya está resuelto en este paquete** (RPATH absoluto + `--disable-new-dtags` en el CMakeLists): recompilad desde cero (`rm -rf build install && colcon build ...`) y volved a aplicar el `setcap`. |
| `pcap_open_live: permission denied` | Falta el `setcap` (sección 1) |
| No llegan paquetes | IP/interfaz sin configurar, o switch con IGMP snooping (usad `multicast_join.py` del paquete ROS1 o conexión directa) |
| `RDI packet is not protocol v2` | Radar con firmware v1 → usad el driver Noetic, o pedid soporte v1 |
| `tail lost` | Más de 52 detecciones near en un ciclo (exterior denso): perdéis la cola, el resto sigue siendo válido |
| RViz no muestra nada | Comprobad Fixed Frame `radar_fixed` y los topics `/radar_pointcloud_1` y `/radar_pointcloud_99` |
