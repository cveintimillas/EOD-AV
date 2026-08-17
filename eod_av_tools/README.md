# eod_av_tools

## `sync_check`

Mide el desfase real de `header.stamp` de cada tópico contra el reloj del host.

```bash
ros2 run eod_av_tools sync_check
```

Descubre los tópicos solo. Para fijarlos:

```bash
ros2 run eod_av_tools sync_check --ros-args \
  -p topics:="['/lidar_points','/arena_camera_node/enp6s0/image','/fix']" \
  -p report_period:=15.0
```

### Por qué no alcanza `ros2 topic delay`

Es Python y usa una cola RELIABLE. Sobre `/lidar_points` (5 MB por mensaje a
10 Hz = 50 MB/s) no da abasto: la cola crece y lo que reporta como *delay* es su
propio atraso. Se lo vio subir de 3,98 s a 4,42 s en 12 s de medición — 78000
ppm, imposible para un reloj.

`sync_check` evita las dos causas: es C++ (sin GIL) y usa **BEST_EFFORT con
depth 1**, así que si no llega a procesar **descarta** en vez de encolar. Un
mensaje perdido no sesga la medición; uno encolado sí.

### Cómo lee el timestamp sin conocer el tipo

Con `GenericSubscription`, leyendo los 12 primeros bytes del CDR. Todo mensaje
cuyo primer campo sea un `std_msgs/Header` los tiene en un lugar fijo. Funciona
igual con `PointCloud2`, `Image`, `Imu`, `NavSatFix` y con los mensajes propios
del radar, sin depender de sus paquetes.

Los tópicos que **no** empiezan con `Header` se detectan solos (el timestamp no
da plausible) y se cuentan aparte en vez de ensuciar la tabla.

### Cómo leer la salida

```
                              n     media       min       max        sd    deriva
  ------------------------------------------------------------------------------
  /lidar_points              100  45.20 ms  40.10 ms  52.30 ms   2.10 ms    0.4ppm
  ...camera_node/enp6s0/image 1200 118.00 ms 117.00 ms 119.00 ms 510.0 us   -0.2ppm
```

| Columna | Qué significa |
|---|---|
| `media` | `now - header.stamp` = **(offset de reloj) + (latencia real del sensor)**. Los dos términos están sumados |
| `sd` | **Lo que importa desde acá**: estabilidad. Chico y constante = el reloj del sensor sigue al del host |
| `deriva` | Pendiente del delay, en ppm. ~0 = enganchado. Sostenida y grande = el sensor corre con su propio cristal, **o** el nodo se está atrasando y acumulando backlog |

### Lo que este nodo NO puede decir

`media` **no** es el desfase de reloj, y **restar dos medias tampoco lo da**: la
resta arrastra la diferencia de latencia de pipeline, que entre una cámara
(exposición + transferencia de 3 MB) y un radar (un paquete UDP) es de decenas
de milisegundos aunque los relojes estén perfectos.

Para separar offset de latencia hace falta un **evento físico común** visible
por los dos sensores — un LED estroboscópico entre cámaras, un objeto que cae
para LiDAR↔cámara.

### Cómo distinguir "reloj libre" de "nodo atrasado"

Las dos dan `deriva` grande. La diferencia está en `min`:

* `min` **estable** y sólo la media/max crecen → el nodo acumula backlog: los
  timestamps son correctos, la entrega es tardía.
* `min` **también crece** → el reloj del sensor se está separando de verdad.

### Tests

```bash
colcon test --packages-select eod_av_tools
# o sin ROS:
g++ -std=c++17 -Iinclude -o /tmp/t test/test_sync_stats.cpp && /tmp/t
```

Cubren los dos endianness del CDR, buffers cortos, tópicos sin `Header`, y la
regresión de deriva contra rampas de pendiente conocida (recupera 50,00 ppm de
una rampa de 50 ppm).
