# Arquitectura de sincronización temporal — EOD-AV

Diseño de referencia para la plataforma multisensor sobre Ubuntu 24.04 + ROS 2
Jazzy, sin switch PTP.

Lo que está **medido** en esta máquina o en una reproducción se marca como tal.
Lo que **no está verificado** también se marca. No hay nada dado por bueno sin
una de las dos etiquetas.

---

## 1. El principio que ordena todo el diseño

Hay una sola fuente de tiempo y un solo reloj central:

> **El PPS del GNSS disciplina `CLOCK_REALTIME` vía chrony. Todo lo demás
> cuelga de `CLOCK_REALTIME`.**

No hay dos cadenas de tiempo compitiendo. `ptp4l` **nunca** ajusta el reloj del
sistema (corre con `serverOnly 1`, o sea siempre grandmaster, nunca cliente), y
`chrony` es el único que lo toca. `phc2sys` sólo escribe en el PHC de la Intel.

Esto importa porque el modo de fallo más común en estos stacks es chrony y
ptp4l peleándose por `CLOCK_REALTIME`, con el offset oscilando sin converger.

### Diagrama

```
   simpleRTK3B
    │        │
    │ NMEA   │ TIMEPULSE (1 PPS)
    │ USB    │
    ▼        ▼
 /dev/ttyUSB0   FTDI232 DCD ── /dev/ttyUSB1 ── pps_ldisc ── /dev/ppsN
    │                                                          │
    ▼                                                          │
  gpsd ──SHM(0) grosero──┐                  ┌──SHM(1) fino ────┘
                         ▼                  ▼
                    ┌─────────────────────────────┐
                    │   chrony   →  CLOCK_REALTIME │  (UTC)
                    └─────────────────────────────┘
                       │            │             │
        ┌──────────────┘            │             └──────────────┐
        ▼                           ▼                            ▼
  phc2sys -s CLOCK_REALTIME   ptp4l-sw (SW ts, L2)         gpsd_client
        -c enp11s0            enp6s0/7s0/8s0                  → /fix
        │                     esclavos de bridge0        radar: SO_TIMESTAMPING
        ▼                           │                     del socket pcap
   PHC de la I225-V                 ▼
        │                    3× Triton TRI023S
        ▼                     (PtpSlaveOnly)
   ptp4l-hw (HW ts, UDPv4)
        │
        ▼
   Hesai XT32M2X
```

### Por qué dos instancias de `ptp4l`

`time_stamping` es una opción **global** de `ptp4l`: una instancia no puede
mezclar puertos con hardware timestamping y puertos sin él. Como la I225-V
tiene PHC y las RTL8125 no, hacen falta dos procesos:

| Instancia | Puertos | `time_stamping` | Reloj que sirve | Transporte |
|---|---|---|---|---|
| `ptp4l-hw` | `enp11s0` | `hardware` | PHC de la I225-V | `UDPv4` |
| `ptp4l-sw` | `enp6s0`, `enp7s0`, `enp8s0` | `software` | `CLOCK_REALTIME` | `L2` |

Cada una necesita su propio `uds_address` (`/var/run/ptp4l-hw` y
`/var/run/ptp4l-sw`); si no, la segunda no arranca porque el socket por defecto
`/var/run/ptp4l` ya está tomado.

Ambas son grandmaster en dominio 0 sobre **segmentos L2 físicamente separados**,
así que nunca se ven entre sí y no hay BMCA cruzado. Los dos relojes que sirven
(PHC y `CLOCK_REALTIME`) los mantiene alineados `phc2sys`, así que las cuatro
familias de sensores terminan en la misma escala.

---

## 2. Respuestas a las seis preguntas

### 2.1 ¿Arquitectura PTP sin comprar un switch?

La de arriba. La PC es grandmaster en las dos instancias y cada sensor cuelga de
un enlace punto a punto (o del bridge, en el caso de las cámaras). No hace falta
switch: **no** necesitás un *boundary clock* ni un *transparent clock* porque no
hay ningún salto de red entre el grandmaster y los esclavos.

Un switch PTP-aware sólo haría falta si un mismo puerto tuviera que servir a
varios dispositivos a través de un equipo intermedio que introduce latencia
variable. Acá el único elemento intermedio es `bridge0`, y es software: ver 2.2.

### 2.2 ¿Puede Ubuntu ser grandmaster con la I225-V y servir al Hesai y a las cámaras?

**Al Hesai, sí, con timestamping por hardware.** `enp11s0` tiene PHC
(`PTP Hardware Clock: 0`), es punto a punto contra el LiDAR, y no está en el
bridge. Ese es el enlace bueno.

**A las cámaras, no con el PHC.** No se puede "repartir" el PHC de la Intel a
puertos Realtek: el timestamping por hardware se toma en el silicio de la NIC
que transmite. Las RTL8125 no tienen PHC, así que su instancia de `ptp4l` usa
timestamping por software sobre `CLOCK_REALTIME`. Lo que las une al Hesai no es
el PHC, es que **`phc2sys` mantiene el PHC igual a `CLOCK_REALTIME`**.

Sobre los intentos que ya hiciste:

* `ptp4l -i bridge0 -m -H` → `interface 'bridge0' does not support requested
  timestamping mode`. **Reproducido.** Un netdev de bridge no expone
  capacidades de timestamping utilizables por `ptp4l`; tampoco con `-S`.
  El bridge no es un puerto PTP válido, ni por hardware ni por software.
* `ptp4l -i enp6s0 -m -H` → falla, correcto: no hay PHC.
* `ptp4l -i enp6s0 -m -S` → anda y toma el rol de grandmaster. **Pero eso no
  alcanza**, y es la trampa de todo esto: ver la sección siguiente.

#### El problema del bridge (MEDIDO)

Reproducción con `veth` + `bridge` en un network namespace, linuxptp 4.0,
grandmaster de un lado y cliente (`clientOnly 1`) del otro:

| Configuración | `path delay` | |
|---|---|---|
| Interfaz suelta, `UDPv4` | 1395–1552 ns | OK |
| **Esclavo de bridge, `UDPv4`** | **0** | **ROTO** |
| El bridge mismo (`br0`), `UDPv4` | no arranca | ROTO |
| **Esclavo de bridge, `L2`** | 1507–1696 ns | OK |

En un puerto de bridge, `br_handle_frame` (el `rx_handler`) se queda con la
trama **antes** de que llegue a los *protocol handlers* de IP. El socket UDP de
`ptp4l` con `SO_BINDTODEVICE` sobre `enp6s0` nunca ve el `Delay_Req` de la
cámara. El sentido inverso sí funciona, porque `ptp4l` transmite directo por el
netdev sin pasar por el bridge.

Por eso el síntoma engaña: la cámara ve el `Announce` y el `Sync`, loguea
`new foreign master`, pasa a `UNCALIBRATED` y reporta `master offset` — todo
parece andar. Pero el `path delay` queda en `0` para siempre: nunca se cierra el
intercambio E2E ni se compensa la latencia del enlace.

Con transporte **L2** desaparece, porque los taps de `AF_PACKET` corren en
`__netif_receive_skb_core` **antes** del `rx_handler` del bridge.

Se descartó STP y `mcast_snooping` como causa: el mismo test con
`stp_state 0 mcast_snooping 0` sigue dando `path delay 0`.

> **Sin verificar:** que las Triton acepten PTP sobre L2. Ver el paso B.3 del
> plan. Si sólo hablan UDPv4, el bridge y el PTP son incompatibles y hay que
> sacar las cámaras del bridge.

### 2.3 Integración gpsd + chrony + phc2sys + ptp4l

Ver el plan paso a paso, sección 3. El reparto de responsabilidades es:

| Componente | Qué hace | Qué **no** hace |
|---|---|---|
| `pps_ldisc` (`ldattach 18`) | Convierte los flancos de DCD en `/dev/ppsN` | — |
| `gpsd` | Parsea NMEA, alimenta SHM(0) y SHM(1) | No ajusta ningún reloj |
| `chrony` | **Único** que disciplina `CLOCK_REALTIME` | — |
| `phc2sys` | Copia `CLOCK_REALTIME` → PHC de `enp11s0` | No toca `CLOCK_REALTIME` |
| `ptp4l` ×2 | Sirve tiempo a LiDAR y cámaras | No ajusta ningún reloj (`serverOnly`) |

El refclock NMEA de gpsd es **grosero** (decenas de ms: el sentence llega por
USB después del segundo). Sirve sólo para dar el *segundo* correcto. La
precisión real la pone el PPS, que da la *fracción* de segundo. Por eso hacen
falta los dos: `refclock SHM 0` con `noselect` como fuente de ToD, y
`refclock SHM 1` como la fuente que se selecciona.

### 2.4 Configurar las Triton TRI023S con Arena SDK

Nodos GenICam relevantes:

| Nodo | Valor | Por qué |
|---|---|---|
| `PtpEnable` | `true` | Habilita el cliente PTP |
| `PtpSlaveOnly` | `true` | La cámara nunca intenta ser grandmaster |
| `PtpStatus` | *leer* | Tiene que llegar a `Slave` |
| `AcquisitionStartMode` | `PTPSync` | Disparo sincronizado por PTP entre las 3 |
| `AcquisitionFrameRate` | igual en las 3 | `PTPSync` exige que coincidan |

`arena_camera_node` ya escribe `PtpEnable` y `PtpSlaveOnly` en
`set_nodes_ptp_()`. **Tiene dos defectos conocidos**, ambos sin corregir:

1. Lee `PtpStatus` **una sola vez** al arrancar y no espera a que llegue a
   `Slave`. El enganche PTP tarda decenas de segundos.
2. Estampa cada imagen con `pImage->GetTimestampNs()`, que es el reloj del
   dispositivo. Antes del enganche ese reloj es el contador interno desde el
   boot de la cámara, así que **las primeras imágenes de cada sesión salen con
   timestamps basura** — típicamente "hace varias horas".

   ```cpp
   // arena_camera_node/src/ArenaCameraNode.cpp:~323
   image_msg.header.stamp.sec     = pImage->GetTimestampNs() / 1000000000;
   image_msg.header.stamp.nanosec = pImage->GetTimestampNs() % 1000000000;
   ```

   Mitigación operativa mientras no se corrija: **esperar a que `PtpStatus` sea
   `Slave` en las tres cámaras antes de empezar a grabar**, y descartar el
   primer minuto del bag.

`AcquisitionStartMode = PTPSync` es la ganancia grande y no está activado hoy:
hace que las tres cámaras expongan en el mismo instante en vez de correr libres
con fases arbitrarias. A 12 Hz, sin esto el desfase entre cámaras puede ser de
hasta 83 ms aunque los timestamps sean perfectos.

### 2.5 Verificar en ROS 2 que los timestamps están alineados

Tres niveles, de más barato a más concluyente:

**Nivel A — cada sensor contra el reloj del host.** `ros2 topic delay` mide
`now() − header.stamp`:

```bash
ros2 topic delay /lidar_points
ros2 topic delay /camera_enp6s0/image_raw
ros2 topic delay /fix
ros2 topic delay /radar/packets
```

Todos deberían dar el mismo orden de magnitud (decenas de ms, dominado por la
latencia de transporte real, no por desalineación). Qué delata qué:

* `±37 s` exactos → problema TAI/UTC (ver 2.6).
* Delay **negativo** → el sensor está adelantado: reloj no disciplinado.
* Delay de horas → PTP nunca enganchó, el dispositivo usa su contador interno.
* Delay que **crece linealmente** → el reloj del sensor corre libre, sin PTP.

**Nivel B — sensores entre sí, sobre un bag.** Grabá 60 s y comparé las marcas
de tiempo de mensajes contiguos entre tópicos. `ros2 bag info` ya muestra los
`Start`/`End` por tópico, lo que basta para detectar un desfase grueso.

**Nivel C — un evento físico común.** Es el único que valida de punta a punta.
Un LED estroboscópico visible por las tres cámaras da el desfase
cámara↔cámara directamente. Para LiDAR↔cámara sirve un objeto que cae o un
péndulo: se compara el instante en que cruza un plano conocido.

Los niveles A y B validan la *cadena de relojes*; sólo el C valida que el
timestamp corresponde al instante de la *medición* (exposición, disparo del
láser) y no al de la entrega del dato.

### 2.6 Precisión realista esperable

| Eslabón | Jitter/error típico | Comentario |
|---|---|---|
| PPS por **FTDI DCD (USB)** | **50–200 µs**, outliers de ms | Limitado por el polling USB y el `latency_timer` del FTDI |
| PPS por DCD de un **UART nativo** (cabezal COM) | 1–5 µs | Ver la nota de hardware abajo |
| NMEA por USB, sin PPS | 10–50 ms | Sólo sirve para el segundo entero |
| `phc2sys` `CLOCK_REALTIME` → PHC | 100 ns – 1 µs | Muy bueno, no es el cuello de botella |
| PTP **hardware** (I225-V ↔ Hesai) | **±100 ns – 1 µs** | Sobre el cable |
| PTP **software** (RTL8125 ↔ Triton) | **±20–100 µs** | Depende de carga de CPU e IRQ; peor bajo tráfico de imagen |
| Radar ARS430 | ver nota | Sin PTP; depende de cómo se estampe |

**La conclusión que importa para fusión sensorial:** hay que separar dos cosas.

* **Exactitud absoluta contra UTC** — la limita el eslabón más débil, que es el
  PPS por FTDI: **~100 µs**. Para georreferenciar contra una base RTK externa,
  eso es lo que hay.
* **Exactitud relativa entre sensores** — es mucho mejor, porque todos cuelgan
  del *mismo* `CLOCK_REALTIME` y un error común se cancela. LiDAR↔host queda
  sub-µs; cámara↔host en decenas de µs; cámara↔LiDAR en el orden de **50–100 µs**.

Para fusión LiDAR-cámara-radar lo que manda es la relativa, y 100 µs a 12 Hz es
el 0,1 % del período de cuadro. Sobra.

#### Nota de hardware: el cabezal COM de la placa

La ASUS ROG Strix B550-F Gaming tiene un cabezal COM en la placa (confirmalo en
el manual). Un UART 8250/16550 nativo hace que el PPS pase de ~100 µs a ~1–5 µs,
porque el flanco de DCD genera una interrupción directa en vez de esperar al
próximo poll USB. Es la mejora más grande disponible por menos plata que
cualquier otra cosa del stack. Requiere un adaptador de niveles del cabezal a
DB9/TTL y que el puerto aparezca como `/dev/ttyS0`.

Mientras tanto, bajar el `latency_timer` del FTDI de 16 ms a 1 ms es gratis y se
nota (ver paso A.4).

#### Lo que NO aplica: `ts2phc`

`ts2phc` disciplina el PHC **directamente desde el PPS**, salteándose
`CLOCK_REALTIME`, y sería la arquitectura ideal. No sirve acá: sus dos fuentes
son `generic` (un 1-PPS conectado a un **pin de hardware** de la NIC) y `nmea`.
La I225-V de una placa de consumo no expone los pines SDP, y el PPS llega por
USB. Queda como camino de mejora si alguna vez se agrega una NIC con entrada PPS
por SMA.

---

## 3. Plan de configuración paso a paso

`setup/setup_ptp_sync.sh` implementa A y B. Esta sección explica qué hace y en
qué orden, para poder auditarlo y para diagnosticar cuando falla.

### Bloque A — la referencia de tiempo (GNSS → `CLOCK_REALTIME`)

**A.1 — Configurar el TIMEPULSE del receptor.** En u-center (o el configurador
del receptor):

* Frecuencia: **1 Hz**
* Alineado al **top of second UTC**
* Ancho de pulso: **100 ms** (un pulso de µs puede perderse entre polls USB)
* *Lock to GPS/UTC*: activado
* Polaridad: flanco **ascendente** como *assert*

**A.2 — Identificar cuál `/dev/ppsN` es el del GNSS.** Crítico y fácil de
equivocar: el PHC de la Intel también registra un dispositivo PPS.

```bash
for p in /sys/class/pps/pps*; do echo "$p -> $(cat $p/name)"; done
```

* nombre `ptp0` → es el **PHC de la I225-V**, no el GNSS.
* nombre `usbserialN` → es el **FTDI**, ése es el del GNSS.

**A.3 — Enganchar la disciplina de línea PPS.**

```bash
sudo modprobe pps_ldisc
sudo ldattach 18 /dev/ttyUSB1     # 18 = N_PPS
```

**A.4 — Bajar la latencia del FTDI** (de 16 ms a 1 ms):

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
```

Permanente, vía udev:

```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", \
  ATTR{latency_timer}="1"
```

**A.5 — gpsd** con el dispositivo correcto, sin socket-activation (el
`gpsd.socket` de Ubuntu contesta en el 2947 aunque no tenga ningún device, y
entonces `/fix` queda mudo sin un solo error visible).

**A.6 — chrony**:

```
refclock SHM 0 refid NMEA offset 0.0 delay 0.2 poll 4 noselect
refclock SHM 1 refid PPS  precision 1e-6 poll 3 prefer
```

`noselect` en el NMEA es deliberado: da el segundo entero pero es demasiado
ruidoso para seleccionarlo como fuente.

**A.7 — Verificar el bloque A antes de seguir.**

```bash
sudo ppstest /dev/ppsN            # con la N de A.2
chronyc sources -v
chronyc tracking
```

> **Cómo leer `ppstest`.** Fijate en la **parte fraccionaria** del assert. Un
> PPS real disciplinando un reloj ya convergido asserta cerca de `.000000` o
> `.999999`. Si ves algo como `.362924` con deriva de decenas de µs por
> segundo, el reloj del sistema todavía **no está disciplinado** (deriva libre
> de ~50 ppm, que es lo normal en un cristal de PC). Eso está bien al principio;
> lo que no puede pasar es que siga así después de que `chronyc tracking` diga
> que convergió. Si sigue, o el `/dev/ppsN` es el del PHC (ver A.2) o el
> refclock no está seleccionado.

### Bloque B — distribución por PTP

**B.1 — `ptp4l-hw` (Hesai).** `enp11s0`, hardware timestamping, `UDPv4`.

**B.2 — `phc2sys`.** `CLOCK_REALTIME` → PHC de `enp11s0`.

**B.3 — Averiguar qué transporte PTP hablan las Triton.** Con `PtpEnable` ya
activo, **antes** de fijar la configuración:

```bash
sudo timeout 20 tcpdump -ni enp6s0 -e 'ether proto 0x88f7'          # L2
sudo timeout 20 tcpdump -ni enp6s0 'udp port 319 or udp port 320'   # UDPv4
```

Ya se sabe que las cámaras emiten `Delay_Req`, así que algo va a aparecer en uno
de los dos. Si es sólo UDPv4 → las cámaras tienen que salir del bridge.

**B.4 — `ptp4l-sw` (cámaras).** `enp6s0/7s0/8s0`, software timestamping,
transporte según B.3 (`L2` si siguen en el bridge).

**B.5 — Configurar el Hesai.** Por su web UI: `Clock Source: PTP`,
`PTP Profile: 1588v2`, `Transport: UDP/IP`, `Domain: 0`. Hoy no emite tráfico
PTP (`tcpdump ... ether proto 0x88f7` da 0 paquetes), lo cual es consistente con
PTP deshabilitado — y también con que esté configurado en UDP/IP, donde el
filtro correcto sería `udp port 319 or udp port 320`. Chequeá los dos.

**B.6 — Escala de tiempo (TAI vs UTC).** `ptp4l` **siempre** usa la escala PTP
(TAI) y anuncia `currentUtcOffset`. En linuxptp 4.0 **no existe** la opción
`ptp_timescale` (`unknown option ptp_timescale at line N` — verificado); lo que
se controla es `utc_offset`, cuyo default es 37.

Como en este diseño tanto `CLOCK_REALTIME` como el PHC están en **UTC**
(`phc2sys -O 0`), hay que anunciar coherentemente:

```
utc_offset   0
timeSource   0x20        # GPS
```

en **las dos** instancias. Así, un esclavo que calcula `UTC = PTP − offset`
obtiene UTC, y uno que ignora el offset también. Con el default de 37 el LiDAR
quedaría 37 s corrido respecto de las cámaras.

La alternativa estándar-correcta es poner el PHC en TAI (`phc2sys -O 37`) y
dejar `utc_offset 37` — pero entonces la instancia de software, que sirve
`CLOCK_REALTIME` y no puede desplazarse, quedaría inconsistente con la de
hardware. Por eso se elige UTC en todo.

> **Verificación:** `sudo phc_ctl enp11s0 cmp` tiene que dar microsegundos, no
> 37 s. Y en el nivel A de 2.5, ningún tópico puede aparecer con ±37 s.

### Bloque C — el radar (no tiene PTP)

El ARS430 no soporta PTP: se captura con `libpcap` y no hay reloj que
disciplinar. La única palanca es **de dónde sale el timestamp**.

Hoy, en `ars430_ros_publisher/src/radar_publisher_node.cpp` (líneas 238 y 292):

```cpp
msg.header.stamp = now();     // reloj del nodo, en el momento de parsear
```

`now()` incluye el bufferizado de libpcap y la latencia de scheduling del hilo
de captura — décimas de ms, variables con la carga. Y en la **misma función**
ya está disponible el timestamp del kernel, que se usa para el replay offline
pero no para el vivo:

```cpp
const int64_t ts = (int64_t)h->ts.tv_sec * 1000000 + h->ts.tv_usec;  // línea 157
```

`h->ts` es el timestamp de recepción tomado por el driver, del orden de µs
contra `CLOCK_REALTIME`. **Cambiar `now()` por `h->ts` mejora el timestamp del
radar en dos o tres órdenes de magnitud y es un cambio de dos líneas.** Es la
mejora con mejor relación resultado/esfuerzo de todo este documento.

Además se puede pedir timestamping por software explícito en el socket de
captura (`SO_TIMESTAMPING` con `SOF_TIMESTAMPING_RX_SOFTWARE`), pero el
`h->ts` que ya entrega libpcap alcanza para el objetivo.

> **APLICADO.** El nodo declara `use_kernel_timestamp` (por defecto `true` en
> vivo, `false` en replay de pcap para no cambiar el flujo offline) y ambos
> sitios usan `frameStamp()`. Medido después: mínimo 323–406 µs. La dispersión
> de 0,5–104 ms que queda es el *read timeout* de 100 ms de libpcap —
> `header.stamp` es correcto igual, porque `h->ts` se toma en la recepción y no
> en el parseo.

---

## 4. Orden de ejecución y criterio de "listo"

| # | Paso | Criterio de aceptación |
|---|---|---|
| 1 | Bloque A | `chronyc tracking` converge; `ppstest` con fracción cerca de `.000` |
| 2 | B.3 | Se sabe qué transporte hablan las cámaras |
| 3 | B.1 + B.2 | `phc_ctl enp11s0 cmp` en µs |
| 4 | B.5 | El Hesai emite PTP; `pmc ... GET PORT_DATA_SET` lo ve |
| 5 | B.4 | Un `Delay_Resp` en el puerto 320 dentro de 1 ms del `Delay_Req` del sensor (ver nota abajo) |
| 6 | B.6 | Ningún tópico con ±37 s |
| 7 | 2.5 nivel A | Todos los `ros2 topic delay` en el mismo orden de magnitud |
| 8 | Bloque C | Radar estampado con `h->ts` |
| 9 | 2.5 nivel C | Evento físico común coherente entre sensores |

No tiene sentido pasar al siguiente hasta que el anterior cumpla: cada eslabón
roto se disfraza de "sincronización mala" en el siguiente.

> **Corrección al paso 5.** En un grandmaster, `meanPathDelay`,
> `offsetFromMaster` y `peerMeanPathDelay` valen **siempre 0** — no tiene a
> quién medirle el camino (`stepsRemoved 0`). Pedir "path delay ≠ 0" ahí es un
> criterio equivocado. Lo que hay que mirar es el cable: que a cada `Delay_Req`
> del sensor le siga un `Delay_Resp` del host en el puerto 320.

---

## 5. El Hesai en `PTP: Free Run` — estado y rodeo

**Sin resolver.** Documentado acá para no volver a recorrer el mismo camino.

### Lo que está verificado del lado del host

| Comprobación | Resultado |
|---|---|
| `ptp4l-hw` estable | 20 min de *uptime*, 0 reinicios |
| Announce / Sync / Follow_Up | cada 2 s, sin huecos (tcpdump) |
| `Delay_Req` del LiDAR | contestado en ~50 µs |
| `gm.ClockClass` | 6 (reloj atómico/GNSS trazable) |
| `gm.ClockAccuracy` | `0x21` (±100 ns) |
| `gm.OffsetScaledLogVariance` | `0x4e5d` |
| PHC ↔ `CLOCK_REALTIME` | −285 ns |

Y del lado del LiDAR: *Clock Source* PTP, 1588v2, dominio 0, UDP/IP,
`logAnnounce 1`, `logSync 1`, `logMinDelayReq 0`, umbral de enganche en su
máximo (100 µs). Aun así: **`PTP Free Run`**.

Los únicos campos que quedan sin poder poner en 1 son `currentUtcOffsetValid`
y `timeTraceable`. Medido: ni `utc_offset` ni `leapfile` los cambian en
linuxptp 4.0. Si el LiDAR los exige para enganchar, no hay palanca desde acá y
la vía es reclamarle a Hesai con esta evidencia.

### Qué provoca

`/lidar_points` con `header.stamp` a ~2,5–3,3 s del reloj del host, y el mínimo
oscilando entre 0,8 y 4,4 s. **No es backlog** — dos medidas lo descartan: el
modo de paquetes crudos (que no parsea) da lo mismo, y la tasa de entrada
iguala a la de salida (9,8 Hz → 9,8 Hz), así que no hay cola creciendo. Lo que
se mueve son los timestamps: el driver los reconstruye del reloj libre del
sensor.

### El rodeo: `use_timestamp_type: 1`

El SDK tiene una segunda fuente de timestamp, y la elige un parámetro del YAML:

```
libhesai/UdpParser/include/udp6_1_parser.h:259     (6_1 = parser del XT32M2X)
  if (use_timestamp_type == 0)
    sensor_timestamp = pTail->GetMicroLidarTimeU64(...);   // reloj del LiDAR
  else
    sensor_timestamp = udpPacket.recv_timestamp;           // reloj del host

libhesai/Lidar/lidar.h:776
  udp_packet.recv_timestamp = GetMicroTimeU64();

libhesai/Common/src/plat_utils.cc:127-144
  GetMicroTimeU64()  ->  gettimeofday()  ->  CLOCK_REALTIME
```

`CLOCK_REALTIME` es el reloj que chrony disciplina con el PPS (~3 µs RMS) y el
mismo que lee `sync_check`. Y de `sensor_timestamp` sale `frame_start_timestamp`,
que es el `header.stamp` **tanto de `/lidar_points` como de `/lidar_packets`**
(`source_driver_ros2.hpp`, `hesai_lidar_sdk.hpp:327`) — así que arregla los dos.

Se entrega como `HesaiLidar_ROS_2.0/config/config_recv_timestamp.yaml`,
idéntico a `config.yaml` salvo ese parámetro:

```bash
ros2 launch eod_av_launch all_sensors.launch.py \
    lidar_config:=config_recv_timestamp.yaml
```

**Es un rodeo, no un arreglo.** El timestamp deja de marcar el barrido y pasa a
marcar la recepción en el host: se le suma la latencia de red más el retorno
del `recv()`, decenas de µs, constante. Un sesgo fijo y chico sirve para un
dataset; un reloj que corre libre, no. El día que el LiDAR enganche
(`PTP Locked` en su web UI), volver a `config.yaml`.

---

## 6. El GNSS: por qué `use_gps_time` va en `true`

### El receptor va a 10 Hz, no a 1 Hz

Verificado en el cable con `gpspipe -r`:

```
$GNGGA,174522.60,...   $GNGGA,174522.70,...   $GNGGA,174522.80,...
$GNGGA,174522.90,...   $GNGGA,174523.00,...   $GNGGA,174523.10,...
```

Una época cada 0,1 s, dos decimales, clavadas en el borde de décima, y la
posición cambia en cada una. Son **diez fixes distintos por segundo**. La misma
cadencia sale de gpsd (`gpspipe -w`: `.800 .900 .000 .100 .200 …`).

Consecuencia directa: **`publish_rate` tiene que ser ≥ 10**. Con 1 se tiran
nueve de cada diez posiciones reales. Y `gpsd_client` hace
`while (waiting(0)) p = read();` quedándose con la última lectura, así que las
épocas del medio se pierden sin aviso — conviene margen, no ajuste justo.

### La trampa de leer `sync_check` al revés

Con `use_gps_time: true`, `/fix` mide 475 ms de media. Con `false`, 150 µs.
El segundo número es más lindo y es **peor**:

| | `true` | `false` |
|---|---|---|
| `header.stamp` | época GNSS real de la solución | hora de llegada al host |
| media en `sync_check` | ~475 ms | ~150 µs |
| qué es esa media | latencia de entrega, **medida y visible** | `now − now`, no mide nada |
| error dentro del dato | ninguno | los ~475 ms, **escondidos** |

Con `false` la latencia no desaparece: se mete adentro del timestamp y deja de
verse. Para un bag eso es exactamente lo que no se quiere.

Y `/extended_fix` **no sirve como referencia del receptor**: `parseGpsFix`
(`gpsd_parser_base.cpp:106-107`) usa el reloj del host siempre, ni mira
`use_gps_time`. Sus ~200 µs miden el transporte de ROS, nada más.

### Diferencia con el caso del LiDAR

Parecen el mismo problema y no lo son. La pregunta en los dos casos es *¿el
reloj que puso el sello es confiable?*:

- **LiDAR:** no. Está en `Free Run`, corre con su cristal y se separa. Por eso
  se lo cambia por el del host.
- **GNSS:** sí. Épocas en el borde de décima y PPS entrando a `.000008`. No hay
  nada que cambiar.

### El enlace serie NO está saturado (hipótesis descartada)

Al ver sentencias faltantes en el NMEA supuse saturación del puerto. **Medido,
es falso.** El chequeo del nivel 1 de `diagnose_ptp.sh` da:

```
sentencias: GNGGA 111, GNRMC 106, GNTHS 100, GNHPR 89,
            GPGSV 48, GNGSA 44, GBGSV 32, GAGSV 24, GLGSV 22, GNGST 11, GNVTG 10
epocas: 111 en 11.30s -> 10.0 Hz (periodo modal 0.10s)
3 epocas faltantes de 111
ancho de banda: 33624 bit/s necesarios sobre 115200 configurados (29%)
```

29 %, no 104 %. Y la pérdida es de 3 épocas sobre 111 (2,7 %), no el patrón
sistemático que había supuesto. Sobra ancho de banda: el cuello de botella está
en otro lado.

### La latencia de ~400 ms: qué está probado y qué no

**Probado — no es de ROS.** Comparando la época que reporta gpsd contra el
reloj del host, sin pasar por ROS, gpsd ya la entrega envejecida 175–450 ms:

```bash
gpspipe -w -n 80 | python3 -c "
import sys, json, time, datetime
for line in sys.stdin:
    try: d = json.loads(line)
    except Exception: continue
    if d.get('class') != 'TPV' or not d.get('time'): continue
    t = datetime.datetime.fromisoformat(d['time'].replace('Z','+00:00')).timestamp()
    print('%8.1f ms' % ((time.time()-t)*1000))
"
```

**Probado — la cola está en el cable.** Los saltos de latencia entre épocas
consecutivas son todos múltiplos **enteros** de 5,26 ms:

```
saltos:  -52.7  +42.1  -5.3  -5.2  -5.3  +47.3  -5.3  +94.8  +47.3 ...
/5.26:   -10    +8     -1    -1    -1    +9     -1    +18    +9
```

5,26 ms a 115200 baud 8N1 son 61 caracteres: **una sentencia NMEA**. La
latencia se mueve de a sentencias enteras, así que 400 ms son ~4600 bytes
encolados esperando el cable.

**Probado — tampoco es el cable.** Se sospechó que el presupuesto de 27 %
subestimaba, porque `gpspipe -r` solo muestra el NMEA que gpsd reenvía y no el
binario que consume en silencio (RTCM3 del enlace RTK, UBX). Medido con
`gpspipe -R` (todos los bytes tal como llegan del dispositivo):

```
solo el NMEA reenviado:  31519 bit/s de 115200 (27%)
carga REAL del cable:    31907 bit/s de 115200 (28%)
```

Un punto porcentual de diferencia: **no hay tráfico oculto**, y el puerto está
al 28 %. El cable no es.

**Conclusión: el retardo se genera dentro del receptor.** El argumento decisivo
ya estaba en los datos:

| | cuándo llega | precisión |
|---|---|---|
| PPS del mismo receptor | en el borde de segundo | **3,1 µs RMS** (chrony) |
| NMEA de esa misma época | 175–450 ms después | — |

El receptor sabe perfectamente qué hora es — su PPS lo demuestra con precisión
de microsegundos. Lo que tarda es en **calcular y emitir la sentencia**. Es
coherente con lo que es: un simpleRTK3B **Compass**, de doble antena, que además
del RTK resuelve rumbo (de ahí `GNTHS` y `GNHPR`). Esa solución necesita datos
de las dos antenas y cuesta tiempo.

No hay palanca desde el host, y no hace falta: el `header.stamp` es la época
real de la solución. Es latencia de entrega, no error de sello. Para un bag
grabado da exactamente lo mismo que el dato llegue 400 ms tarde mientras esté
correctamente atribuido al instante en que la posición fue válida.

> Las 2 épocas perdidas de 117 (1,7 %) son del mismo origen: alguna solución
> que no llegó a tiempo. No es descarte por congestión.

### Por qué el refclock de chrony apunta a `/dev/ppsN` y no a un symlink

No se puede fijar con udev. El kernel registra el dispositivo PPS de la línea
de disciplina **sin padre** (`drivers/pps/clients/pps-ldisc.c`):

```c
info.dev = NULL;
pps = pps_register_source(&info, ...);
```

Como `ATTRS{}` sube por la cadena de padres y aquí no hay ninguna, una regla
`SUBSYSTEM=="pps", ATTRS{serial}=="..."` **nunca** matchea. Lo único que lleva
el dispositivo es `/sys/class/pps/ppsN/name` = `usbserial<n>`, atado al número
de `ttyUSB` y por lo tanto igual de inestable.

Consecuencia operativa: **si se desenchufa y vuelve a enchufar el USB del GNSS,
hay que volver a correr `setup_ptp_sync.sh`.** El paso `[6/9]` detecta el
`/dev/ppsN` nuevo y corrige la línea de `chrony.conf` solo. El resto de los
sensores (cámaras, LiDAR, radar) no necesita nada: alcanza con reconectarlos.
