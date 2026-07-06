# setup/ — PTP time synchronization provisioning

This folder provisions IEEE 1588 PTP (Precision Time Protocol) on the host PC so the 3 LUCID Triton cameras, the Hesai LiDAR, and the GNSS receiver all timestamp their data off one shared, disciplined clock.

## Why this matters for this project

EOD-AV exists to record a fused, multi-sensor dataset (3x camera + LiDAR + GNSS/RTK). "Fused" means every sensor's timestamps have to be directly comparable — a downstream consumer lines up a camera frame with a LiDAR scan and a GNSS fix purely by their `header.stamp` values, with no separate calibration step to correct for clock drift after the fact.

Without PTP, each device free-runs on its own independent clock:
- The 3 cameras and the LiDAR each sit behind their own port on the host's multiport NIC, and each port has its own independent hardware clock (PHC) — they do **not** share a clock with each other by default, even though they're plugged into the same physical card.
- Clocks drift at different rates. Over a multi-minute recording session that drift accumulates into timestamps that are visibly wrong relative to each other — enough to break frame-to-scan alignment for anything that needs millisecond-or-better precision.
- The GNSS/RTK receiver's position fixes are timestamped on GPS time, which has no relationship at all to the host's free-running system clock unless something explicitly ties the two together.

`setup_ptp_sync.sh` is what ties all of this to one clock. It's the host-side half of the PTP work; the other half is in the ROS drivers themselves ([`arena_camera_node`](../arena_camera_node) sets `PtpEnable`/`PtpSlaveOnly` on each camera, `hesai_ros_driver` exposes a `/lidar_ptp` diagnostics topic) — see the root [README.md](../README.md#time-synchronization-ptp). Both halves are required: the script makes the host a PTP grandmaster the sensors can sync to; the driver changes make the sensors actually try to sync to it.

## Architecture this script builds

```
simpleRTK3B GNSS (PPS + NMEA via USB-serial)
        │
        ▼
   gpsd  →  chrony disciplines CLOCK_REALTIME (the host's system clock)
        │
        ▼
   4x phc2sys (one per NIC port, all sourced from CLOCK_REALTIME)
        │
        ▼
   4 independent hardware clocks (PHCs), one per port — now mutually aligned
        │
        ▼
   ptp4l (serverOnly, E2E, domain 0) on all 4 ports ── PTP ──▶ 3x Triton cameras + Hesai LiDAR (PTP slaves)
```

The reason there are 4 separate `phc2sys` instances instead of one: the host's multiport NIC gives each port its **own independent PHC** — they don't share a clock with each other natively. Running all 4 `phc2sys` instances off the same source (`CLOCK_REALTIME`) is what keeps all 4 ports mutually aligned, not just each one individually aligned to GPS. `ptp4l` itself runs with `serverOnly 1` on all 4 ports so it never touches a PHC directly — it only speaks the PTP protocol (Announce/Sync/Delay-Req) to the cameras/LiDAR; `phc2sys` does the actual clock discipline. This split is required because the Triton cameras only support the E2E delay mechanism, which fixes the delay mechanism for the whole domain.

## What `setup_ptp_sync.sh` does

Run as `sudo bash setup_ptp_sync.sh`. It performs 7 steps:

1. **Installs packages**: `linuxptp` (`ptp4l`, `phc2sys`), `chrony`, `pps-tools`, `setserial`, `gpsd`, `gpsd-clients`.
2. **Checks hardware timestamping** on each configured interface (`ethtool -T <iface>`) — confirms each port exposes a PTP Hardware Clock before trying to use it.
3. **Creates a systemd service** (`gps-pps-ldattach.service`) that attaches the `pps-ldisc` line discipline to the GNSS's USB-serial adapter, exposing its PPS pulse as `/dev/pps0`.
4. **Configures `gpsd`** (`/etc/default/gpsd`) to read NMEA + PPS from that same serial device.
5. **Adds refclocks to chrony** (`/etc/chrony/chrony.conf`): an NMEA refclock (coarse time) and a PPS refclock (precise edge, `prefer`red) — this is what disciplines `CLOCK_REALTIME` to GPS time.
6. **Writes `/etc/linuxptp/ptp4l.conf`**: one `[global]` block (`domainNumber 0`, `delay_mechanism E2E`, `network_transport UDPv4`, `serverOnly 1`) plus one section per interface (the 3 camera ports + the LiDAR port).
7. **Creates and enables systemd services**: `ptp4l-eodav.service` (one `ptp4l` process covering all 4 interfaces) and a templated `phc2sys@.service` (instantiated once per interface, e.g. `phc2sys@enp3s0.service`) — both set to restart automatically and start on boot.

All of this is meant to run **once**, independently of ROS — it installs permanent systemd services, not something tied to a `ros2 launch` session. Once installed, PTP keeps running in the background at all times; you launch the ROS sensor nodes on top of it whenever you want to record.

## Before running

Edit the `CONFIGURACIÓN` block at the top of the script:

| Variable | Meaning | Status |
|---|---|---|
| `CAM1_IFACE` / `CAM2_IFACE` / `CAM3_IFACE` | Linux interface names for the 3 Triton cameras | Defaulted to `enp3s0`/`enp4s0`/`enp5s0` — **verify with `ip link show`**, interface names can differ per host/NIC. |
| `LIDAR_IFACE` | Linux interface name for the Hesai LiDAR's port | Placeholder `CAMBIAR_ESTO` — **must be set** or the script aborts immediately. Find it with `ip link show` + physical process of elimination. |
| `GPS_SERIAL_DEV` | USB-serial adapter carrying the GNSS's PPS (via DCD) + NMEA | Defaults to `/dev/ttyUSB0` — confirm against whatever the adapter actually enumerates as. |
| `PTP_DOMAIN` | PTP domain number, must match on every device | `0` — must also match the `Domain` setting configured in the Hesai LiDAR's web UI. |

Also required before this is useful end-to-end (not done by the script):
- Physically wire the GNSS's PPS output to the serial adapter's DCD pin, and its NMEA TX to the adapter's RX, then confirm the pulse arrives (`sudo modprobe pps_ldisc && sudo ldattach 18 /dev/ttyUSB0 && sudo ppstest /dev/pps0`) *before* running the script.
- Enable PTP on each Triton camera (handled by `arena_camera_node`'s `ptp_enable` parameter — on by default) and confirm each transitions `Listening → Uncalibrated → Slave`.
- Configure the Hesai LiDAR's own PTP settings via its web UI (Clock Source=`PTP`, Profile=`1588v2`, Network Transport=`UDP/IP`, Domain=`0`) — this can't be automated from this repo, the LiDAR has no remote "set PTP mode" command.

## Verification

The script prints these at the end; they're also the standard health checks going forward:

```bash
sudo ppstest /dev/pps0                      # GNSS PPS pulse is arriving
chronyc sources -v                          # chrony is using the PPS/NMEA refclocks
sudo pmc -u -b 0 'GET PARENT_DATA_SET'      # ptp4l's view of the PTP domain
sudo phc_ctl <iface> cmp                    # per-port PHC offset (run once per interface)
```

From the ROS side, `ros2 topic echo /lidar_ptp` (once `hesai_ros_driver`'s `ros_send_ptp_topic` config is active, and the LiDAR is wired + configured) reports the LiDAR's own lock offset/status without needing a separate host tool.
