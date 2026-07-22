# setup/ — PTP time synchronization provisioning

This folder provisions IEEE 1588 PTP (Precision Time Protocol) on the host PC so the 3 LUCID Triton cameras, the Hesai LiDAR, and the GNSS receiver all timestamp their data off one shared, disciplined clock.

## Why this matters for this project

EOD-AV exists to record a fused, multi-sensor dataset (3x camera + LiDAR + GNSS/RTK). "Fused" means every sensor's timestamps have to be directly comparable — a downstream consumer lines up a camera frame with a LiDAR scan and a GNSS fix purely by their `header.stamp` values, with no separate calibration step to correct for clock drift after the fact.

Without PTP, each device free-runs on its own independent clock:
- The GNSS/RTK receiver's position fixes are timestamped on GPS time, which has no relationship at all to the host's free-running system clock unless something explicitly ties the two together.
- Clocks drift at different rates. Over a multi-minute recording session that drift accumulates into timestamps that are visibly wrong relative to each other — enough to break frame-to-scan alignment for anything that needs millisecond-or-better precision.

`setup_ptp_sync.sh` is what ties all of this to one clock. It's the host-side half of the PTP work; the other half is in the ROS drivers themselves ([`arena_camera_node`](../arena_camera_node) sets `PtpEnable`/`PtpSlaveOnly` on each camera, `hesai_ros_driver` exposes a `/lidar_ptp` diagnostics topic). Both halves are required: the script makes the host a PTP grandmaster the sensors can sync to; the driver changes make the sensors actually try to sync to it.

## Real hardware architecture (confirmed on the bench, not the original assumption)

The 3 Tritons and the Hesai turned out to sit on **different NIC hardware with different timestamping capability**, confirmed via `ethtool -i`/`lspci` after getting physical access — not the single multiport-card-with-4-independent-PHCs design assumed before that:

- **3x Triton cameras** — Realtek RTL8125 multiport card (driver `r8169`). This chipset does **not** expose a PTP Hardware Clock (PHC): **software timestamping only**.
- **Hesai LiDAR** — Intel I225-V onboard NIC (driver `igc`). This one **does** have a real PHC: **hardware timestamping + `phc2sys`**.

Because of that split, the script runs **two separate `ptp4l` instances** (one per timestamping mode) instead of one covering all 4 ports, and only **one `phc2sys`** (there's only one real PHC to discipline — the Triton ports have none to sync):

```
simpleRTK3B GNSS (PPS + NMEA, single USB-serial adapter)
        │
        ▼
   gpsd  →  chrony disciplines CLOCK_REALTIME (the host's system clock)
        │
        ▼
   phc2sys-eodav.service (CLOCK_REALTIME → the Hesai port's PHC, the only real one)
        │
        ├── ptp4l-hw.service   (Hesai port, hardware timestamping, own uds_address socket)
        └── ptp4l-sw.service   (3x Triton ports, software timestamping, own uds_address socket)
                │
                ▼
        3x Triton cameras + Hesai LiDAR (PTP slaves)
```

`ptp4l` runs with `serverOnly 1` on both instances so it never touches a PHC directly — it only speaks the PTP protocol (Announce/Sync/Delay-Req); `phc2sys` does the actual clock discipline of the Hesai's PHC. The Triton cameras only support the E2E delay mechanism, which fixes `delay_mechanism E2E` for the whole domain (both `ptp4l` instances).

**GPS: one single adapter for both NMEA and PPS.** The simpleRTK3B Compass's own native USB port is power/passthrough only (no PPS pin exposed). NMEA and PPS both come off one separate FTDI FT232R USB-serial adapter, wired TPS→DCD, TX→RX, GND→GND. `gpsd` correlates the fix (SHM 0) and the PPS pulse (SHM 1) automatically via sysfs because both signals share the same device. A two-adapter setup (NMEA on the compass's native port, PPS on a second adapter) was tried first and produced a fixed but spurious ~367ms PPS offset — `gpsd` cannot correlate PPS and fix across two different USB devices. Single-adapter is the only correct wiring.

## What `setup_ptp_sync.sh` does

Run as `sudo bash setup_ptp_sync.sh`. It performs 8 steps:

1. **Installs packages**: `linuxptp` (`ptp4l`, `phc2sys`), `chrony`, `pps-tools`, `setserial`, `gpsd`, `gpsd-clients`.
2. **Checks hardware timestamping** on each configured interface (`ethtool -T <iface>`) — expect `PTP Hardware Clock: 0` on the Hesai port and `none` on the 3 Triton ports.
3. **Installs a udev rule** pinning the GNSS's USB-serial adapter to a persistent `/dev/gps_pps` symlink by USB serial number — `ttyUSBn` numbering isn't stable across reconnects/reordering, which silently broke the services in testing.
4. **Creates `gps-pps-ldattach.service`**, a persistent service that attaches the `pps-ldisc` line discipline to `/dev/gps_pps`.
5. **Replaces the packaged `gpsd.service`/`gpsd.socket`** with a custom `gpsd-eodav.service` with an explicit `ExecStart` — the packaged unit's systemd env-var substitution for `DEVICES` (via `/etc/default/gpsd`) doesn't reliably apply in practice.
6. **Adds refclocks to chrony** (`/etc/chrony/chrony.conf`): an NMEA refclock (SHM 0, coarse time) and a PPS refclock (SHM 1, precise edge, `prefer`red) — this is what disciplines `CLOCK_REALTIME` to GPS time.
7. **Writes `/etc/linuxptp/ptp4l-hw.conf`** (Hesai, hardware timestamping) **and `/etc/linuxptp/ptp4l-sw.conf`** (3x Triton, software timestamping) — each with its own `[global]` block including a distinct `uds_address` (`/var/run/ptp4l-hw` / `/var/run/ptp4l-sw`), required because two simultaneous `ptp4l` instances would otherwise both try to use the default `/var/run/ptp4l` socket. `uds_address` must be declared inside `[global]`, before any interface section — placed after an interface section (e.g. `[enp11s0]`), `ptp4l` reads it as a port option and fails at startup with `unknown option uds_address` (exit 254).
8. **Neutralizes Ubuntu's own `linuxptp` default services** (`ptp4l.service`, `phc2sys.service`, and the `ptp4l@.service`/`phc2sys@.service` templates for all 4 interfaces) via `systemctl mask` — not just `disable`, since an `apt upgrade` of `linuxptp` can silently re-enable disabled-but-unmasked units. Left unmasked, these fight the `-eodav` services for the same PHC/socket; seen in testing as `clockcheck: clock frequency changed unexpectedly!` with the offset jumping erratically in `phc2sys-eodav.service`. Then creates and enables the real services: `ptp4l-hw.service`, `ptp4l-sw.service`, `phc2sys-eodav.service` (all `Restart=always`).

All of this is meant to run **once**, independently of ROS — it installs permanent systemd services, not something tied to a `ros2 launch` session. Once installed, PTP keeps running in the background at all times; you launch the ROS sensor nodes on top of it whenever you want to record.

## Before running

Edit the `CONFIGURACIÓN` block at the top of the script:

| Variable | Meaning | Status |
|---|---|---|
| `CAM1_IFACE` / `CAM2_IFACE` / `CAM3_IFACE` | Linux interface names for the 3 Triton cameras (Realtek RTL8125, no PHC) | Defaulted to `enp6s0`/`enp7s0`/`enp8s0` — **verify with `ip link show`**, interface names can differ per host/NIC (changed after a PC swap; matches `arena_camera_node/launch/three_cameras.launch.py`). |
| `LIDAR_IFACE` | Linux interface name for the Hesai LiDAR's port (Intel I225-V onboard, has a real PHC) | Defaulted to `enp11s0` — **verify with `ip link show`**. |
| `GPS_SERIAL` | `ID_SERIAL_SHORT` of the FTDI FT232R USB-serial adapter carrying PPS (via DCD) + NMEA | Defaulted to `A5069RR4` — confirm with `udevadm info -q property -n /dev/ttyUSBx \| grep ID_SERIAL_SHORT`. |
| `GPS_DEV` | Persistent symlink the udev rule creates for that adapter | `/dev/gps_pps` — shouldn't need changing, it's created by step 3 of the script itself. |
| `PTP_DOMAIN` | PTP domain number, must match on every device | `0` — must also match the `Domain` setting configured in the Hesai LiDAR's web UI. |

Also required before this is useful end-to-end (not done by the script):
- Physically wire the GNSS's PPS output to the FTDI adapter's DCD pin, and its NMEA TX to the adapter's RX, then confirm the pulse arrives (`sudo modprobe pps_ldisc && sudo ldattach 18 /dev/gps_pps && sudo ppstest /dev/pps0`, using the real `/dev/ppsN` reported under `/sys/class/pps/pps*/name`) *before* running the script.
- Enable PTP on each Triton camera (handled by `arena_camera_node`'s `ptp_enable` parameter — on by default) and confirm each transitions `Listening → Uncalibrated → Slave`.
- Configure the Hesai LiDAR's own PTP settings via its web UI (Clock Source=`PTP`, Profile=`1588v2`, Network Transport=`UDP/IP`, Domain=`0`) — this can't be automated from this repo, the LiDAR has no remote "set PTP mode" command.

## Verification

The script prints these at the end; they're also the standard health checks going forward:

```bash
cat /sys/class/pps/pps*/name                                    # identify which pps* corresponds to /dev/gps_pps
sudo ppstest /dev/ppsN                                           # GNSS PPS pulse is arriving (N from above)
chronyc sources -v                                               # chrony is using the PPS/NMEA refclocks
sudo pmc -u -b 0 -s /var/run/ptp4l-hw 'GET PARENT_DATA_SET'      # hw bus (Hesai)
sudo pmc -u -b 0 -s /var/run/ptp4l-sw 'GET PARENT_DATA_SET'      # sw bus (3x Triton)
sudo phc_ctl <LIDAR_IFACE> cmp                                   # the only port with a real PHC worth comparing
```

The default `pmc` socket (no `-s`, or `/var/run/ptp4l`) doesn't correspond to either running instance — always pass `-s` explicitly. Since the host is the grandmaster for both buses and there's no other grandmaster on the network, `GET PORT_DATA_SET` should show `portState MASTER`; it's normal and expected for `GET TIME_STATUS_NP` to report `gmPresent false` (that field means "I see *another* grandmaster besides myself," not "I'm unsynchronized").

From the ROS side, `ros2 topic echo /lidar_ptp` (once `hesai_ros_driver`'s `ros_send_ptp_topic` config is active, and the LiDAR is wired + configured) reports the LiDAR's own lock offset/status without needing a separate host tool.

See [`PTP_SYNC_CONTEXT.md`](PTP_SYNC_CONTEXT.md) for the full planning history (including the earlier, superseded 4-PHC/multiport assumption) and [`guia_ptp_sincronizacion.md`](guia_ptp_sincronizacion.md) for a step-by-step manual walkthrough.
