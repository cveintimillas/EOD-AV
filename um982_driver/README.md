# um982_driver

Publishes GNSS **velocity** (`/gnss/velocity`) and dual-antenna **heading**
(`/gnss/heading`) from a Unicore UM982 (ArduSimple simpleRTK3B Compass), by
parsing its proprietary `PVTSLNA`/`BESTNAVA`/`HPR` logs via the
[`um982-driver`](https://pypi.org/project/um982-driver/) PyPI package
(GPL-3.0).

This package does **not** publish position (`NavSatFix`). `gps_bringup`'s
`gpsd_client` keeps owning `/fix`, with PPS-disciplined timestamps for the
PTP sync pipeline (`setup/setup_ptp_sync.sh`) -- see "Why not one node" below.

## Scope

This covers instrumentation/config only (T1: node, T2: receiver config). It
does **not** implement RTCM/NTRIP corrections or PPK post-processing --
that architectural decision is explicitly out of scope here and unresolved.

## Why not one node that also does /fix (gpsd conflict)

`gps_bringup`'s README documents that `gpsd` exclusively owns a single
serial adapter (`/dev/gps_pps`, pinned by udev to a USB serial number) that
carries **both** NMEA and PPS for chrony's GPS+PPS refclock -- the whole PTP
architecture depends on this. A previous two-adapter attempt caused a
spurious ~367ms PPS offset, which is why it's now single-adapter only.

`um982-driver`'s `UM982Serial` opens a serial port directly. If this node
opened `/dev/gps_pps` too, it would race `gpsd` for the same tty -- the
exact failure class already hit once. So this node requires **its own,
separate** serial link to the UM982 (e.g. the Compass board's native USB
passthrough port, currently unused since NMEA+PPS moved to the single FTDI
adapter) -- never `/dev/gps_pps`.

Given that, and that this node's timestamps are software time (see below,
not PPS-disciplined), `/gnss/fix` was deliberately dropped from scope:
`gpsd_client`'s `/fix` remains the position source. This node adds only what
gpsd cannot parse -- velocity and heading.

## Confirmed on-bench (previously placeholders)

- **Physical serial device** for the second UM982 link: the Compass board's
  native USB passthrough port, an FTDI FT230X (`ID_SERIAL_SHORT=D30HIC4U`,
  distinct from the FT232R used by `/dev/gps_pps`). Pinned by
  `udev/99-eodav-um982-heading.rules.template` (already filled in, drop the
  `.template` suffix into `/etc/udev/rules.d/` and
  `udevadm control --reload-rules && udevadm trigger` to activate on a new
  machine).
- **UM982-internal COM port**: `COM3`. Configured via Unicore's **UPrecise**
  GUI tool (not this repo's `configure_um982` script) to log
  `PVTSLNA`/`BESTNAVA`/`HPR` at the already-decided 10 Hz. Confirmed present
  on the wire by directly sniffing `/dev/um982_heading`, alongside the
  receiver's default NMEA suite (`GNGGA`/`GNRMC`/`GNGSA`/etc.) and
  `RTKSTATUSA`/`UNIHEADINGA` -- all coexist fine on the same port since
  `UM982Serial.read_frame` only reacts to the three sentence types it knows
  and silently ignores the rest.
- **Serial baud for the dedicated link is 115200, not 230400.** The repo's
  earlier "already decided: 230400" below was never actually verified against
  hardware and turned out wrong -- confirmed by direct capture on the bench.
  Fixed in this repo's defaults (`heading_node.py`'s `baud` parameter and the
  launch file). `configure_um982` remains available (and untested against
  this receiver) if NVM ever needs reprovisioning from scratch, e.g. after a
  factory reset -- pass `--connect-baud 115200` if you use it.
- `gps_bringup/launch/gps.launch.py` still launches this node with
  `enable_um982_heading:=false` by default pending T3 field verification
  (see below) -- flip it once `/gnss/velocity` and `/gnss/heading` look sane.

Already decided (do not re-ask):

- Message rate: **10 Hz** for PVTSLNA/BESTNAVA/HPR.
- Serial baud for the dedicated link: **115200** (see confirmation above).
- `configure_um982` **SAVECONFIGs by default** (one-time NVM provisioning);
  pass `--no-save` to configure for the current power cycle only.

## T1 -- Node

Run standalone:

```bash
ros2 launch um982_driver um982_heading.launch.py port:=/dev/um982_heading baud:=115200
```

Or via `gps_bringup` (recommended, keeps one GPS TF frame):

```bash
ros2 launch gps_bringup gps.launch.py enable_um982_heading:=true
```

Topics:

- `/gnss/velocity` (`geometry_msgs/TwistWithCovarianceStamped`) -- ENU
  linear velocity from `BESTNAVA`. Angular part is unmeasured; set to a
  large covariance (Twist has no formal "unavailable" marker, unlike Imu).
- `/gnss/heading` (`sensor_msgs/Imu`) -- orientation-only quaternion built
  from `HPR`'s heading/pitch/roll (the library exposes these as raw degrees,
  not a quaternion -- converted here). `angular_velocity_covariance[0]` and
  `linear_acceleration_covariance[0]` are `-1` (not available, per the
  `sensor_msgs/Imu` convention) since the UM982 doesn't measure either.
  `orientation_covariance[0]` is also `-1`: `um982-driver` doesn't expose a
  per-sample heading/pitch/roll accuracy, so a number was not invented for it.
  **Requires both antennas connected** -- no heading without a baseline fix.

Known limitations:

- **Timestamping is software time** (`self.get_clock().now()` when the
  already-parsed sample is read), not hardware/PTP time. This is a known
  limitation for T1's scope; integrating with the PTP/PPS work
  (`setup/setup_ptp_sync.sh`) is explicit future work, not solved here.
- **Pitch/roll sign convention from `HPR` was not verified** against the
  Unicore manual or hardware -- only degrees-to-radians conversion is
  applied. Confirm during T3 field verification by physically tilting the
  antenna baseline and checking the sign matches expectation. Heading (yaw)
  *was* converted from true-north-clockwise (compass) to ENU
  (east-counterclockwise) per REP-103.
- `um982-driver`'s `UM982Serial.__init__` reads exactly 10 lines and assumes
  a `#PVTSLNA` line appeared among them; if not, it raises `TypeError`
  before the node starts. Since it's used unmodified (GPL-3.0, not
  vendored), the node retries construction with backoff instead of patching
  the library.

Install the pip dependency (not resolved automatically by `colcon build`
for `ament_python` packages -- `install_requires` isn't installed the way a
plain `pip install .` would):

```bash
pip install um982-driver
```

## T2 -- Receiver configuration

**Status: done, but via Unicore's UPrecise GUI, not the command below.** The
receiver already logs `PVTSLNA`/`BESTNAVA`/`HPR` on `COM3` at 10 Hz, saved to
NVM. `configure_um982` was written for this step but hasn't actually been
exercised against this receiver -- keep it working (don't let it bit-rot) since
it's the only documented way to redo this from scratch (e.g. after a factory
reset), but treat it as unverified until someone runs it for real.

```bash
ros2 run um982_driver configure_um982 \
    --connect-port /dev/um982_heading --connect-baud 115200 \
    --target-com COM3 --rate-hz 10
```

Sends (period = 1/rate, e.g. `0.1` for 10 Hz):

```
PVTSLNA COM3 0.1
BESTNAVA COM3 0.1
HPR COM3 0.1
SAVECONFIG
```

Idempotent: re-issuing the same log command on the same target/rate is a
no-op on the receiver (re-asserts the same log, doesn't duplicate it), so
running this again is safe. Pass `--no-save` to skip `SAVECONFIG`.

## T3 -- Field verification (checklist, not automated here)

```bash
ros2 topic echo /gnss/velocity
ros2 topic echo /gnss/heading
```

- Confirm heading changes coherently while physically rotating the antenna
  assembly.
- Confirm velocity is the same order of magnitude as before (no fix topic
  to cross-check against here -- compare against `gpsd_client`'s `/fix`
  movement instead).

## License

`um982-driver` is GPL-3.0 and is used strictly as an unmodified runtime pip
dependency (see `package.xml`), not vendored into this repo, to avoid
inheriting copyleft into EOD-AV's own code.
