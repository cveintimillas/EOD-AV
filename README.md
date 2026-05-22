# EOD-AV

This repository contains the `gps_bringup` ROS 2 package used to bring up an NMEA GPS serial driver (`nmea_navsat_driver`).

Requirements
- ROS 2 installed and sourced
- System package: `python3`, `python3-pip`
- System rosdep key for serial support: `python3-pyserial` (Debian/Ubuntu)
- Python requirements: see `gps_bringup/requirements.txt`

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
- Launch with defaults (uses `/dev/ttyUSB0` 115200):

```bash
ros2 launch gps_bringup gps.launch.py
```

- The launch file now prepares the serial port before starting the driver by setting it to `115200`, `raw`, and disabling hardware flow control (`-crtscts`). This avoids the terminal waiting forever for an RTS/CTS handshake when the GPS module only streams data continuously.

- Override serial settings:

```bash
ros2 launch gps_bringup gps.launch.py port:=/dev/ttyUSB1 baud:=9600
```

Notes
- The launch file runs the `nmea_serial_driver` node from the `nmea_navsat_driver` package and exposes `port`, `baud`, and `frame_id` as launch arguments.
- The package provides `gps_bringup/requirements.txt` for Python dependencies (`pyserial`). You can install them with:

```bash
pip3 install -r gps_bringup/requirements.txt
```

- Prefer installing Python serial via your OS package manager (`python3-pyserial`) when using `rosdep` on Debian/Ubuntu.

Hardware & GNSS heading
- This setup uses a SimpleRTK3B receiver connected to Calibrated Survey Triple-band GNSS antennas (the antennas feed the RTK3B). The SimpleRTK3B may output NMEA sentences over a serial interface but does not itself publish ROS 2 topics — the `nmea_navsat_driver` translates those serial NMEA sentences into ROS 2 topics.
- Important: multi-antenna heading (baseline/heading solution) may be provided by the RTK3B as specific NMEA sentences (or as vendor/proprietary sentences). Typical sentence identifiers that carry heading information include `PASHR`, `HDT`, `HDG`, or vendor-specific `$P...` sentences. If the RTK3B is not configured to emit heading, you will not see heading on ROS topics.

How to verify the RTK3B output (serial checks)
- Ensure your user has serial access:

```bash
sudo usermod -a -G dialout $USER
newgrp dialout   # or log out and log in again
```

- Inspect raw serial output (replace `/dev/ttyUSB0` and `115200` when needed):

```bash
# using screen
sudo apt install -y screen
sudo stty -F /dev/ttyUSB0 115200 raw -echo -echoe -echok -crtscts
screen /dev/ttyUSB0 115200

# or simple cat (useful for quick checks)
sudo stty -F /dev/ttyUSB0 115200 raw -echo -echoe -echok -crtscts
stdbuf -oL cat /dev/ttyUSB0 | sed -n '1,200p'
```

- If the port stalls or appears empty, re-run the `stty` command above first. It tells Ubuntu not to wait for RTS/CTS and to read the GPS stream in raw mode.

- Look for NMEA sentences that contain heading information (`PASHR`, `HDT`, `HDG`, `VTG`, or vendor-specific `$P...`). If you see them on the serial port, the driver can be used to surface them in ROS 2 (see next section).

How to verify in ROS 2
- After launching the driver, list topics and inspect NMEA sentences or fixes:

```bash
ros2 topic list
ros2 topic echo /fix            # NavSatFix messages
ros2 topic echo /nmea_sentence  # raw NMEA sentences (if provided by the driver)
```

- If the driver publishes raw NMEA sentences, watch for the heading sentence identifiers mentioned above. If the heading is only available in a proprietary sentence, you may need a small parser node that subscribes to the NMEA sentence topic and extracts heading into a standard ROS message (`sensor_msgs/Imu` or a custom message).

Expected topics (example output)
- `/fix` — NavSatFix messages (latitude/longitude/altitude).
- `/heading` — heading information (provided when the receiver/driver emits heading sentences).
- `/vel` — velocity information (if the receiver provides it).
- `/time_reference` — time reference messages used by some drivers.
- `/parameter_events` — ROS parameter change events (system-level).
- `/rosout` — ROS logging topic (system-level).

Troubleshooting
- No output on serial: confirm device node (`ls /dev/ttyUSB*`), check `dmesg` after plugging the device, and verify baud/port settings.
- No heading in NMEA: verify RTK3B configuration — some receivers require enabling specific message output for heading/baseline solutions.
- Permissions / access denied: make sure you are in the `dialout` group or run with appropriate permissions.


License
- MIT


