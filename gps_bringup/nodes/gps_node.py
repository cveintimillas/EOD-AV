#!/usr/bin/env python3
import serial
import time
import subprocess

# Configuración del puerto GPS
port = "/dev/ttyUSB0"
baud = 115200

# Abrimos el puerto sin tocar DTR (dsrdtr=False)
try:
    ser = serial.Serial(port, baudrate=baud, timeout=2, dsrdtr=False)
    time.sleep(0.1)
    ser.close()
except Exception as e:
    print(f"Error abriendo el puerto {port}: {e}")

# Lanzamos el driver ROS 2
subprocess.run([
    "ros2", "run", "nmea_navsat_driver", "nmea_serial_driver",
    "--ros-args",
    "-r", "__node:=nmea_serial_driver",
    "-p", f"port:={port}",
    "-p", f"baud:={baud}",
    "-p", "frame_id:=gps_link"
])
