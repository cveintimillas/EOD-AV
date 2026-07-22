#!/usr/bin/env python3
"""Live statistics for ars430_ros_publisher RadarPacket topics (ROS2 version).

Prints a summary every few seconds so live data can be compared against the
known-good reference values.

Reference (WATonomous outdoor bags, decoded detections):
    dets/cycle ~46 | SNR med ~11.7 dB | RCS med ~+8.7 dBsm | moving 6-8%

Usage:
    ros2 run ars430_ros_publisher radar_stats.py -t /unfiltered_radar_packet_1
"""

import argparse
import math
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ars430_ros_publisher.msg import RadarPacket

EVENT_NAMES = {2: 'FAR', 4: 'NEAR'}


def percentile(sorted_vals, pct):
    if not sorted_vals:
        return float('nan')
    k = (len(sorted_vals) - 1) * pct / 100.0
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] * (1.0 - (k - lo)) + sorted_vals[hi] * (k - lo)


class RadarStats(Node):
    def __init__(self, topic, window):
        super().__init__('radar_stats')
        self.window = window
        self.lock = threading.Lock()
        self.reset()
        self.create_subscription(RadarPacket, topic, self.callback, 50)
        self.create_timer(window, self.report)
        print('radar_stats: listening on %s, reporting every %.0f s' % (topic, window))

    def reset(self):
        self.packets = 0
        self.events = {}
        self.snr = []
        self.rcs = []
        self.vrel = []
        self.rng = []
        self.az = []

    def callback(self, msg):
        with self.lock:
            self.packets += 1
            self.events[msg.event_id] = self.events.get(msg.event_id, 0) + 1
            for d in msg.detections:
                self.snr.append(d.snr)
                self.rcs.append(d.rcs)
                self.vrel.append(abs(d.vrel_rad))
                self.rng.append(d.range)
                self.az.append(math.degrees(d.az_ang))

    def report(self):
        with self.lock:
            packets = self.packets
            events = dict(self.events)
            snr = sorted(self.snr)
            rcs = sorted(self.rcs)
            vrel = self.vrel
            rng = sorted(self.rng)
            az = sorted(self.az)
            self.reset()

        print('\n=============== RADAR STATS (last %.0f s) ===============' % self.window)
        if packets == 0:
            print('  NO PACKETS RECEIVED on this topic!')
            print('  -> check: topic name, publisher running, iface/port, setcap')
            print('==========================================================')
            sys.stdout.flush()
            return
        n = len(snr)
        ev = ' '.join('%s:%d' % (EVENT_NAMES.get(k, 'EV%d' % k), v) for k, v in sorted(events.items()))
        print('  packets: %d (%.1f pkt/s) | events: %s' % (packets, packets / self.window, ev))
        print('  detections: %d | %.1f per packet' % (n, float(n) / packets))
        if n:
            moving = sum(1 for v in vrel if v > 0.3)
            print('  range [m]: min=%.1f med=%.1f max=%.1f | az [deg]: %.1f..%.1f'
                  % (rng[0], percentile(rng, 50), rng[-1], az[0], az[-1]))
            print('  SNR [dB]: med=%.1f p75=%.1f | RCS [dBsm]: med=%.1f'
                  % (percentile(snr, 50), percentile(snr, 75), percentile(rcs, 50)))
            print('  moving (|vrel|>0.3 m/s): %d (%.1f%%)' % (moving, 100.0 * moving / n))
        print('==========================================================')
        sys.stdout.flush()


def main():
    argv = remove_ros_args(sys.argv)
    ap = argparse.ArgumentParser(description='RadarPacket topic statistics')
    ap.add_argument('-t', '--topic', default='/unfiltered_radar_packet_1')
    ap.add_argument('-w', '--window', type=float, default=5.0)
    args = ap.parse_args(argv[1:])

    rclpy.init()
    node = RadarStats(args.topic, args.window)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
