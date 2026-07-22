#!/usr/bin/env python3
"""Offline decoder for the v2 RDI protocol (our radar, multicast 239.0.0.1:40000).

Implements the same layout as the v2 path in parser.cpp so captures can be
validated without ROS. Reads a pcap taken with:

    sudo tcpdump -i <iface> -w radar_raw.pcap 'udp port 40000'

and prints per-cycle detections plus summary statistics.

Layout (reverse-engineered July 2026, validated against a real capture):
  SOME/IP header 16 B big-endian: service 220, events 2 (far) / 4 (near)
  Payload (little-endian): alive counter @22 (6 repeats per measurement),
  timestamp u32 @31, MeasurementCounter u32 @35, detection count u16 @44,
  27-byte detection records from offset 52.
  Record: 0 u16 Range | 2 s16 VrelRad | 4 s16 AzAng0 | 6 s16 AzAng1 |
  8 s16 ElAng (far only) | 10 u16 ElAngVar | 12 s16 RCS0 | 14 s16 RCS1 |
  16 u8 Prob0 | 17 u8 Prob1 | 18 u16 RangeVar | 20 u16 SNR |
  22 u8 flags (TBD) | 23 rsvd | 24 u16 AzAngVar | 26 u8 scan type.
  Resolutions: legacy ARS430 multipliers (resMultipliers.h).
"""

import argparse
import math
import struct
import sys

R_RANGE = 0.004577776
R_VEL = 0.004577776
R_ANG = 0.0000958767
R_RCS = 0.003051851
R_VAR = 0.000152593
R_ANGVAR = 0.0000152593
R_SNR = 0.1

EVENT_NAMES = {2: 'FAR', 4: 'NEAR'}


def iter_udp_payloads(path):
    with open(path, 'rb') as f:
        gh = f.read(24)
        if len(gh) < 24:
            return
        magic = struct.unpack('<I', gh[:4])[0]
        if magic in (0xa1b2c3d4, 0xa1b23c4d):
            endian = '<'
        elif magic in (0xd4c3b2a1, 0x4d3cb2a1):
            endian = '>'
        else:
            sys.exit('Not a classic pcap file (pcapng not supported; use tcpdump -w)')
        while True:
            rh = f.read(16)
            if len(rh) < 16:
                return
            _, _, caplen, _ = struct.unpack(endian + 'IIII', rh)
            frame = f.read(caplen)
            if len(frame) < caplen:
                return
            if len(frame) < 34 or frame[12:14] != b'\x08\x00':
                continue
            ihl = (frame[14] & 0x0F) * 4
            if frame[23] != 17:  # UDP
                continue
            frag = struct.unpack('!H', frame[20:22])[0] & 0x1FFF
            if frag:
                continue
            yield frame[14 + ihl:]


def decode_v2(payload):
    """payload starts at the UDP header (8 bytes), SOME/IP at offset 8."""
    p = payload[8:]
    if len(p) < 52:
        return None
    svc = (p[0] << 8) | p[1]
    ev = (p[2] << 8) | p[3]
    if svc != 220 or p[21] != 0x14 or p[43] != 0x01:
        return None
    mc = struct.unpack_from('<I', p, 35)[0]
    ts = struct.unpack_from('<I', p, 31)[0]
    alive = p[22]
    cnt = struct.unpack_from('<H', p, 44)[0]
    nvis = (len(p) - 52) // 27
    dets = []
    for k in range(min(cnt, nvis)):
        o = 52 + k * 27
        prob0, prob1 = p[o + 16], p[o + 17]
        az = struct.unpack_from('<h', p, o + 4)[0] * R_ANG if prob0 >= prob1 \
            else struct.unpack_from('<h', p, o + 6)[0] * R_ANG
        rcs = struct.unpack_from('<h', p, o + 12)[0] * R_RCS if prob0 >= prob1 \
            else struct.unpack_from('<h', p, o + 14)[0] * R_RCS
        dets.append({
            'range': struct.unpack_from('<H', p, o)[0] * R_RANGE,
            'vrel': struct.unpack_from('<h', p, o + 2)[0] * R_VEL,
            'az': az,
            'el': struct.unpack_from('<h', p, o + 8)[0] * R_ANG,
            'rcs': rcs,
            'snr': struct.unpack_from('<H', p, o + 20)[0] * R_SNR,
            'flags': p[o + 22],
        })
    return {'event': ev, 'mc': mc, 'ts': ts, 'alive': alive, 'count': cnt, 'dets': dets}


def main():
    ap = argparse.ArgumentParser(description='Decode v2 RDI pcap capture')
    ap.add_argument('pcap')
    ap.add_argument('-v', '--verbose', action='store_true', help='print every detection')
    args = ap.parse_args()

    seen = {}
    stats = {}
    for payload in iter_udp_payloads(args.pcap):
        d = decode_v2(payload)
        if d is None:
            continue
        key = (d['event'], d['mc'])
        if key in seen:
            continue
        seen[key] = True
        name = EVENT_NAMES.get(d['event'], 'EV%d' % d['event'])
        st = stats.setdefault(name, {'cycles': 0, 'dets': []})
        st['cycles'] += 1
        st['dets'].extend(d['dets'])
        if args.verbose:
            print('%s MC=%d ts=%d count=%d' % (name, d['mc'], d['ts'], d['count']))
            for det in d['dets']:
                print('   r=%6.2fm v=%+6.2fm/s az=%+6.1fdeg el=%+5.1fdeg rcs=%+6.1f snr=%5.1f flags=0x%02x'
                      % (det['range'], det['vrel'], math.degrees(det['az']),
                         math.degrees(det['el']), det['rcs'], det['snr'], det['flags']))

    for name, st in sorted(stats.items()):
        dets = st['dets']
        if not dets:
            continue
        rngs = sorted(d['range'] for d in dets)
        vels = sorted(abs(d['vrel']) for d in dets)
        azs = sorted(math.degrees(d['az']) for d in dets)
        snrs = sorted(d['snr'] for d in dets)
        moving = sum(1 for d in dets if abs(d['vrel']) > 0.3)
        n = len(dets)
        print('\n=== %s: %d detections in %d cycles (%.1f/cycle) ===' % (name, n, st['cycles'], n / st['cycles']))
        print('  range [m]: min=%.2f med=%.2f max=%.2f' % (rngs[0], rngs[n // 2], rngs[-1]))
        print('  az  [deg]: min=%.1f med=%.1f max=%.1f' % (azs[0], azs[n // 2], azs[-1]))
        print('  SNR  [dB]: med=%.1f max=%.1f' % (snrs[n // 2], snrs[-1]))
        print('  |vrel| med=%.2f max=%.2f m/s | moving(>0.3m/s): %d (%.1f%%)'
              % (vels[n // 2], vels[-1], moving, 100.0 * moving / n))


if __name__ == '__main__':
    main()
