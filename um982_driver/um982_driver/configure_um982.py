r"""
Idempotent provisioning script for the Unicore UM982 (T2).

Sends the PVTSLNA/BESTNAVA/HPR log-enable commands already validated in
UPrecise, then SAVECONFIGs them into the receiver's NVM by default.
Re-issuing the same log command on the same target COM/rate is a no-op on
the receiver (it just re-asserts the same log, it does not duplicate it),
so running this script twice in a row is safe.

Usage:
    ros2 run um982_driver configure_um982 --connect-port /dev/um982_heading \
        --connect-baud 115200 --target-com COM3 --rate-hz 10

NOTE: --target-com COM3 and 115200 baud were confirmed on the bench by
directly sniffing /dev/um982_heading (see README) -- both were placeholders
before that.
"""
import argparse
import sys
import time
from typing import List

import serial

_LOG_COMMANDS = ('PVTSLNA', 'BESTNAVA', 'HPR')
_RESPONSE_TIMEOUT_S = 2.0


def _send_command(ser: serial.Serial, command: str) -> str:
    ser.reset_input_buffer()
    ser.write((command + '\r\n').encode('ascii'))
    ser.flush()

    deadline = time.monotonic() + _RESPONSE_TIMEOUT_S
    lines: List[str] = []
    while time.monotonic() < deadline:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line:
            lines.append(line)
        if any('OK' in line or 'ERROR' in line for line in lines):
            break
    return '\n'.join(lines)


def configure(
    connect_port: str, connect_baud: int, target_com: str, rate_hz: float, save: bool,
) -> int:
    """
    Send the log-enable commands (and optionally SAVECONFIG) to the UM982.

    Returns 0 on success, 1 if the receiver reported an error for any command.
    """
    period_s = 1.0 / rate_hz
    commands = [f'{log} {target_com} {period_s:g}' for log in _LOG_COMMANDS]
    if save:
        commands.append('SAVECONFIG')

    with serial.Serial(connect_port, connect_baud, timeout=0.2) as ser:
        for command in commands:
            print(f'--> {command}')
            response = _send_command(ser, command)
            print(response or '(no response)')
            if 'ERROR' in response:
                print(f'UM982 rejected command: {command!r}', file=sys.stderr)
                return 1

    return 0


def main() -> None:
    """Parse CLI args and run configure(), exiting with its return code."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--connect-port', default='/dev/um982_heading',
        help='Linux serial device used to talk to the UM982 (placeholder -- confirm on bench).'
    )
    parser.add_argument('--connect-baud', type=int, default=115200)
    parser.add_argument(
        '--target-com', default='COM3',
        help='UM982-internal COM port to stream the logs out of (placeholder -- confirm on bench).'
    )
    parser.add_argument('--rate-hz', type=float, default=10.0)
    parser.add_argument(
        '--no-save', dest='save', action='store_false',
        help='Skip SAVECONFIG: leave the logs configured for this power cycle only, instead of '
             'persisting to receiver NVM (this is a one-time provisioning script by default -- '
             'see README for why SAVECONFIG was chosen as the default).'
    )
    args = parser.parse_args()

    sys.exit(configure(
        args.connect_port, args.connect_baud, args.target_com, args.rate_hz, args.save))


if __name__ == '__main__':
    main()
