#!/usr/bin/env python3
"""UDP command sender for the Windows EEG PC. Standard library only.

Copy this file to the Windows PC. In the EEG pipeline, use CommandSender:

    from fmp_udp_sender import CommandSender, CLOCKWISE, NONE, ANTI_CLOCKWISE
    sender = CommandSender('192.168.50.2', 5005)
    sender.send(CLOCKWISE)  # once per classifier output, ~50 Hz

Run directly, it plays a scripted command pattern for testing the link and a
whole session without EEG (works from either PC):

    python fmp_udp_sender.py --host 192.168.50.2
    python fmp_udp_sender.py --pattern "0:1,1:2,0:1,-1:2,0:2"

Datagram format (ASCII): "<seq>,<command>,<sender_time>\\n".
"""

import argparse
import socket
import time

CLOCKWISE = 1
NONE = 0
ANTI_CLOCKWISE = -1


class CommandSender:
    """Sends one command per call. Not thread-safe; use one per thread."""

    def __init__(self, host, port=5005):
        self._address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0

    def send(self, command):
        if command not in (ANTI_CLOCKWISE, NONE, CLOCKWISE):
            raise ValueError('command must be -1, 0 or 1, got %r' % command)
        self._seq += 1
        packet = '%d,%d,%.6f\n' % (self._seq, command, time.time())
        self._socket.sendto(packet.encode('ascii'), self._address)


def parse_pattern(text):
    """'1:2,0:1' -> [(1, 2.0), (0, 1.0)]: command, then how long to send it [s]."""
    steps = []
    for item in text.split(','):
        command, duration = item.split(':')
        steps.append((int(command), float(duration)))
    return steps


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='192.168.50.2', help='Ubuntu PC address')
    parser.add_argument('--port', type=int, default=5005)
    parser.add_argument('--rate', type=float, default=50.0, help='packets per second')
    parser.add_argument('--pattern', default='0:1,1:2,0:1,-1:2,0:2',
                        help='comma-separated command:seconds steps; the default turns '
                             'clockwise for 2 s, then back for 2 s, which ends a session')
    args = parser.parse_args()

    sender = CommandSender(args.host, args.port)
    period = 1.0 / args.rate
    # Scheduled against absolute deadlines so sleep overshoot does not drift.
    # Windows' sleep resolution is coarse on Python < 3.11 (~15 ms), so expect
    # jitter there; the receiver's 30 ms command hold absorbs it.
    next_time = time.perf_counter()
    for command, duration in parse_pattern(args.pattern):
        print('sending %+d for %.1f s' % (command, duration))
        end_time = next_time + duration
        while next_time < end_time:
            sender.send(command)
            next_time += period
            delay = next_time - time.perf_counter()
            if delay > 0:
                time.sleep(delay)


if __name__ == '__main__':
    main()
