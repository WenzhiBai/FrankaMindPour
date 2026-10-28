#!/usr/bin/env python3
"""Receives EEG-decoded commands from the Windows PC over UDP and republishes
them as franka_mind_pour/PourCommand.

Datagram format (ASCII, one per packet): "<seq>,<command>,<sender_time>\n"
  seq          sender sequence number, incremented once per datagram
  command      -1 anti-clockwise, 0 none, 1 clockwise
  sender_time  sender clock in seconds (not synchronised with this host)

Packets that arrive after a newer one are dropped, since only the latest
command matters to the controller. Losses are counted from gaps in seq.
"""

import socket

import rospy
from franka_mind_pour.msg import PourCommand

VALID_COMMANDS = (PourCommand.ANTI_CLOCKWISE, PourCommand.NONE, PourCommand.CLOCKWISE)


def parse(data):
    """Returns (seq, command, sender_time); raises ValueError on a bad packet."""
    fields = data.decode('ascii').strip().split(',')
    if len(fields) != 3:
        raise ValueError('expected 3 fields, got %d' % len(fields))
    seq, command, sender_time = int(fields[0]), int(fields[1]), float(fields[2])
    if command not in VALID_COMMANDS:
        raise ValueError('command must be -1, 0 or 1, got %d' % command)
    return seq, command, sender_time


def main():
    rospy.init_node('mind_pour_udp_bridge')
    bind_address = rospy.get_param('~bind_address', '0.0.0.0')
    port = rospy.get_param('~port', 5005)
    command_topic = rospy.get_param('~command_topic', '/mind_pour/command')
    seq_restart_gap = rospy.get_param('~seq_restart_gap', 100)

    publisher = rospy.Publisher(command_topic, PourCommand, queue_size=1, tcp_nodelay=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_address, port))
    # So the loop notices rospy shutdown while no packets are coming in.
    sock.settimeout(0.5)
    rospy.loginfo('udp_bridge: listening on %s:%d, publishing on %s',
                  bind_address, port, command_topic)

    last_seq = None
    sender = None
    received = lost = late = malformed = 0
    while not rospy.is_shutdown():
        try:
            data, address = sock.recvfrom(256)
        except socket.timeout:
            continue
        stamp = rospy.Time.now()

        try:
            seq, command, sender_time = parse(data)
        except ValueError as error:
            malformed += 1
            rospy.logwarn_throttle(1.0, 'udp_bridge: dropping malformed packet %r from %s: %s'
                                   % (data, address, error))
            continue

        if address != sender:
            rospy.loginfo('udp_bridge: receiving from %s:%d', address[0], address[1])
            sender = address

        if last_seq is not None:
            if seq <= last_seq:
                if last_seq - seq < seq_restart_gap:
                    late += 1
                    continue
                rospy.logwarn('udp_bridge: sequence number fell from %d to %d, '
                              'taking it as a restarted sender', last_seq, seq)
            elif seq > last_seq + 1:
                lost += seq - last_seq - 1
        last_seq = seq
        received += 1

        msg = PourCommand()
        msg.header.stamp = stamp
        msg.seq = seq
        msg.command = command
        msg.sender_time = sender_time
        publisher.publish(msg)

    rospy.loginfo('udp_bridge: %d packets published, %d lost, %d late (dropped), %d malformed',
                  received, lost, late, malformed)


if __name__ == '__main__':
    main()
