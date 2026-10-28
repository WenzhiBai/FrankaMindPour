# One EEG-decoded command, as received from the Windows PC over UDP and
# republished by udp_bridge.py. MindPourController acts on the latest one for
# command_hold_duration seconds after header.stamp, or until the next arrives.

int8 ANTI_CLOCKWISE=-1
int8 NONE=0
int8 CLOCKWISE=1

# header.stamp is the host wall clock (ros::Time::now()) at the moment the
# datagram was received - the controller ages the command against it.
Header header

# Sender's sequence number, incremented once per datagram.
uint32 seq

# ANTI_CLOCKWISE, NONE or CLOCKWISE. Which way joint 7 turns for CLOCKWISE is
# set by clockwise_sign in config/mind_pour.yaml.
int8 command

# Sender's own clock [s] when it sent the datagram. Not synchronised with this
# host, so only differences between packets are meaningful.
float64 sender_time
