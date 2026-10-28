# One sample of the mind-pour controller's state, published from update() at
# `state_rate` Hz (see config/mind_pour.yaml). session_manager.py ends the
# session on it and mind_pour.launch records it into a rosbag.

# Controller FSM state. These values match MindPourController::ControllerState.
# WAITING: holding the start pose, no rotate command acted on yet.
# ROTATING: the trial timer is running.
# DONE: joint 7 came back to its start angle; commands are ignored from here on.
uint8 STATE_WAITING=0
uint8 STATE_ROTATING=1
uint8 STATE_DONE=2

# header.stamp is the host wall clock (ros::Time::now()) at the control tick
# the sample was taken on.
Header header

uint8 state

# The command acted on this tick (PourCommand constants): NONE once the latest
# command is older than command_hold_duration, and always NONE in DONE.
int8 command
# Sequence number of the latest received command, acted on or not.
uint32 command_seq

# Measured joint positions [rad].
float64[7] q

# Joint 7 at controller start, and the commanded joint 7 position and velocity
# [rad, rad/s].
float64 q7_start
float64 q7_desired
float64 dq7_desired

# Measured joint 7 rotation away from q7_start [rad], positive = clockwise.
float64 angle

# Trial time [s]: 0 in WAITING, running in ROTATING, the final trial duration
# in DONE.
float64 elapsed
