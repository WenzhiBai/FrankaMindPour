# FrankaMindPour

A ROS 1 Noetic (Ubuntu 20.04) catkin package that lets EEG decoded on a separate **Windows PC**
drive a **pouring action** on a Franka Research 3 (FR3). The Windows PC streams one of three
commands — **clockwise**, **none**, **anti-clockwise** — at about 50 Hz over UDP. This Ubuntu PC
rotates **only the last joint (joint 7)** to match, within ±90° of where the session started. It
also times how long the participant takes to pour and bring the end effector back.

## How a session runs

```
Windows EEG PC ──UDP "seq,cmd,time"──▶ udp_bridge.py ──/mind_pour/command──▶ MindPourController ──▶ FR3
                                                                                   │
                                     session_manager.py ◀──/mind_pour_controller/pour_state
```

One call to `./scripts/run_session.sh` is one session:

1. `franka_control` comes up with franka's `position_joint_trajectory_controller` running and
   `mind_pour_controller` loaded but stopped.
2. `session_manager.py` drives the arm to the **start pose** in `config/mind_pour.yaml`. It uses the
   same method as `franka_example_controllers/move_to_start.py`: one trajectory goal, timed so the
   furthest joint moves at `max_dq`.
3. It switches to `mind_pour_controller`, which holds joints 1–6 and waits (`WAITING`).
4. The **first rotate command starts the trial timer** (`ROTATING`). Each command turns joint 7 at
   `angular_velocity_deg`. A command lasts `command_hold_duration` (30 ms) after it arrives, or until
   the next command replaces it. A steady 50 Hz stream therefore gives continuous motion, `none`
   stops it at once, and a dropped packet is bridged.
5. Joint 7 stays within ±`angle_limit_deg` of its start angle, and slows down so it comes to rest on
   the limit.
6. The **timer stops** once joint 7 has been at least `min_excursion_deg` away and is back within
   `return_tolerance_deg` of its start angle, with no command active and the joint at rest (`DONE`).
   After that, commands are ignored.
7. The trial time is appended to `results.csv`, the bag is closed, and everything shuts down.

## Design choices

| Decision | Choice | Why |
| --- | --- | --- |
| Link Windows → Ubuntu | **UDP**, ASCII datagrams | Lowest latency and no head-of-line blocking: a lost packet never delays later ones, and the latest command is the only one that matters. The 30 ms hold covers a lost packet. Trivial to send from Python. |
| Packet | `"<seq>,<command>,<sender_time>\n"` | Readable in Wireshark/netcat. `seq` lets the bridge drop late packets and count losses. `sender_time` is kept for offline analysis; the two clocks are not synchronised. |
| Joint 7 control | `PositionJointInterface` controller plugin | Joints 1–6 are held exactly. The ±limit is an exact clamp on the commanded position. q7 is integrated from a velocity that is acceleration-limited, because the FR3 rejects velocity steps. `franka_control`'s `rate_limiting` also applies on top. |
| Start pose | Franka's trajectory controller, `move_to_start.py` logic | Proven, smooth and planned motion, with no new motion code to verify. |
| Sessions | One script call per session | Simple process lifetime, and a clean bag and a clean controller state for each session. Costs a few seconds to reconnect to FCI each time. |

## Network setup (once)

The two PCs share a direct Ethernet cable. On this PC that link appears to be `eno1` with
`192.168.50.2/24`; check with `ip -br addr`. The robot is on a different interface
(`192.168.1.x`).

1. On Windows, give the Ethernet adapter a static IPv4 address on the same subnet, e.g.
   `192.168.50.1`, mask `255.255.255.0`, no gateway.
2. Check the link: `ping 192.168.50.1` from Ubuntu, and `ping 192.168.50.2` from Windows.
3. If `ufw` is enabled here, allow the port: `sudo ufw allow in on eno1 to any port 5005 proto udp`.
4. Test end to end. Copy `scripts/fmp_udp_sender.py` to Windows, then:
   - on Ubuntu: `roscore` & `rosrun franka_mind_pour udp_bridge.py`, then `rostopic echo /mind_pour/command`
   - on Windows: `python fmp_udp_sender.py --host 192.168.50.2`

## Sending commands from the EEG pipeline (Windows, Python)

`scripts/fmp_udp_sender.py` uses only the standard library:

```python
from fmp_udp_sender import CommandSender, CLOCKWISE, NONE, ANTI_CLOCKWISE

sender = CommandSender('192.168.50.2', 5005)
# once per classifier output, ~50 Hz
sender.send(CLOCKWISE)
```

Run directly, it plays a scripted pattern, which is useful for a full session without EEG. By
default it turns clockwise for 2 s and back for 2 s, which completes a trial:

```bash
python fmp_udp_sender.py --host 192.168.50.2 --pattern "0:1,1:2,0:1,-1:2,0:2"
```

## Build / run

Like the sibling projects, this repo is **not** built in place. `compile_package.sh` wipes
`~/workspace/catkin_ws_fmp/`, copies `franka_mind_pour/` into it and `catkin build`s there against
`catkin_ws_franka`.

```bash
./scripts/compile_package.sh                               # from the repo root
./scripts/run_session.sh participant:=P01 session:=1       # one session
./scripts/run_session.sh --help                            # all accepted keys
./scripts/run_session.sh stage:=udp                        # debug one step, see below
```

Before running: release the FR3 brakes and put the robot into FCI/execution mode in Desk. **The arm
moves on its own** to the start pose at the beginning of every session, so keep its path clear.

## Debugging one step at a time

`stage:=` runs one step of a session on its own. `participant` and `session` are optional here
(they default to `debug` and `0`), and **only a `full` session writes a row to `results.csv`**.
Work through the steps in this order:

| Step | Command | Robot | What to look for |
| --- | --- | --- | --- |
| 1. Windows link | `./scripts/run_session.sh stage:=udp` | not used | `receiving from 192.168.50.1:<port>` when the first packet arrives, then `average rate: ~50` once per second. `no new messages` means nothing is arriving: check the cable, the IP addresses, the port and `ufw` (see Network setup). Ctrl+C prints the published / lost / late / malformed counts. |
| 2. Start pose | `./scripts/run_session.sh stage:=move` | **moves** | The arm goes to `joint_pose` and everything exits. |
| 3. Rotation | `./scripts/run_session.sh stage:=rotate` | **joint 7 moves** | No move to the start pose: the arm holds where it is, and joint 7 turns on commands. The controller prints state, command, angle and velocity every 0.5 s. It ends when joint 7 is back at its start angle after an excursion, or on Ctrl+C. |
| 4. Everything | `./scripts/run_session.sh participant:=P01 session:=1` | **moves** | A real session. |

Steps 1, 3 and 4 take commands from the Windows PC or from a simulated sender on this PC, started
in a second terminal:

```bash
python3 scripts/fmp_udp_sender.py --host 127.0.0.1 --pattern=1:1     # 1 s clockwise, about 30 deg
python3 scripts/fmp_udp_sender.py --host 127.0.0.1 --pattern=-1:1    # 1 s anti-clockwise
python3 scripts/fmp_udp_sender.py --host 127.0.0.1                   # 2 s there, 2 s back
```

Write a pattern that starts with `-1` as `--pattern=-1:1`; with a space, argparse takes it for an
option. In step 3, sending `1:1` and then `-1:1` brings joint 7 back to its start angle, which ends
the trial. This is also the step to check `clockwise_sign` on.

To see the raw messages in any step: `rostopic echo /mind_pour/command` and
`rostopic echo /mind_pour_controller/pour_state`. Steps 2 and 3 record a bag like a session does,
unless `record:=false`.

## Outputs

- `~/fmp_logs/results.csv`: one row per **completed** session: `date_time, participant, session,
  trial_time_sec, max_clockwise_deg, max_anti_clockwise_deg`. A session stopped with Ctrl-C writes
  no row.
- `~/fmp_logs/mind_pour_<participant>_s<session>_<date>.bag`: `/mind_pour/command` (every received
  command, host receive time) and `/mind_pour_controller/pour_state` (FSM state, active command,
  q, commanded q7 and dq7, angle, elapsed time, at `state_rate`).

Change both locations with `record_dir:=` / `results_file:=`.

## Parameters

Everything is in `franka_mind_pour/config/mind_pour.yaml`, with units and comments. The main ones:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `mind_pour_session/joint_pose` | 0, 80, 0, −60, 0, 230, 45 deg | session start pose [rad]: flange plane vertical, facing forward |
| `mind_pour_session/max_dq` | 0.2 rad/s | speed of the move to the start pose |
| `mind_pour_udp_bridge/port` | 5005 | UDP port |
| `angular_velocity_deg` | 30 deg/s | joint 7 speed while rotating |
| `max_acceleration_deg` | 200 deg/s² | ramp to and from that speed |
| `command_hold_duration` | 0.03 s | how long one command stays active |
| `angle_limit_deg` | 90 deg | ± range around the start angle |
| `clockwise_sign` | 1 | flips which way "clockwise" turns joint 7 |
| `min_excursion_deg` | 10 deg | how far joint 7 must go before a return counts |
| `return_tolerance_deg` | 2 deg | how close to the start counts as returned |

**Direction.** With `clockwise_sign: 1`, a clockwise command turns joint 7 in its positive
direction. That is clockwise when seen from behind the flange, looking along the tool axis, which
points forward in the start pose. Check this on the robot before the first experiment, and flip
`clockwise_sign` if needed.

## Layout

```
franka_mind_pour/
  CMakeLists.txt  package.xml
  franka_controllers_plugin.xml        plugin export for controller_manager
  config/mind_pour.yaml                all parameters
  config/rosconsole.config             log levels
  include/franka_mind_pour/mind_pour_controller.h
  src/mind_pour_controller.cpp         joint 7 controller + trial timer (1 kHz)
  launch/mind_pour.launch              one session
  launch/debug_udp.launch              stage:=udp, the bridge without the robot
  msg/PourCommand.msg                  one received EEG command
  msg/PourState.msg                    one controller state sample
  scripts/udp_bridge.py                UDP -> /mind_pour/command
  scripts/session_manager.py           start pose, controller switch, results row
  scripts/record_bag.sh                rosbag wrapper, creates the output dir
scripts/
  compile_package.sh                   -> ~/workspace/catkin_ws_fmp
  run_session.sh                       one session
  fmp_udp_sender.py                    sender for the Windows PC / test pattern
```

## Status

The build is clean. The UDP path (sender → bridge → topic) has been tested on localhost. The
controller logic has been checked against a fake position interface: the ramp, the ±90° limit, the
hold, the timer, and the minimum excursion. `stage:=udp` has been run on localhost. **Nothing has
been run on the real robot yet**, `stage:=move` and `stage:=rotate` included.
