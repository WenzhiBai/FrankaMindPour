# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`franka_mind_pour` is a ROS 1 Noetic (Ubuntu 20.04) catkin package. EEG commands decoded on a
separate Windows PC (clockwise / none / anti-clockwise, ~50 Hz, UDP over a direct Ethernet cable)
rotate **only joint 7** of a Franka Research 3 to perform a pour, within ±`angle_limit_deg` of the
session start angle. The package also times each trial. See `README.md` for the session flow, the
network setup and the design choices.

## Build / run

This repo is **not** built in place. `compile_package.sh` wipes `~/workspace/catkin_ws_fmp/`, copies
`franka_mind_pour/` into it and `catkin build`s there against `catkin_ws_franka`. Edits here do not
reach the running binary until you re-run the compile script. This applies to the yaml, the launch
file and the Python nodes as well as the C++, because `roslaunch` resolves all of them from the copy.

```bash
./scripts/compile_package.sh                           # from the repo root
./scripts/run_session.sh participant:=P01 session:=1   # one session; unknown keys are rejected
```

`run_session.sh stage:=udp|move|rotate` runs one step on its own for debugging (README, "Debugging
one step at a time"). `udp` launches `debug_udp.launch` and never touches the robot. `move` and
`rotate` go through `mind_pour.launch` and are branches on `~stage` in `session_manager.py`. Only
`stage:=full` appends to the results CSV.

`compile_package.sh` takes `ROOT_DIR` from `pwd`, so it must be run from the repo root. It also
appends a `source` line for `catkin_ws_fmp` to `~/.bashrc` and `~/.zshrc` if it is missing. The
`build/` directory at the repo root is the VS Code CMake extension's Debug configure of
`franka_mind_pour/` (gitignored, used for `compile_commands.json`). It is not what runs.

There is no test suite and no CI. Verification is: a clean build; the UDP path tested on localhost;
and then a real-robot session. Treat any change to `update()` as unverified until it has run on the
robot, and say so. The fake position interface that the README's Status section mentions is not in
the repo.

```bash
# UDP path without the robot, one terminal each
./scripts/run_session.sh stage:=udp                   # bridge + `rostopic hz`, safe to run any time
python3 scripts/fmp_udp_sender.py --host 127.0.0.1    # default pattern: 2 s clockwise, 2 s back
```

Do not run `stage:=move`, `stage:=rotate` or a full session yourself: they move the arm.

Code style: Google C++, `IndentWidth: 2`, `ColumnLimit: 100` (`.clang-format`). C++17.

## Architecture

There are two `scripts/` directories. The one at the repo root holds the operator tools
(`compile_package.sh`, `run_session.sh`, and `fmp_udp_sender.py`, which is copied to the Windows
PC). The paths below are inside the package, `franka_mind_pour/`.

- `scripts/udp_bridge.py`: parses `"<seq>,<command>,<sender_time>\n"` datagrams, drops late
  packets (by `seq`) and malformed ones, and publishes `PourCommand` on `/mind_pour/command`, stamped
  with the **host receive time**.
- `src/mind_pour_controller.cpp`: a `PositionJointInterface` controller plugin, spawned stopped by
  `controller_manager` (not a node). FSM `WAITING → ROTATING → DONE`. `starting()` latches all 7
  joints. Joints 1–6 are commanded to those values every tick. Joint 7 follows `stepJoint7()`.
- `scripts/session_manager.py`: moves to the start pose through `position_joint_trajectory_controller`
  (a port of `move_to_start.py`), switches STRICT to `mind_pour_controller`, and waits for
  `PourState.STATE_DONE`. It then appends to the results CSV and exits. It is `required` in the
  launch file, so its exit ends the session. A session stopped before `DONE` (Ctrl-C included)
  writes no CSV row.
- `launch/mind_pour.launch`: includes `franka_control.launch`, loads the yaml with `subst_value`
  (so `$(arg arm_id)` works inside it), and spawns `position_joint_trajectory_controller` running
  and `mind_pour_controller` `--stopped`. `udp_bridge.py` is also `required`. The rosbag recorder
  is not.
- The controller plugin runs inside the `franka_control` process, so its log lines appear under that
  node. `config/rosconsole.config` sets every logger to WARN except `ros.franka_mind_pour` (INFO).

## Conventions

- **A new launch argument has to be added in two places:** `launch/mind_pour.launch` and the
  whitelist in the root `scripts/run_session.sh` (default, usage text, `case` entry, and the
  `roslaunch` call). `run_session.sh` rejects any key it does not know.
- **Defaults are repeated.** A controller default appears in the yaml, in the header's member
  initialiser, and in the README's parameter table. The robot IP `192.168.1.11` is in both
  `run_session.sh` and the launch file. Change them together.
- **A new message or Python node needs a `CMakeLists.txt` entry** (`add_message_files`,
  `catkin_install_python`). New `.cpp` files under `src/` are picked up by the glob.

- **All parameters live in `config/mind_pour.yaml`.** Nothing is hard-coded. Adding a controller
  parameter means: a yaml entry (with a comment and units) → a member and default in the header →
  `node_handle.param` plus validation in `readParameters()`. Experimenter-facing angles are in
  degrees (`_deg`) and converted once in `readParameters()`. Joint positions are in radians.
- **A command is aged against the host clock.** It is active while
  `time - header.stamp <= command_hold_duration`, or until a newer command replaces it. Do not age it
  against `sender_time`: the Windows clock is not synchronised.
- **Every speed change goes through `max_acceleration`,** stopping included. `stepJoint7()` also caps
  speed at `sqrt(2 a d)` to the nearer limit, so the joint comes to rest on the limit. The clamp
  after it is only a backstop. Do not command velocity steps: the FR3 rejects them.
- **`DONE` needs all of:** `max_excursion_ >= min_excursion_`, `|q7 - q7_start| <= return_tolerance_`,
  no active command, and `dq7_d_ == 0` exactly. The ramp lands exactly on zero, which is why the
  equality check is safe. Without the rest condition, swinging through the start angle to the other
  side would end the trial.
- **`angle` in `PourState` is clockwise-positive** (`clockwise_sign * (q7 - q7_start)`). The CSV's
  max clockwise / anti-clockwise values come from it.
- **`PourState` constants must match `ControllerState`.** The `static_assert` in `publishState()`
  enforces this.
- 1 kHz path: no allocation, no blocking. `publishState()` uses `trylock()`, and commands come in
  through `RealtimeBuffer`. Extend the one `ROS_INFO_STREAM_THROTTLE` line rather than adding
  per-tick logging.

## Safety

The arm **moves on its own** to the start pose at the beginning of every session (a straight
trajectory with no obstacle avoidance), and joint 7 then turns on EEG commands. Bounds on joint 7:
±`angle_limit_deg` around the start angle, cut to `[joint7_min, joint7_max]`; `angular_velocity_deg`;
`max_acceleration_deg`; and `franka_control`'s rate limiting. Keep these bounds. `clockwise_sign` has
not been verified on the robot yet.
