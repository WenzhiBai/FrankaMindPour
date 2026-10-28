#!/usr/bin/env python3
"""Runs one mind-pour session.

1. Drives the arm to the session start pose (~joint_pose) through franka's
   position_joint_trajectory_controller, the same way as
   franka_example_controllers/move_to_start.py.
2. Switches from that controller to mind_pour_controller, which then waits for
   EEG commands and times the trial.
3. When mind_pour_controller reports DONE, appends one row to ~results_file and
   exits. mind_pour.launch marks this node required, so that shuts the whole
   session down (and closes the bag).
"""

import csv
import datetime
import math
import os
import sys

import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
                              FollowJointTrajectoryResult)
from controller_manager_msgs.srv import (ListControllers, SwitchController,
                                         SwitchControllerRequest)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from franka_mind_pour.msg import PourState

TRAJECTORY_CONTROLLER = 'position_joint_trajectory_controller'
POUR_CONTROLLER = 'mind_pour_controller'

RESULT_FIELDS = ['date_time', 'participant', 'session', 'trial_time_sec',
                 'max_clockwise_deg', 'max_anti_clockwise_deg']


def move_to_start(pose, max_dq):
    """Port of franka_example_controllers/move_to_start.py. Returns success."""
    action = '/%s/follow_joint_trajectory' % TRAJECTORY_CONTROLLER
    client = SimpleActionClient(action, FollowJointTrajectoryAction)
    rospy.loginfo("session_manager: waiting for '%s'", action)
    client.wait_for_server()

    joint_state = rospy.wait_for_message('/franka_state_controller/joint_states', JointState)
    initial_pose = dict(zip(joint_state.name, joint_state.position))
    max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

    point = JointTrajectoryPoint()
    # The time to move the furthest joint at max_dq, or 0.5 s, whichever is longer.
    point.time_from_start = rospy.Duration.from_sec(max(max_movement / max_dq, 0.5))
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
    point.velocities = [0] * len(pose)
    goal.trajectory.points.append(point)
    goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)

    rospy.loginfo('session_manager: moving to the start pose (%.1f s)',
                  point.time_from_start.to_sec())
    client.send_goal_and_wait(goal)
    result = client.get_result()
    if result is None or result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
        rospy.logerr('session_manager: move to the start pose failed (error code %s, %s)',
                     None if result is None else result.error_code,
                     None if result is None else result.error_string)
        return False
    return True


def wait_for_controllers(timeout):
    """Waits until the trajectory controller runs and the pour controller is loaded."""
    rospy.wait_for_service('/controller_manager/list_controllers', timeout)
    list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers',
                                          ListControllers)
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        states = {c.name: c.state for c in list_controllers().controller}
        if states.get(TRAJECTORY_CONTROLLER) == 'running' and POUR_CONTROLLER in states:
            return True
        rospy.sleep(0.2)
    return False


def switch_to_pour_controller():
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    request = SwitchControllerRequest()
    request.start_controllers = [POUR_CONTROLLER]
    request.stop_controllers = [TRAJECTORY_CONTROLLER]
    request.strictness = SwitchControllerRequest.STRICT
    return switch(request).ok


class TrialMonitor:
    """Follows PourState until DONE, keeping the largest excursion each way."""

    def __init__(self):
        self.done = False
        self.trial_time = None
        self.max_clockwise = 0.0
        self.max_anti_clockwise = 0.0
        self._subscriber = rospy.Subscriber('/%s/pour_state' % POUR_CONTROLLER, PourState,
                                            self._callback, queue_size=100)

    def _callback(self, msg):
        if self.done:
            return
        self.max_clockwise = max(self.max_clockwise, msg.angle)
        self.max_anti_clockwise = max(self.max_anti_clockwise, -msg.angle)
        if msg.state == PourState.STATE_DONE:
            self.trial_time = msg.elapsed
            self.done = True


def append_result(results_file, participant, session, monitor):
    directory = os.path.dirname(results_file)
    if directory:
        os.makedirs(directory, exist_ok=True)
    new_file = not os.path.isfile(results_file)
    with open(results_file, 'a', newline='') as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(RESULT_FIELDS)
        writer.writerow([datetime.datetime.now().isoformat(timespec='seconds'), participant,
                         session, '%.3f' % monitor.trial_time,
                         '%.2f' % math.degrees(monitor.max_clockwise),
                         '%.2f' % math.degrees(monitor.max_anti_clockwise)])


def main():
    rospy.init_node('mind_pour_session')
    pose = rospy.get_param('~joint_pose', None)
    if pose is None:
        rospy.logerr('session_manager: missing required parameter ~joint_pose')
        return 1
    max_dq = rospy.get_param('~max_dq', 0.2)
    controller_wait_timeout = rospy.get_param('~controller_wait_timeout', 30.0)
    post_trial_record_sec = rospy.get_param('~post_trial_record_sec', 1.0)
    participant = rospy.get_param('~participant')
    session = rospy.get_param('~session')
    results_file = os.path.expanduser(rospy.get_param('~results_file'))

    rospy.loginfo('session_manager: participant %s, session %s', participant, session)
    if not wait_for_controllers(controller_wait_timeout):
        rospy.logerr('session_manager: controllers did not come up within %.0f s',
                     controller_wait_timeout)
        return 1
    if not move_to_start(pose, max_dq):
        return 1

    monitor = TrialMonitor()
    if not switch_to_pour_controller():
        rospy.logerr('session_manager: could not switch to %s', POUR_CONTROLLER)
        return 1
    rospy.loginfo('session_manager: READY - waiting for EEG commands')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and not monitor.done:
        rate.sleep()
    if not monitor.done:
        rospy.logwarn('session_manager: stopped before the trial finished, nothing recorded '
                      'in %s', results_file)
        return 1

    append_result(results_file, participant, session, monitor)
    rospy.loginfo('session_manager: trial time %.3f s (max clockwise %.1f deg, max '
                  'anti-clockwise %.1f deg) appended to %s', monitor.trial_time,
                  math.degrees(monitor.max_clockwise), math.degrees(monitor.max_anti_clockwise),
                  results_file)
    # Let the bag catch the final samples before the launch file shuts down.
    rospy.sleep(post_trial_record_sec)
    rospy.loginfo('session_manager: session complete, shutting down')
    return 0


if __name__ == '__main__':
    sys.exit(main())
