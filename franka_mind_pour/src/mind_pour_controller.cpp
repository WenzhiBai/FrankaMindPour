#include "franka_mind_pour/mind_pour_controller.h"

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <algorithm>
#include <string>

namespace franka_mind_pour {

namespace {
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
}  // namespace

bool MindPourController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != kNumJoints) {
    ROS_ERROR_STREAM(
        "MindPourController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!readParameters(node_handle)) {
    return false;
  }

  auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface == nullptr) {
    ROS_ERROR_STREAM("MindPourController: Error getting position joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    try {
      joint_handles_.push_back(position_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("MindPourController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  std::string command_topic;
  node_handle.param<std::string>("command_topic", command_topic, "/mind_pour/command");
  // tcpNoDelay: a command is only worth command_hold_duration, so Nagle's
  // batching on the ROS link would eat most of it.
  command_subscriber_ =
      node_handle.subscribe(command_topic, 1, &MindPourController::commandCallback, this,
                            ros::TransportHints().reliable().tcpNoDelay());

  // The controller's state topic. session_manager.py ends the session on it,
  // and `rosbag record` in mind_pour.launch turns it into the experiment record.
  state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<PourState>>(
      node_handle, "pour_state", kStateQueueSize);
  state_period_ = ros::Duration(1.0 / state_rate_);

  ROS_INFO_STREAM("MindPourController: listening on "
                  << command_topic << ", joint 7 at " << angular_velocity_ * kRadToDeg
                  << "deg/s within +-" << angle_limit_ * kRadToDeg << "deg");
  return true;
}

bool MindPourController::readParameters(ros::NodeHandle& node_handle) {
  double angular_velocity_deg = angular_velocity_ * kRadToDeg;
  double max_acceleration_deg = max_acceleration_ * kRadToDeg;
  double angle_limit_deg = angle_limit_ * kRadToDeg;
  double min_excursion_deg = min_excursion_ * kRadToDeg;
  double return_tolerance_deg = return_tolerance_ * kRadToDeg;
  int clockwise_sign = static_cast<int>(clockwise_sign_);

  node_handle.param("angular_velocity_deg", angular_velocity_deg, angular_velocity_deg);
  node_handle.param("max_acceleration_deg", max_acceleration_deg, max_acceleration_deg);
  node_handle.param("command_hold_duration", command_hold_duration_, command_hold_duration_);
  node_handle.param("angle_limit_deg", angle_limit_deg, angle_limit_deg);
  node_handle.param("joint7_min", joint7_min_, joint7_min_);
  node_handle.param("joint7_max", joint7_max_, joint7_max_);
  node_handle.param("clockwise_sign", clockwise_sign, clockwise_sign);
  node_handle.param("min_excursion_deg", min_excursion_deg, min_excursion_deg);
  node_handle.param("return_tolerance_deg", return_tolerance_deg, return_tolerance_deg);
  node_handle.param("state_rate", state_rate_, state_rate_);

  if (angular_velocity_deg <= 0.0 || max_acceleration_deg <= 0.0 || command_hold_duration_ <= 0.0 ||
      angle_limit_deg <= 0.0 || state_rate_ <= 0.0) {
    ROS_ERROR_STREAM(
        "MindPourController: angular_velocity_deg, max_acceleration_deg, "
        "command_hold_duration, angle_limit_deg and state_rate must all be positive");
    return false;
  }
  if (clockwise_sign != 1 && clockwise_sign != -1) {
    ROS_ERROR_STREAM("MindPourController: clockwise_sign must be 1 or -1, got " << clockwise_sign);
    return false;
  }
  if (joint7_min_ >= joint7_max_) {
    ROS_ERROR_STREAM("MindPourController: joint7_min must be below joint7_max");
    return false;
  }
  // Otherwise a return could never be recognised as one: the joint would be
  // "back" before it had ever been "away".
  if (return_tolerance_deg <= 0.0 || return_tolerance_deg >= min_excursion_deg ||
      min_excursion_deg > angle_limit_deg) {
    ROS_ERROR_STREAM(
        "MindPourController: need 0 < return_tolerance_deg < min_excursion_deg <= "
        "angle_limit_deg");
    return false;
  }

  angular_velocity_ = angular_velocity_deg * kDegToRad;
  max_acceleration_ = max_acceleration_deg * kDegToRad;
  angle_limit_ = angle_limit_deg * kDegToRad;
  min_excursion_ = min_excursion_deg * kDegToRad;
  return_tolerance_ = return_tolerance_deg * kDegToRad;
  clockwise_sign_ = clockwise_sign;
  return true;
}

void MindPourController::commandCallback(const PourCommand::ConstPtr& msg) {
  Command command;
  command.received = msg->header.stamp;
  command.seq = msg->seq;
  command.command = msg->command;
  command_buffer_.writeFromNonRT(command);
}

void MindPourController::starting(const ros::Time& /*time*/) {
  // Hold every joint where it is - the session start pose, which the
  // trajectory controller has just driven the arm to.
  for (size_t i = 0; i < kNumJoints; ++i) {
    q_hold_[i] = joint_handles_[i].getPosition();
  }
  q7_start_ = q_hold_[kJoint7];
  q7_d_ = q7_start_;
  dq7_d_ = 0.0;

  q7_lower_ = std::max(q7_start_ - angle_limit_, joint7_min_);
  q7_upper_ = std::min(q7_start_ + angle_limit_, joint7_max_);
  if (q7_lower_ > q7_start_ - angle_limit_ || q7_upper_ < q7_start_ + angle_limit_) {
    ROS_WARN_STREAM("MindPourController: joint 7 starts at "
                    << q7_start_ << "rad, so its range is cut to [" << q7_lower_ << ", "
                    << q7_upper_ << "]rad by joint7_min / joint7_max");
  }

  current_state_ = ControllerState::WAITING;
  max_excursion_ = 0.0;
  elapsed_ = 0.0;
  next_state_time_ = ros::Time(0);
  ROS_INFO_STREAM("MindPourController: holding the start pose (joint 7 at "
                  << q7_start_ << "rad), waiting for the first rotate command");
}

void MindPourController::update(const ros::Time& time, const ros::Duration& period) {
  const Command latest = *command_buffer_.readFromRT();
  // Active for command_hold_duration after it arrived, or until replaced; a
  // NONE (or nothing at all yet) leaves it at 0.
  const bool fresh =
      !latest.received.isZero() && (time - latest.received).toSec() <= command_hold_duration_;
  const int8_t command =
      (current_state_ != ControllerState::DONE && fresh) ? latest.command : PourCommand::NONE;

  if (current_state_ == ControllerState::WAITING && command != PourCommand::NONE) {
    current_state_ = ControllerState::ROTATING;
    trial_start_time_ = time;
    ROS_INFO_STREAM("MindPourController: first rotate command (seq " << latest.seq
                                                                     << "), trial timer started");
  }

  stepJoint7(static_cast<int>(clockwise_sign_) * command, period.toSec());

  for (size_t i = 0; i < kJoint7; ++i) {
    joint_handles_[i].setCommand(q_hold_[i]);
  }
  joint_handles_[kJoint7].setCommand(q7_d_);

  const double offset = joint_handles_[kJoint7].getPosition() - q7_start_;
  if (current_state_ == ControllerState::ROTATING) {
    elapsed_ = (time - trial_start_time_).toSec();
    max_excursion_ = std::max(max_excursion_, std::abs(offset));
    // Back at the start after a real excursion, and settled there rather than
    // just passing through it on the way to the other side.
    if (max_excursion_ >= min_excursion_ && std::abs(offset) <= return_tolerance_ &&
        command == PourCommand::NONE && dq7_d_ == 0.0) {
      current_state_ = ControllerState::DONE;
      ROS_INFO_STREAM("MindPourController: joint 7 back at its start angle, trial time "
                      << elapsed_ << "s (largest excursion " << max_excursion_ * kRadToDeg
                      << "deg). Commands are ignored from now on");
    }
  }

  const double angle = clockwise_sign_ * offset;
  publishState(time, command, latest.seq, angle);

  ROS_INFO_STREAM_THROTTLE(
      0.5, "state: " << stateToString(current_state_) << ", command: " << static_cast<int>(command)
                     << ", angle: " << angle * kRadToDeg
                     << "deg, velocity: " << clockwise_sign_ * dq7_d_ * kRadToDeg
                     << "deg/s, trial time: " << elapsed_ << "s");
}

void MindPourController::stepJoint7(int velocity_sign, double dt) {
  double velocity_target = velocity_sign * angular_velocity_;
  // Never ask for more speed than can still be shed before the end of the
  // range, so the joint comes to rest on the limit instead of being stopped
  // dead by the clamp below.
  const double max_up = std::sqrt(2.0 * max_acceleration_ * std::max(0.0, q7_upper_ - q7_d_));
  const double max_down = std::sqrt(2.0 * max_acceleration_ * std::max(0.0, q7_d_ - q7_lower_));
  velocity_target = std::clamp(velocity_target, -max_down, max_up);

  const double max_step = max_acceleration_ * dt;
  dq7_d_ += std::clamp(velocity_target - dq7_d_, -max_step, max_step);
  q7_d_ += dq7_d_ * dt;

  if (q7_d_ >= q7_upper_ || q7_d_ <= q7_lower_) {
    q7_d_ = std::clamp(q7_d_, q7_lower_, q7_upper_);
    dq7_d_ = 0.0;
  }
}

void MindPourController::publishState(const ros::Time& time, int8_t command, uint32_t command_seq,
                                      double angle) {
  if (time < next_state_time_) {
    return;
  }
  // Deliberately not next_state_time_ += state_period_: a late tick must not
  // leave a backlog that then publishes on every tick until it has caught up.
  next_state_time_ = time + state_period_;

  // Non-blocking: if the publishing thread still holds the message, the sample
  // is dropped rather than made to wait on the 1 kHz loop.
  if (!state_publisher_->trylock()) {
    return;
  }
  static_assert(static_cast<uint8_t>(ControllerState::WAITING) == PourState::STATE_WAITING &&
                    static_cast<uint8_t>(ControllerState::ROTATING) == PourState::STATE_ROTATING &&
                    static_cast<uint8_t>(ControllerState::DONE) == PourState::STATE_DONE,
                "PourState constants must match ControllerState");

  PourState& msg = state_publisher_->msg_;
  msg.header.stamp = ros::Time::now();
  msg.state = static_cast<uint8_t>(current_state_);
  msg.command = command;
  msg.command_seq = command_seq;
  for (size_t i = 0; i < kNumJoints; ++i) {
    msg.q[i] = joint_handles_[i].getPosition();
  }
  msg.q7_start = q7_start_;
  msg.q7_desired = q7_d_;
  msg.dq7_desired = dq7_d_;
  msg.angle = angle;
  msg.elapsed = elapsed_;
  state_publisher_->unlockAndPublish();
}

const char* MindPourController::stateToString(ControllerState state) {
  switch (state) {
    case ControllerState::WAITING:
      return "WAITING";
    case ControllerState::ROTATING:
      return "ROTATING";
    case ControllerState::DONE:
      return "DONE";
  }
  return "UNKNOWN";
}

}  // namespace franka_mind_pour

PLUGINLIB_EXPORT_CLASS(franka_mind_pour::MindPourController, controller_interface::ControllerBase)
