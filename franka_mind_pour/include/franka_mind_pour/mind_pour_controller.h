#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "franka_mind_pour/PourCommand.h"
#include "franka_mind_pour/PourState.h"

namespace franka_mind_pour {

/**
 * A franka_hw joint position controller for one EEG-driven pouring trial.
 *
 * Joints 1-6 are held at the positions they had when the controller started.
 * Joint 7 is rotated at `angular_velocity` while a CLOCKWISE or ANTI_CLOCKWISE
 * command is active and brought to rest otherwise. A command is active for
 * `command_hold_duration` after it was received, or until the next command
 * replaces it, so the 50 Hz stream turns into continuous motion and a NONE
 * stops it straight away. Every change of speed is ramped at
 * `max_acceleration`, and joint 7 is kept within +-`angle_limit` of its start
 * angle, decelerating so it comes to rest on the limit.
 *
 * The FSM is WAITING -> ROTATING -> DONE. The trial timer starts on the first
 * rotate command acted on and stops when joint 7 is back at its start angle
 * after a real excursion (see `min_excursion`, `return_tolerance`); from then
 * on commands are ignored and the arm holds still.
 */
class MindPourController : public controller_interface::MultiInterfaceController<
                               hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  enum class ControllerState : uint8_t { WAITING = 0, ROTATING = 1, DONE = 2 };

  // The latest received command, handed from the subscriber thread to update().
  struct Command {
    ros::Time received;
    uint32_t seq{0};
    int8_t command{PourCommand::NONE};
  };

  static constexpr size_t kNumJoints = 7;
  static constexpr size_t kJoint7 = 6;
  static constexpr int kStateQueueSize = 100;

  bool readParameters(ros::NodeHandle& node_handle);
  void commandCallback(const PourCommand::ConstPtr& msg);

  /**
   * Advances q7_d_ / dq7_d_ by one tick towards `velocity_sign` *
   * angular_velocity_, acceleration-limited and braking early enough to come
   * to rest inside [q7_lower_, q7_upper_].
   */
  void stepJoint7(int velocity_sign, double dt);

  void publishState(const ros::Time& time, int8_t command, uint32_t command_seq, double angle);
  static const char* stateToString(ControllerState state);

  std::vector<hardware_interface::JointHandle> joint_handles_;
  ros::Subscriber command_subscriber_;
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  std::unique_ptr<realtime_tools::RealtimePublisher<PourState>> state_publisher_;
  ros::Duration state_period_;
  ros::Time next_state_time_;

  // Parameters, read from config/mind_pour.yaml and converted to radians.
  double angular_velocity_{30.0 * M_PI / 180.0};   // [rad/s]
  double max_acceleration_{200.0 * M_PI / 180.0};  // [rad/s^2]
  double command_hold_duration_{0.03};             // [s]
  double angle_limit_{90.0 * M_PI / 180.0};        // [rad]
  double joint7_min_{-2.8};                        // [rad]
  double joint7_max_{2.8};                         // [rad]
  double clockwise_sign_{1.0};                     // +-1
  double min_excursion_{10.0 * M_PI / 180.0};      // [rad]
  double return_tolerance_{2.0 * M_PI / 180.0};    // [rad]
  double state_rate_{100.0};                       // [Hz]

  // Per-trial state, reset in starting().
  ControllerState current_state_{ControllerState::WAITING};
  std::array<double, kNumJoints> q_hold_{};  // commanded positions of joints 1-6
  double q7_start_{0.0};
  double q7_lower_{0.0};
  double q7_upper_{0.0};
  double q7_d_{0.0};
  double dq7_d_{0.0};
  double max_excursion_{0.0};  // largest |q7 - q7_start_| seen in ROTATING
  ros::Time trial_start_time_;
  double elapsed_{0.0};
};

}  // namespace franka_mind_pour
