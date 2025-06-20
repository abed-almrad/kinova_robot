#ifndef KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// rclcpp tools
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

// Hardware Interface tools
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/visibility_control.h"  // To expose the hardware interface symbols

// Needed to communicate with the Kinova robotic arm
#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace k_api = Kinova::Api;

namespace kortex_driver
{
enum class StopStartInterface : std::uint8_t // Memory efficient instead of the default int 
{
  NONE,
  STOP_POS,
  STOP_TWIST,
  STOP_GRIPPER,
  STOP_FAULT_CTRL,
  START_POS,
  START_TWIST,
  START_GRIPPER,
  START_FAULT_CTRL,
}; // Used in command mode switching
class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  KortexMultiInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware); // Defines smart pointers aliases for our class

  KORTEX_DRIVER_PUBLIC  // public symbol visibility
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) final; // Overriding once

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) final;
  KORTEX_DRIVER_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) final; // Commented parameters to match the base class function definition

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) final; // Commented parameters to match the base class function definition

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) final; // Commented parameters to match the base class function definition

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) final; // Commented parameters to match the base class function definition

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) final; // Commented parameters to match the base class function definition

private:
  // Kinova arm needed connection params
  k_api::TransportClientTcp transport_tcp_;
  k_api::RouterClient router_tcp_;
  k_api::SessionManager session_manager_;
  k_api::TransportClientUdp transport_udp_realtime_;
  k_api::RouterClient router_udp_realtime_;
  k_api::SessionManager session_manager_real_time_;

  // twist temporary command params
  k_api::Base::Twist * k_api_twist_;
  k_api::Base::TwistCommand k_api_twist_command_;
  // twist command values (linear and angular xyz at the end effector)
  std::vector<double> twist_commands_;

  // kinova robotic arm control params
  k_api::Base::BaseClient base_;
  k_api::BaseCyclic::BaseCyclicClient base_cyclic_;
  k_api::BaseCyclic::Command base_command_;
  k_api::BaseCyclic::Feedback feedback_; // To minimize bandwidth we synchronize feedback with the robot only when write() is called
  std::size_t actuator_count_;
  // Variables to store the joint commands and states
  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_velocities_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // Gripper params and variables
  k_api::GripperCyclic::MotorCommand * gripper_motor_command_;
  double gripper_command_position_ = 0.0;
  double gripper_command_max_velocity_ = 0.0;
  double gripper_command_max_force_ = 0.0;
  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_force_command_ = 0.0;
  double gripper_speed_command_ = 0.0;
  std::string gripper_joint_name_;
  bool use_internal_bus_gripper_comm_;

  std::atomic<bool> block_write = false; // Used to pose writing to the robot during command_switch

  // Kinova params related to the servoing mode
  k_api::Base::ServoingMode arm_mode_;
  k_api::Base::ServoingModeInformation servoing_mode_hw_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_lvl_t : std::uint8_t // Memory efficient instead of the default int
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    EFFORT = 3
  };

  std::vector<integration_lvl_t> arm_joints_control_level_;

  // what controller is running
  bool joint_based_controller_running_;
  bool twist_controller_running_;
  bool gripper_controller_running_;
  bool fault_controller_running_;

  // keeping track of which controller is active so appropriate control mode can be adjusted
  // controller manager sends array of interfaces that should be stopped/started and this is the
  // way to internally collect information on which controller should be stopped and started
  // (different controllers claim different interfaces)

  // Control mode switching variables
  std::vector<StopStartInterface> stop_modes_;
  std::vector<StopStartInterface> start_modes_;
  // switching auxiliary booleans
  bool stop_joint_based_controller_;
  bool stop_twist_controller_;
  bool stop_gripper_controller_;
  bool stop_fault_controller_;
  bool start_joint_based_controller_;
  bool start_twist_controller_;
  bool start_gripper_controller_;
  bool start_fault_controller_;

  // first pass flag
  bool first_pass_;  // To get the robot feedback in read() only the first time then in write() in subsequent times to minimize bandwidth

  // temp variables to use in read()/write() update loop
  float cmd_degrees_tmp_;

  // Kinova Kortex API fault control variables
  double reset_fault_cmd_;
  double reset_fault_async_success_;
  double in_fault_;
  static constexpr double NO_CMD = std::numeric_limits<double>::quiet_NaN(); // Compile time  variable to prevent repeated robot fault triggering

  // Kortex API methods to command the robotic arm
  void sendTwistCommand();
  void incrementId();
  void sendJointCommands();
  void prepareCommands();
  void sendGripperCommand(
    k_api::Base::ServoingMode arm_mode, double position, double velocity, double force);

  void readGripperPosition();
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
