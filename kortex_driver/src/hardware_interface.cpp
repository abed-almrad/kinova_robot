#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");
}

namespace kortex_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware()
: router_tcp_{
    &transport_tcp_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_{&router_tcp_},
  router_udp_realtime_{
    &transport_udp_realtime_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_real_time_{&router_udp_realtime_},
  k_api_twist_(nullptr),
  base_{&router_tcp_},
  base_cyclic_{&router_udp_realtime_},
  gripper_motor_command_(nullptr),
  gripper_command_max_velocity_(100.0),
  gripper_command_max_force_(100.0),
  servoing_mode_hw_(k_api::Base::ServoingModeInformation()),
  joint_based_controller_running_(false),
  twist_controller_running_(false),
  gripper_controller_running_(false),
  fault_controller_running_(false),
  stop_joint_based_controller_(false),
  stop_twist_controller_(false),
  stop_gripper_controller_(false),
  stop_fault_controller_(false),
  start_joint_based_controller_(false),
  start_twist_controller_(false),
  start_gripper_controller_(false),
  start_fault_controller_(false),
  first_pass_(true),
  gripper_joint_name_(""),
  use_internal_bus_gripper_comm_(false)
{
  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // Debug messages will be printed
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  if (robot_ip.empty())
  {
    RCLCPP_ERROR(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip.c_str());
  }
  // Username to log into the robot controller
  std::string username = info_.hardware_parameters["username"];
  if (username.empty())
  {
    RCLCPP_ERROR(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username.c_str());
  }
  // Password to log into the robot controller
  std::string password = info_.hardware_parameters["password"];
  if (password.empty())
  {
    RCLCPP_ERROR(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  int port = std::stoi(info_.hardware_parameters["port"]); // string to integer conversion
  if (port <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Port used '%d'", port);
  }
  int port_realtime = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Realtime port used '%d'", port_realtime);
  }

  int session_inactivity_timeout =
    std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout);
  }
  int connection_inactivity_timeout =
    std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout);
  }
  // gripper joint name
  gripper_joint_name_ = info_.hardware_parameters["gripper_joint_name"];
  if (gripper_joint_name_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  }

  gripper_command_max_velocity_ = std::stod(info_.hardware_parameters["gripper_max_velocity"]);
  gripper_command_max_force_ = std::stod(info_.hardware_parameters["gripper_max_force"]);

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  // connections
  transport_tcp_.connect(robot_ip, port);
  transport_udp_realtime_.connect(robot_ip, port_realtime);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(session_inactivity_timeout);  // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(
    connection_inactivity_timeout);  // (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(LOGGER, "Creating session for communication");
  session_manager_.CreateSession(create_session_info);
  session_manager_real_time_.CreateSession(create_session_info);
  RCLCPP_INFO(LOGGER, "Session created");

  // reset faults on activation, go back to low level servoing after
  {
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = Kinova::Api::Base::SINGLE_LEVEL_SERVOING;

    try
    {
      base_.ClearFaults();
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }

    // low level servoing on startup
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
    arm_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;
    base_.SetServoingMode(servoing_mode_hw_);
  }

  // initialize kortex api twist commandd
  {
    k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  actuator_count_ = base_.GetActuatorCount().count();
  RCLCPP_INFO(LOGGER, "Actuator count reported by robot is '%lu'", actuator_count_);

  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_joints_control_level_.resize(
    actuator_count_, integration_lvl_t::UNDEFINED);  // start with undefined control mode for each joint rather than POSITION/VELOCITY/EFFORT
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  // set size of the twist interface
  twist_commands_.resize(6, 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  if(
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
    // To be on the safe side, we should account for both True and true after the argument value parsing
  {
    use_internal_bus_gripper_comm_ = true;
    RCLCPP_INFO(LOGGER, "Using internal bus communication for gripper!");
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully setup");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  // state interface which reports if robot is in fault state
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "reset_fault", "async_success", &reset_fault_async_success_)); // Reports whether the fault reset was successfull (1.0) or not (0.0)


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    // Always expose all interfaces even if part of them is claimed under <ros2_control> in URDF 
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_)); // 0 rd < gripper command < 0.81 rd

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_velocity", &gripper_speed_command_));
      gripper_speed_command_ = gripper_command_max_velocity_; // gripper speed default is set to max value (100.0 as set in the class constructor)
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_effort", &gripper_force_command_));
      gripper_force_command_ = gripper_command_max_force_; // gripper force is default is set to max value (100.0 as set in the class constructor)
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }
  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  // Always expose all interfaces even if part of them is claimed under <ros2_control> in URDF 
  {
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
  }

  // register twist command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_commands_[3]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_commands_[4]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_commands_[5]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_)); // NaN or a number value to reset the robot's fault state 
                                                                                        
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface("reset_fault", "async_success", &reset_fault_async_success_)); 
  // Reports whether the fault state was successfully reset (1.0) or not (0.0)
  return command_interfaces;
  // Command interfaces values are set by the controllers and communicated through the Controller Manager
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces) // stop and start interfaces are automatically filled by the Controller Manager when a switch is triggered
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = false;

  block_write = true; // stops writing to the robot until done with the command switching
  std::this_thread::sleep_for(std::chrono::milliseconds(200)); // sleep to ensure all outgoing write commands have finished

  start_modes_.clear();
  stop_modes_.clear();

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto & key : stop_interfaces)
  {
    if (key == gripper_joint_name_ + "/" + hardware_interface::HW_IF_POSITION)
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_GRIPPER);
      continue;
    }
    if (key == gripper_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)
    {
      continue; // gripper velocity control is not implemented in write()
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_TWIST);
      continue;
    }
    if (key == "reset_fault/command")
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_FAULT_CTRL);
      continue;
    }

    for (auto & joint : info_.joints)
    {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS);
        break;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        break; // joint velocity control is not implemented in write()
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        break; // joint effort control is not implemented in write()
      }
    }

  }

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto & key : start_interfaces)
  {
    if (key == gripper_joint_name_ + "/" + hardware_interface::HW_IF_POSITION)
    {
      start_modes_.emplace_back(StopStartInterface::START_GRIPPER);
      continue;
    }
    if (key == gripper_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY)
    {
      continue; // gripper velocity control is not implemented in write()
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      start_modes_.emplace_back(StopStartInterface::START_TWIST);
      continue;
    }
    if (key == "reset_fault/command")
    {
      start_modes_.emplace_back(StopStartInterface::START_FAULT_CTRL);
      continue;
    }

    for (auto & joint : info_.joints)
    {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS);
        break;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        break; // joint velocity control is not implemented in write()
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        break; // joint effort control is not implemented in write()
      }
    }

  }

  // prepare flags for performing the switch
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_POS) !=
      stop_modes_.end())
  {
    stop_joint_based_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_TWIST) !=
      stop_modes_.end())
  {
    stop_twist_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_GRIPPER) !=
      stop_modes_.end())
  {
    stop_gripper_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_FAULT_CTRL) !=
      stop_modes_.end())
  {
    stop_fault_controller_ = true;
  }

  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_POS) !=
     start_modes_.end()))
  {
    start_joint_based_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_TWIST) !=
      start_modes_.end())
  {
    start_twist_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_GRIPPER) !=
     start_modes_.end()))
  {
    start_gripper_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_FAULT_CTRL) !=
     start_modes_.end()))
  {
    start_fault_controller_ = true;
  }

  // handle exclusiveness between twist and joint based controller
  if (twist_controller_running_ && start_joint_based_controller_ && !stop_twist_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start joint based controller while twist controller is running!");
    return hardware_interface::return_type::ERROR;
  }
  if (joint_based_controller_running_ && start_twist_controller_ && !stop_joint_based_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start twist controller while joint based controller is running!");
    return hardware_interface::return_type::ERROR;
  }

  return ret_val;
}

return_type KortexMultiInterfaceHardware::perform_command_mode_switch(
  const vector<std::string> & /*start_interfaces*/, const vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_joint_based_controller_)
  {
    joint_based_controller_running_ = false; // key boolen in write()
    arm_commands_positions_ = arm_positions_;
    arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // commanding the joints to keep their current positions
                                                              // as a fail-safe mechanism in case the robot was controlled
                                                          // one last time using residual commands from the joint_based_controller
  }
  if (stop_twist_controller_)
  {
    twist_controller_running_ = false; // key boolean in write()
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (stop_gripper_controller_)
  {
    gripper_controller_running_ = false; // key boolean in write()
    gripper_command_position_ = gripper_position_; // commanding the gripper to keep its current position
                                                  // as a fail-safe mechanism in case the gripper was controlled
                                                  // one last time using residual commands from the gripper_controller
  }
  if (stop_fault_controller_)
  {
    fault_controller_running_ = false;
  }

  if (start_joint_based_controller_)
  {
    servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
    twist_controller_running_ = false;
    arm_commands_positions_ = arm_positions_;
    arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // commanding the joints to keep their current positions
                                                          // to ensure a smooth controller start without unexpected jerk/jittering
    joint_based_controller_running_ = true; // key boolean in write()
    // refresh feedback
    feedback_ = base_cyclic_.RefreshFeedback();
  }
  if (start_twist_controller_)
  {
    servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
    joint_based_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // zero velocities to ensure a smooth controller start without unexpected
                                                      // jerk/jittering
    twist_controller_running_ = true; // key boolean in write()
  }
  if (start_gripper_controller_)
  {
    gripper_command_position_ = gripper_position_;
    gripper_controller_running_ = true; // key boolean in write()
  }
  if (start_fault_controller_)
  {
    fault_controller_running_ = true; // key boolean in write()
  }

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = false;

  start_modes_.clear();
  stop_modes_.clear();

  block_write = false; // resume writing to the robot since the command switching is done

  return ret_val;
}
//************************************************************************** */

CallbackReturn KortexMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");
  // first read
  auto base_feedback = base_cyclic_.RefreshFeedback();

  // Add each actuator to the base_command_ and set the command to its current position
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    base_command_.add_actuators()->set_position(base_feedback.actuators(i).position());
  }

  // Initialize gripper
  float gripper_initial_position =
    base_feedback.interconnect().gripper_feedback().motor()[0].position();
  RCLCPP_INFO(LOGGER, "Gripper initial position is '%f'.", gripper_initial_position);

  // to radians
  gripper_command_position_ = gripper_initial_position / 100.0 * 0.81; // Command interface

  // Initialize interconnect command to current gripper position.
  base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
  gripper_motor_command_ = base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
  gripper_motor_command_->set_position(gripper_initial_position);  // % initial position
  gripper_motor_command_->set_velocity(gripper_speed_command_);    // % speed set to max = 100.0 in the export_command_interfaces()
  gripper_motor_command_->set_force(gripper_force_command_);       // % force set to max = 100.0 in the export_command_interfaces()

  // Send a first frame
  base_feedback = base_cyclic_.Refresh(base_command_); // Send the previously set commands to the arm and the gripper 
  // Set some default values
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (std::isnan(arm_positions_[i]))
    {
      arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(base_feedback.actuators(i).position()));  // rad // State interface
    }
    if (std::isnan(arm_velocities_[i]))
    {
      arm_velocities_[i] = 0;  // Since the robot is idle // State interface
    }
    if (std::isnan(arm_efforts_[i]))
    {
      arm_efforts_[i] = base_feedback.actuators(i).torque(); // State interface
    }
    if (std::isnan(arm_commands_positions_[i]))
    {
      arm_commands_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(base_feedback.actuators(i).position()));  // rad
    }
    if (std::isnan(arm_commands_velocities_[i]))
    {
      arm_commands_velocities_[i] = 0;  // Velocity control is not implemented, so this value doesn't matter.
    }
    if (std::isnan(arm_commands_efforts_[i]))
    {
      arm_commands_efforts_[i] = 0; // Effort control is not implemented, so this value doesn't matter.
    }
    arm_joints_control_level_[i] = integration_lvl_t::UNDEFINED;
  }

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating KortexMultiInterfaceHardware...");

  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set back the servoing mode to Single Level Servoing
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode);

  // Close API session
  session_manager_.CloseSession();
  session_manager_real_time_.CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_tcp_.SetActivationStatus(false);
  transport_tcp_.disconnect();
  router_udp_realtime_.SetActivationStatus(false);
  transport_udp_realtime_.disconnect();

  // memory handling
  delete k_api_twist_;
  delete gripper_motor_command_;

  // Reset controller flags
  joint_based_controller_running_ = false;
  twist_controller_running_ = false;
  gripper_controller_running_ = false;
  fault_controller_running_ = false;

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

return_type KortexMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (first_pass_) // Set to true in the class constructor
  {
    first_pass_ = false;
    feedback_ = base_cyclic_.RefreshFeedback(); // The robot state is read only the first time in read() and subsquently in write() to minimize bandwidth
  }

  // read if robot is faulted
  in_fault_ = (feedback_.base().active_state() == Kinova::Api::Common::ArmState::ARMSTATE_IN_FAULT);

  // read gripper state
  readGripperPosition();

  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    // read torque
    arm_efforts_[i] = feedback_.actuators(i).torque();  // N*m // State interface
    // read velocity
    arm_velocities_[i] = KortexMathUtil::toRad(feedback_.actuators(i).velocity());  // rad/sec // State interface
    // read position
    arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
      KortexMathUtil::toRad(feedback_.actuators(i).position()));  // rad // State interface

    in_fault_ += (feedback_.actuators(i).fault_bank_a() + feedback_.actuators(i).fault_bank_b());
  }

  // add all base's faults and warnings
  in_fault_ += (feedback_.base().fault_bank_a() + feedback_.base().fault_bank_b());

  // If in_fault_ is non_zero, we won't be able to command the robot in write()

  return return_type::OK;
}

void KortexMultiInterfaceHardware::readGripperPosition()
{
  // max joint angle = 0.81 rad for robotiq_2f_85
  // TODO(anyone) read in as parameter from kortex_controllers.yaml
  if (use_internal_bus_gripper_comm_) // Set to true in on_init()
  {
    gripper_position_ =
      feedback_.interconnect().gripper_feedback().motor()[0].position() / 100.0 * 0.81;  // rad
  }
}

return_type KortexMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (block_write) // stops writing to robot while performing command switch
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    return return_type::OK;
  }

  if (!std::isnan(reset_fault_cmd_) && fault_controller_running_)
  {
    try
    {
      // change servoing mode first
      servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base_.SetServoingMode(servoing_mode_hw_);
      // apply emergency stop - twice to make it sure it is applied
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      // clear faults
      base_.ClearFaults();
      // back to original servoing mode
      if (
        arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING ||
        arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        servoing_mode_hw_.set_servoing_mode(arm_mode_);
        base_.SetServoingMode(servoing_mode_hw_);
      }
      reset_fault_async_success_ = 1.0;
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
      reset_fault_async_success_ = 0.0;
    }
    catch (...)
    {
      reset_fault_async_success_ = 0.0;
    }
    reset_fault_cmd_ = NO_CMD; // To make sure the fault reset is triggered only once per fault command
  }

  if (in_fault_ == 0.0)  // robot NOT in fault state
  {
    if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING &&
      (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_READY)) // Check both the intended and actual arm state
    {
      if (gripper_controller_running_)
      {
      // gripper control
      sendGripperCommand(
        arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_); // Can be commanded in both HIGH
                                                                                              // and LOW level modes
      }
      // Twist controller active
      if (twist_controller_running_)
      {
        // twist control
        sendTwistCommand();
      }
      else
      {
        // Keep alive mode - no controller active
        RCLCPP_DEBUG(LOGGER, "No arm's controller active in SINGLE_LEVEL_SERVOING mode!");
      }

      // read after writing to the robot
      feedback_ = base_cyclic_.RefreshFeedback();
    }
    else if (
      (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
      (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL)) 
                                                                                  // Check both the intended and actual arm state
    {
      if (gripper_controller_running_)
      {
      // gripper control
      sendGripperCommand(
        arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
      }
      if (joint_based_controller_running_)
      {
        // send commands to the joints
        sendJointCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        feedback_ = base_cyclic_.RefreshFeedback();  // feedback is not updated if sendJointCommands() is called since it 
                                                    // is updated there
        RCLCPP_DEBUG(LOGGER, "No arm's controller active in LOW_LEVEL_SERVOING mode!");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      feedback_ = base_cyclic_.RefreshFeedback();
      RCLCPP_DEBUG(
        LOGGER,
        "Fault was not recognized on the robot but the combination of intended Control Mode and arm's Active State "
        "is not supported!");
    }
  }
  else
  {
    // if the arm goes to fault there will be nowhere else to refresh the feedback and recognize if the fault got cleared
    feedback_ = base_cyclic_.RefreshFeedback();
  }

  return return_type::OK;
}

void KortexMultiInterfaceHardware::incrementId()
{
  // Incrementing the frame id for each sent command to ensure the arm executes the command otherwise it will be rejected
  base_command_.set_frame_id(base_command_.frame_id() + 1);
  if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);  // uint16_t max out  ==> enough to afford reusing 
                                                                        // the same IDs
}

void KortexMultiInterfaceHardware::prepareCommands()
{  // update the command for each joint
  for (size_t i = 0; i < actuator_count_; i++)
  {
    // set command per joint
    cmd_degrees_tmp_ = static_cast<float>(
      KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i])));
    //cmd_vel_tmp_ = static_cast<float>(KortexMathUtil::toDeg(arm_commands_velocities_[i]));
      
    base_command_.mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp_);
    // Velocity command interface not implemented
    // base_command_.mutable_actuators(i)->set_velocity(cmd_vel_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  }
}

void KortexMultiInterfaceHardware::sendJointCommands()
{
  // identifier++
  incrementId();

  prepareCommands();

  // send the command to the robot
  try
  {
    feedback_ = base_cyclic_.Refresh(base_command_);  // sends the command to the arm and returns feedback
  }
  catch (k_api::KDetailedException & ex)
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(
      LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error & ex_runtime)
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error & ex_future)
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception & ex_std)
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
  // Make sure to read feedback even in the case of exception
}

void KortexMultiInterfaceHardware::sendGripperCommand(
  k_api::Base::ServoingMode arm_mode, double position, double velocity, double force)
{
  if (!std::isnan(position) && use_internal_bus_gripper_comm_)
  {
    try
    {
      if (arm_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
      {
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(
          static_cast<float>(position / 0.81));  // This values needs to be between 0 and 1
        base_.SendGripperCommand(gripper_command);  // Sends command to move the gripper
      }
      else if (arm_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        // % open/closed, this values needs to be between 0 and 100
        gripper_motor_command_->set_position(static_cast<float>(position / 0.81 * 100.0)); // This values needs to be between 0 
                                                                                          // and 100%
        // % gripper speed between 0 and 100 percent
        gripper_motor_command_->set_velocity(static_cast<float>(velocity));
        // % max force threshold, between 0 and 100
        gripper_motor_command_->set_force(static_cast<float>(force));
        feedback_ = base_cyclic_.Refresh(base_command_); // Send the gripper command to move the gripper 
      }
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception caught while sending internal gripper command!");
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }
  }
}

void KortexMultiInterfaceHardware::sendTwistCommand()
{
  k_api_twist_->set_linear_x(static_cast<float>(twist_commands_[0]));
  k_api_twist_->set_linear_y(static_cast<float>(twist_commands_[1]));
  k_api_twist_->set_linear_z(static_cast<float>(twist_commands_[2]));
  k_api_twist_->set_angular_x(static_cast<float>(twist_commands_[3]));
  k_api_twist_->set_angular_y(static_cast<float>(twist_commands_[4]));
  k_api_twist_->set_angular_z(static_cast<float>(twist_commands_[5]));
  base_.SendTwistCommand(k_api_twist_command_);
}

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface) // Needed so that ROS2 loads this class as
                                                                                    // a System Interface at runtime
