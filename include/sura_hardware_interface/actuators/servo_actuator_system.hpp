#pragma once

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace sura_hardware_interface
{

class ServoActuatorSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ServoActuatorSystem)

  // Standard ros2_control lifecycle and interface methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Helper function to send servos to a safe/center position when deactivated
  void publish_home_command();

  // Environment type (e.g., "real" or "sim")
  std::string environment_;

  // Topic for simulation if applicable
  std::string sim_topic_{"/bluerov/controller/servo_setpoints_sim"};

  bool is_active_{false};

  // Vectors to hold the commands (target positions) and states (current positions)
  std::vector<double> position_commands_;
  std::vector<double> position_states_;
  std::vector<double> last_position_commands_;

  // Per-joint configuration flags
  // If true, invert the rotation direction logic for the specific servo
  std::vector<bool> inverted_flags_;
  
  // Offset in radians/normalized value to center the servo correctly in software
  std::vector<double> center_offsets_;
  std::vector<double> pulse_min_us_;
  std::vector<double> pulse_max_us_;
  // Internal ROS 2 node and publisher for simulation outputs
  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_sim_pub_;

  // Hardware PWM variables
  bool pwm_enabled_{false};
  double pwm_frequency_hz_{50.0}; // 50 Hz is standard for most RC servos

  // Hardware pin or channel mapping for the servos
  std::vector<int> pwm_channel_indices_;
};

}  // namespace sura_hardware_interface