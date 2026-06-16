#include "sura_hardware_interface/actuators/servo_actuator_system.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "sura_hardware_interface/navigator_access.hpp"

namespace sura_hardware_interface
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("sura_hardware_interface");

#ifdef TARGET_RASPBERRY
// Converts a microsecond pulse (e.g., 1500us) into a 12-bit PCA9685 count (0-4095)
static uint16_t pulse_us_to_counts(double pulse_us, double freq_hz)
{
  const double period_us = 1e6 / freq_hz;
  const double counts = pulse_us * 4096.0 / period_us;

  const long rounded = std::lround(counts);
  return static_cast<uint16_t>(std::clamp(rounded, 0L, 4095L));
}
#endif

std::vector<int> parse_pwm_channels(const std::string & channels)
{
  std::vector<int> parsed_channels;
  std::stringstream stream(channels);
  std::string token;

  while (std::getline(stream, token, ',')) {
    if (token.empty()) {
      continue;
    }
    parsed_channels.push_back(std::stoi(token));
  }
  return parsed_channels;
}

bool parse_bool_parameter(const std::string & value)
{
  if (value == "true" || value == "True" || value == "TRUE" || value == "1") return true;
  if (value == "false" || value == "False" || value == "FALSE" || value == "0") return false;
  throw std::invalid_argument("Invalid boolean value: " + value);
}

}  // namespace

void ServoActuatorSystem::publish_home_command()
{
  // Manda a zero i comandi nella simulazione
  if (environment_ == "sim" && servo_sim_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(info_.joints.size(), 0.0);
    servo_sim_pub_->publish(msg);
  } else if (environment_ == "real" && pwm_enabled_) {
#ifdef TARGET_RASPBERRY
    // Imposta 1500us come comando di sicurezza iniziale (neutral)
    // Con la nuova logica -1.0 a 1.0, lo zero (0.0) corrisponde proprio a 1500us
    const uint16_t neutral_counts = pulse_us_to_counts(1500.0, pwm_frequency_hz_);

    try {
        navigator_access::call([&]() {
        for (const int channel_index : pwm_channel_indices_) {
            set_pwm_channel_value(static_cast<uintptr_t>(channel_index), neutral_counts);
        }
    });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to send home PWM command: %s", e.what());
    }
#endif
  }
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    environment_ = info_.hardware_parameters.at("environment");
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(kLogger, "Missing hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto sim_topic_it = info_.hardware_parameters.find("sim_topic");
  if (sim_topic_it != info_.hardware_parameters.end()) {
    sim_topic_ = sim_topic_it->second;
  }

  if (info_.joints.empty()) {
    RCLCPP_ERROR(kLogger, "Expected at least one servo joint");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate that joints request position interfaces
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "position") {
      RCLCPP_ERROR(kLogger, "Joint %s must have exactly one command interface: position", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "position") {
      RCLCPP_ERROR(kLogger, "Joint %s must have exactly one state interface: position", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  position_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);
  last_position_commands_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  inverted_flags_.assign(info_.joints.size(), false);
  center_offsets_.assign(info_.joints.size(), 0.0);

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    
    // Parse Inverted flag
    const auto inverted_it = joint.parameters.find("inverted");
    if (inverted_it != joint.parameters.end()) {
      inverted_flags_[index] = parse_bool_parameter(inverted_it->second);
    }

    // Parse Center Offset (if the servo is physically misaligned)
    const auto offset_it = joint.parameters.find("center_offset_rad");
    if (offset_it != joint.parameters.end()) {
      center_offsets_[index] = std::stod(offset_it->second);
    }
  }

  is_active_ = false;
  pwm_enabled_ = false;
  pwm_channel_indices_.clear();

  const auto pwm_channels_it = info_.hardware_parameters.find("pwm_channels");
  if (pwm_channels_it != info_.hardware_parameters.end()) {
    pwm_channel_indices_ = parse_pwm_channels(pwm_channels_it->second);
  }

  if (environment_ == "real" && pwm_channel_indices_.size() != info_.joints.size()) {
    RCLCPP_ERROR(kLogger, "pwm_channels must contain one channel per joint.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  pwm_enabled_ = false;

  if (environment_ == "sim") {
    internal_node_ = std::make_shared<rclcpp::Node>("servo_hardware_interface_pub");
    servo_sim_pub_ = internal_node_->create_publisher<std_msgs::msg::Float64MultiArray>(sim_topic_, 10);
    RCLCPP_INFO(kLogger, "Simulation publisher created for topic %s", sim_topic_.c_str());
  } else if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    if (pwm_channel_indices_.empty()) {
      RCLCPP_ERROR(kLogger, "Real environment requires pwm_channels hardware parameter");
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        navigator_access::initialize_once();
        navigator_access::call([&]() {
        set_pwm_freq_hz(pwm_frequency_hz_);
        set_pwm_enable(true);
        });
        pwm_enabled_ = true;
        publish_home_command();
    RCLCPP_INFO(kLogger, "PWM enabled at %.2f Hz", pwm_frequency_hz_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to initialize PWM for servos: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
#else
    RCLCPP_ERROR(kLogger, "Environment is 'real' but built without hardware bindings.");
    return hardware_interface::CallbackReturn::ERROR;
#endif
  }

  std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  publish_home_command();

#ifdef TARGET_RASPBERRY
  if (environment_ == "real" && pwm_enabled_) {
    navigator_access::call([]() {
      set_pwm_enable(false);
    });
  }
#endif

  pwm_enabled_ = false;
  internal_node_.reset();
  servo_sim_pub_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = true;
  std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  publish_home_command();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  publish_home_command();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoActuatorSystem::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> ServoActuatorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "position", &position_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ServoActuatorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[i].name, "position", &position_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type ServoActuatorSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If we don't have absolute encoders, we assume the servo reaches the commanded position.
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoActuatorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    publish_home_command();
    return hardware_interface::return_type::OK;
  }

  std::vector<double> sim_outputs(info_.joints.size(), 0.0);
#ifdef TARGET_RASPBERRY
  std::vector<uint16_t> pwm_counts(info_.joints.size(), 0U);
#endif

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    double target_position = position_commands_[index];

    // Apply software offsets and inversions
    target_position += center_offsets_[index];
    if (inverted_flags_[index]) {
      target_position = -target_position;
    }

    sim_outputs[index] = target_position;

#ifdef TARGET_RASPBERRY
    // Il target_position arriva da ROS scalato tra -1.0 e 1.0
    // Facciamo un clamp per sicurezza, casomai arrivasse un valore fuori range per sbaglio
    double command_norm = std::clamp(target_position, -1.0, 1.0);

    // Mappatura lineare universale: 
    // -1.0 -> 1000 us (Minimo / Chiuso / 0 Giri)
    //  0.0 -> 1500 us (Centro / Fermo / 1.5 Giri)
    // +1.0 -> 2000 us (Massimo / Aperto / 3 Giri)
    double pulse_us = 1500.0 + (command_norm * 500.0);

    // Clamp finale di sicurezza per l'hardware RC
    pulse_us = std::clamp(pulse_us, 1000.0, 2000.0);

    pwm_counts[index] = pulse_us_to_counts(pulse_us, pwm_frequency_hz_);
#endif
  }

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    try {
      navigator_access::call([&]() {
        for (std::size_t index = 0; index < pwm_channel_indices_.size(); ++index) {
            set_pwm_channel_value(static_cast<uintptr_t>(pwm_channel_indices_[index]), pwm_counts[index]);
        }
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to write servo PWM: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
#endif
  } else if (environment_ == "sim" && servo_sim_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = sim_outputs;
    servo_sim_pub_->publish(msg);
  }

  // Update states
  position_states_ = position_commands_;
  
  return hardware_interface::return_type::OK;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::ServoActuatorSystem,
  hardware_interface::SystemInterface)