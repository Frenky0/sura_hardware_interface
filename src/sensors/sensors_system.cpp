#include "sura_hardware_interface/sensors/sensors_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace sura_hardware_interface
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("SensorsSystem");

std::string trim_namespace(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }

  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }

  if (value.empty()) {
    return "cirtesub";
  }

  return value;
}
}  // namespace

std::string SensorsSystem::parameter_or(
  const std::string & name,
  const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  return it->second;
}

double SensorsSystem::parameter_or(const std::string & name, double default_value) const
{
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  try {
    return std::stod(it->second);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      kLogger,
      "Invalid double hardware parameter '%s'='%s': %s. Using %.3f",
      name.c_str(),
      it->second.c_str(),
      e.what(),
      default_value);
    return default_value;
  }
}

namespace
{
std::chrono::steady_clock::duration period_from_rate(double rate_hz)
{
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    return std::chrono::steady_clock::duration::max();
  }

  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / rate_hz));
}
}  // namespace

hardware_interface::CallbackReturn SensorsSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(kLogger, "Failed to initialize base SystemInterface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  reset_sensor_state();
  is_active_ = false;

  environment_ = parameter_or("environment", "real");
  if (environment_ != "sim" && environment_ != "real") {
    RCLCPP_ERROR(kLogger, "Unsupported environment '%s'. Use 'sim' or 'real'.", environment_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_namespace_ = trim_namespace(parameter_or("robot_namespace", "cirtesub"));
  const std::string topic_root = "/" + robot_namespace_;

  sim_imu_topic_ = parameter_or("sim_imu_topic", topic_root + "/stonefish/sensors/imu");
  sim_magnetometer_topic_ =
    parameter_or("sim_magnetometer_topic", topic_root + "/stonefish/sensors/magnetometer");
  sim_pressure_topic_ = parameter_or("sim_pressure_topic", topic_root + "/stonefish/sensors/pressure");
  sim_dvl_topic_ = parameter_or("sim_dvl_topic", topic_root + "/stonefish/sensors/dvl");
  sim_dvl_altitude_topic_ =
    parameter_or("sim_dvl_altitude_topic", topic_root + "/stonefish/sensors/dvl/altitude");
  sim_gps_topic_ = parameter_or("sim_gps_topic", topic_root + "/stonefish/sensors/gps");
  pressure_offset_pa_ = parameter_or("pressure_offset_pa", 101325.0);
  sim_dvl_confidence_ = parameter_or("sim_dvl_confidence", 100.0);
  pressure_read_rate_hz_ = parameter_or("pressure_read_rate_hz", 100.0);
  dvl_read_rate_hz_ = parameter_or("dvl_read_rate_hz", 50.0);
  battery_read_rate_hz_ = parameter_or("battery_read_rate_hz", 1.0);

  const auto has_sensor = [this](const std::string & sensor_name) {
    return std::any_of(
      info_.sensors.begin(),
      info_.sensors.end(),
      [&sensor_name](const auto & sensor) {
        return sensor.name == sensor_name;
      });
  };

  has_imu_ = has_sensor(imu_sensor_name_);
  has_magnetometer_ = has_sensor(magnetometer_sensor_name_);
  has_pressure_ = has_sensor(pressure_sensor_name_);
  has_battery_ = has_sensor(battery_sensor_name_);
  has_dvl_ = has_sensor(dvl_sensor_name_);

  if (!has_imu_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      imu_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_magnetometer_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      magnetometer_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_pressure_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      pressure_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_battery_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      battery_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_dvl_) {
    RCLCPP_INFO(
      kLogger,
      "Sensor '%s' not found in ros2_control description. DVL support disabled.",
      dvl_sensor_name_.c_str());
  }

  RCLCPP_INFO(
    kLogger,
    "SensorsSystem initialized for environment='%s', robot_namespace='%s'",
    environment_.c_str(),
    robot_namespace_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Configuring SensorsSystem...");

  reset_sensor_state();
  reset_sim_subscribers();

  if (environment_ == "sim") {
    configure_sim_subscribers();
  } else {
    if (!imu_.initialize(info_)) {
      RCLCPP_ERROR(kLogger, "Failed to initialize IMU interface");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_dvl_) {
      if (!dvl_.initialize(info_)) {
        RCLCPP_ERROR(kLogger, "Failed to initialize DVL interface");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!battery_.initialize(info_)) {
      RCLCPP_ERROR(kLogger, "Failed to initialize battery interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_active_ = false;

  RCLCPP_INFO(kLogger, "SensorsSystem configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Activating SensorsSystem...");

  if (environment_ == "real") {
    if (!imu_.activate()) {
      RCLCPP_ERROR(kLogger, "Failed to activate IMU");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_dvl_) {
      if (!dvl_.activate()) {
        RCLCPP_ERROR(kLogger, "Failed to activate DVL");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!battery_.activate()) {
      RCLCPP_ERROR(kLogger, "Failed to activate battery");
      return hardware_interface::CallbackReturn::ERROR;
    }

    poll_pressure_once();
    poll_battery_once();
    if (has_dvl_) {
      poll_dvl_once();
    }
    start_real_sensor_threads();
  }

  is_active_ = true;

  RCLCPP_INFO(kLogger, "SensorsSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Deactivating SensorsSystem...");

  is_active_ = false;

  if (environment_ == "real") {
    stop_real_sensor_threads();

    if (!imu_.deactivate()) {
      RCLCPP_ERROR(kLogger, "Failed to deactivate IMU");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_dvl_) {
      if (!dvl_.deactivate()) {
        RCLCPP_ERROR(kLogger, "Failed to deactivate DVL");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!battery_.deactivate()) {
      RCLCPP_ERROR(kLogger, "Failed to deactivate battery");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(kLogger, "SensorsSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  interfaces.reserve(
    (has_imu_ ? 10U : 0U) +
    (has_magnetometer_ ? 3U : 0U) +
    (has_pressure_ ? 1U : 0U) +
    (has_battery_ ? 3U : 0U) +
    (has_dvl_ ? 12U : 0U));

  if (has_imu_) {
    interfaces.emplace_back(imu_sensor_name_, "orientation.x", &orientation_x_);
    interfaces.emplace_back(imu_sensor_name_, "orientation.y", &orientation_y_);
    interfaces.emplace_back(imu_sensor_name_, "orientation.z", &orientation_z_);
    interfaces.emplace_back(imu_sensor_name_, "orientation.w", &orientation_w_);

    interfaces.emplace_back(imu_sensor_name_, "angular_velocity.x", &angular_velocity_x_);
    interfaces.emplace_back(imu_sensor_name_, "angular_velocity.y", &angular_velocity_y_);
    interfaces.emplace_back(imu_sensor_name_, "angular_velocity.z", &angular_velocity_z_);

    interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.x", &linear_acceleration_x_);
    interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.y", &linear_acceleration_y_);
    interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.z", &linear_acceleration_z_);
  }

  if (has_magnetometer_) {
    interfaces.emplace_back(magnetometer_sensor_name_, "magnetic_field.x", &magnetic_field_x_);
    interfaces.emplace_back(magnetometer_sensor_name_, "magnetic_field.y", &magnetic_field_y_);
    interfaces.emplace_back(magnetometer_sensor_name_, "magnetic_field.z", &magnetic_field_z_);
  }

  if (has_pressure_) {
    interfaces.emplace_back(pressure_sensor_name_, "fluid_pressure", &fluid_pressure_);
  }

  if (has_battery_) {
    interfaces.emplace_back(battery_sensor_name_, "voltage", &battery_voltage_);
    interfaces.emplace_back(battery_sensor_name_, "current", &battery_current_);
    interfaces.emplace_back(battery_sensor_name_, "present", &battery_present_);
  }

  if (has_dvl_) {
    interfaces.emplace_back(dvl_sensor_name_, "linear_velocity.x", &dvl_linear_velocity_x_);
    interfaces.emplace_back(dvl_sensor_name_, "linear_velocity.y", &dvl_linear_velocity_y_);
    interfaces.emplace_back(dvl_sensor_name_, "linear_velocity.z", &dvl_linear_velocity_z_);

    interfaces.emplace_back(dvl_sensor_name_, "angular_velocity.x", &dvl_angular_velocity_x_);
    interfaces.emplace_back(dvl_sensor_name_, "angular_velocity.y", &dvl_angular_velocity_y_);
    interfaces.emplace_back(dvl_sensor_name_, "angular_velocity.z", &dvl_angular_velocity_z_);

    interfaces.emplace_back(dvl_sensor_name_, "distance_z", &dvl_distance_z_);
    interfaces.emplace_back(dvl_sensor_name_, "confidence", &dvl_confidence_);

    interfaces.emplace_back(dvl_sensor_name_, "gps.latitude", &dvl_gps_latitude_);
    interfaces.emplace_back(dvl_sensor_name_, "gps.longitude", &dvl_gps_longitude_);
    interfaces.emplace_back(dvl_sensor_name_, "gps.altitude", &dvl_gps_altitude_);
    interfaces.emplace_back(dvl_sensor_name_, "gps.valid", &dvl_gps_valid_);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SensorsSystem::export_command_interfaces()
{
  return {};
}

hardware_interface::CallbackReturn SensorsSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Cleaning up SensorsSystem...");

  is_active_ = false;

  if (environment_ == "real") {
    stop_real_sensor_threads();

    if (!imu_.cleanup()) {
      RCLCPP_ERROR(kLogger, "Failed to cleanup IMU");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_dvl_) {
      if (!dvl_.cleanup()) {
        RCLCPP_ERROR(kLogger, "Failed to cleanup DVL");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!battery_.cleanup()) {
      RCLCPP_ERROR(kLogger, "Failed to cleanup battery");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  reset_sim_subscribers();
  reset_sensor_state();

  RCLCPP_INFO(kLogger, "SensorsSystem cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Shutting down SensorsSystem...");

  is_active_ = false;

  if (environment_ == "real") {
    stop_real_sensor_threads();

    (void)imu_.deactivate();
    (void)imu_.cleanup();
    if (has_dvl_) {
      (void)dvl_.deactivate();
      (void)dvl_.cleanup();
    }
    (void)battery_.deactivate();
    (void)battery_.cleanup();
  }

  reset_sim_subscribers();
  reset_sensor_state();

  RCLCPP_INFO(kLogger, "SensorsSystem shutdown completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(kLogger, "SensorsSystem entered error state");

  is_active_ = false;

  if (environment_ == "real") {
    stop_real_sensor_threads();

    (void)imu_.deactivate();
    (void)imu_.cleanup();
    if (has_dvl_) {
      (void)dvl_.deactivate();
      (void)dvl_.cleanup();
    }
    (void)battery_.deactivate();
    (void)battery_.cleanup();
  }

  reset_sim_subscribers();
  reset_sensor_state();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SensorsSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  if (environment_ == "sim") {
    if (sim_node_) {
      rclcpp::spin_some(sim_node_);
    }
    return hardware_interface::return_type::OK;
  }

  const bool imu_ok = imu_.read(
    orientation_x_, orientation_y_, orientation_z_, orientation_w_,
    angular_velocity_x_, angular_velocity_y_, angular_velocity_z_,
    linear_acceleration_x_, linear_acceleration_y_, linear_acceleration_z_);

  if (!imu_ok) {
    RCLCPP_ERROR(kLogger, "Failed to read IMU data");
    return hardware_interface::return_type::ERROR;
  }

  const bool mag_ok = magnetometer_.read(
    magnetic_field_x_, magnetic_field_y_, magnetic_field_z_);

  if (!mag_ok) {
    RCLCPP_ERROR(kLogger, "Failed to read magnetometer data");
    return hardware_interface::return_type::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(real_sensor_cache_mutex_);

    fluid_pressure_ = cached_fluid_pressure_;

    battery_voltage_ = cached_battery_voltage_;
    battery_current_ = cached_battery_current_;
    battery_present_ = cached_battery_present_;

    dvl_linear_velocity_x_ = cached_dvl_linear_velocity_x_;
    dvl_linear_velocity_y_ = cached_dvl_linear_velocity_y_;
    dvl_linear_velocity_z_ = cached_dvl_linear_velocity_z_;
    dvl_angular_velocity_x_ = cached_dvl_angular_velocity_x_;
    dvl_angular_velocity_y_ = cached_dvl_angular_velocity_y_;
    dvl_angular_velocity_z_ = cached_dvl_angular_velocity_z_;
    dvl_distance_z_ = cached_dvl_distance_z_;
    dvl_confidence_ = cached_dvl_confidence_;
    dvl_gps_latitude_ = cached_dvl_gps_latitude_;
    dvl_gps_longitude_ = cached_dvl_gps_longitude_;
    dvl_gps_altitude_ = cached_dvl_gps_altitude_;
    dvl_gps_valid_ = cached_dvl_gps_valid_;
  }

  return hardware_interface::return_type::OK;
}

void SensorsSystem::start_real_sensor_threads()
{
  stop_real_sensor_threads();

  real_sensor_threads_running_.store(true);

  pressure_thread_ = std::thread(&SensorsSystem::pressure_poll_loop, this);
  battery_thread_ = std::thread(&SensorsSystem::battery_poll_loop, this);
  if (has_dvl_) {
    dvl_thread_ = std::thread(&SensorsSystem::dvl_poll_loop, this);
  }
}

void SensorsSystem::stop_real_sensor_threads()
{
  real_sensor_threads_running_.store(false);

  if (pressure_thread_.joinable()) {
    pressure_thread_.join();
  }

  if (battery_thread_.joinable()) {
    battery_thread_.join();
  }

  if (dvl_thread_.joinable()) {
    dvl_thread_.join();
  }
}

void SensorsSystem::pressure_poll_loop()
{
  const auto period = period_from_rate(pressure_read_rate_hz_);
  auto next = std::chrono::steady_clock::now();

  while (real_sensor_threads_running_.load()) {
    poll_pressure_once();

    if (period == std::chrono::steady_clock::duration::max()) {
      break;
    }

    next += period;
    std::this_thread::sleep_until(next);
  }
}

void SensorsSystem::battery_poll_loop()
{
  const auto period = period_from_rate(battery_read_rate_hz_);
  auto next = std::chrono::steady_clock::now();

  while (real_sensor_threads_running_.load()) {
    poll_battery_once();

    if (period == std::chrono::steady_clock::duration::max()) {
      break;
    }

    next += period;
    std::this_thread::sleep_until(next);
  }
}

void SensorsSystem::dvl_poll_loop()
{
  const auto period = period_from_rate(dvl_read_rate_hz_);
  auto next = std::chrono::steady_clock::now();

  while (real_sensor_threads_running_.load()) {
    poll_dvl_once();

    if (period == std::chrono::steady_clock::duration::max()) {
      break;
    }

    next += period;
    std::this_thread::sleep_until(next);
  }
}

void SensorsSystem::poll_pressure_once()
{
  double pressure_mbar = 0.0;
  if (!pressure_.read(pressure_mbar)) {
    return;
  }

  const double pressure_pa = pressure_mbar * 100.0;
  const double fluid_pressure = std::max(0.0, pressure_pa - pressure_offset_pa_);

  std::lock_guard<std::mutex> lock(real_sensor_cache_mutex_);
  cached_fluid_pressure_ = fluid_pressure;
}

void SensorsSystem::poll_battery_once()
{
  double voltage = std::numeric_limits<double>::quiet_NaN();
  double current = std::numeric_limits<double>::quiet_NaN();
  double present = 0.0;

  if (!battery_.read(voltage, current, present)) {
    return;
  }

  std::lock_guard<std::mutex> lock(real_sensor_cache_mutex_);
  cached_battery_voltage_ = voltage;
  cached_battery_current_ = current;
  cached_battery_present_ = present;
}

void SensorsSystem::poll_dvl_once()
{
  double linear_velocity_x = 0.0;
  double linear_velocity_y = 0.0;
  double linear_velocity_z = 0.0;
  double angular_velocity_x = 0.0;
  double angular_velocity_y = 0.0;
  double angular_velocity_z = 0.0;
  double distance_z = 0.0;
  double confidence = 0.0;
  double gps_latitude = 0.0;
  double gps_longitude = 0.0;
  double gps_altitude = 0.0;
  double gps_valid = 0.0;

  if (!dvl_.read(
      linear_velocity_x,
      linear_velocity_y,
      linear_velocity_z,
      angular_velocity_x,
      angular_velocity_y,
      angular_velocity_z,
      distance_z,
      confidence,
      gps_latitude,
      gps_longitude,
      gps_altitude,
      gps_valid))
  {
    gps_valid = 0.0;
  }

  std::lock_guard<std::mutex> lock(real_sensor_cache_mutex_);
  cached_dvl_linear_velocity_x_ = linear_velocity_x;
  cached_dvl_linear_velocity_y_ = linear_velocity_y;
  cached_dvl_linear_velocity_z_ = linear_velocity_z;
  cached_dvl_angular_velocity_x_ = angular_velocity_x;
  cached_dvl_angular_velocity_y_ = angular_velocity_y;
  cached_dvl_angular_velocity_z_ = angular_velocity_z;
  cached_dvl_distance_z_ = distance_z;
  cached_dvl_confidence_ = confidence;
  cached_dvl_gps_latitude_ = gps_latitude;
  cached_dvl_gps_longitude_ = gps_longitude;
  cached_dvl_gps_altitude_ = gps_altitude;
  cached_dvl_gps_valid_ = gps_valid;
}

hardware_interface::return_type SensorsSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

void SensorsSystem::configure_sim_subscribers()
{
  sim_node_ = std::make_shared<rclcpp::Node>(
    "sura_sensors_system_sim_" + robot_namespace_);

  sim_imu_sub_ = sim_node_->create_subscription<sensor_msgs::msg::Imu>(
    sim_imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      sim_imu_callback(msg);
    });

  sim_magnetometer_sub_ = sim_node_->create_subscription<sensor_msgs::msg::MagneticField>(
    sim_magnetometer_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
      sim_magnetometer_callback(msg);
    });

  sim_pressure_sub_ = sim_node_->create_subscription<sensor_msgs::msg::FluidPressure>(
    sim_pressure_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
      sim_pressure_callback(msg);
    });

  if (has_dvl_) {
#if SURA_HAS_STONEFISH
    sim_dvl_sub_ = sim_node_->create_subscription<stonefish_ros2::msg::DVL>(
      sim_dvl_topic_, rclcpp::SensorDataQoS(),
      [this](const stonefish_ros2::msg::DVL::SharedPtr msg) {
        sim_dvl_callback(msg);
      });
#else
    RCLCPP_WARN(
      kLogger,
      "Stonefish support is disabled in this build. Skipping simulated DVL velocity subscriber '%s'.",
      sim_dvl_topic_.c_str());
#endif

    sim_dvl_altitude_sub_ = sim_node_->create_subscription<sensor_msgs::msg::Range>(
      sim_dvl_altitude_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        sim_dvl_altitude_callback(msg);
      });

    sim_gps_sub_ = sim_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      sim_gps_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        sim_gps_callback(msg);
      });
  }

  RCLCPP_INFO(
    kLogger,
    "Configured simulation sensor subscribers: imu='%s', mag='%s', pressure='%s', dvl_enabled=%s",
    sim_imu_topic_.c_str(),
    sim_magnetometer_topic_.c_str(),
    sim_pressure_topic_.c_str(),
    has_dvl_ ? "true" : "false");
}

void SensorsSystem::reset_sim_subscribers()
{
  sim_imu_sub_.reset();
  sim_magnetometer_sub_.reset();
  sim_pressure_sub_.reset();

#if SURA_HAS_STONEFISH
  sim_dvl_sub_.reset();
#endif

  sim_dvl_altitude_sub_.reset();
  sim_gps_sub_.reset();
  sim_node_.reset();
}

void SensorsSystem::sim_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  orientation_x_ = msg->orientation.x;
  orientation_y_ = msg->orientation.y;
  orientation_z_ = msg->orientation.z;
  orientation_w_ = msg->orientation.w;

  angular_velocity_x_ = msg->angular_velocity.x;
  angular_velocity_y_ = msg->angular_velocity.y;
  angular_velocity_z_ = msg->angular_velocity.z;

  linear_acceleration_x_ = msg->linear_acceleration.x;
  linear_acceleration_y_ = msg->linear_acceleration.y;
  linear_acceleration_z_ = msg->linear_acceleration.z;
}

void SensorsSystem::sim_magnetometer_callback(
  const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  magnetic_field_x_ = msg->magnetic_field.x;
  magnetic_field_y_ = msg->magnetic_field.y;
  magnetic_field_z_ = msg->magnetic_field.z;
}

void SensorsSystem::sim_pressure_callback(
  const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
  fluid_pressure_ = msg->fluid_pressure;
}

#if SURA_HAS_STONEFISH
void SensorsSystem::sim_dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg)
{
  dvl_linear_velocity_x_ = msg->velocity.x;
  dvl_linear_velocity_y_ = msg->velocity.y;
  dvl_linear_velocity_z_ = msg->velocity.z;

  dvl_angular_velocity_x_ = 0.0;
  dvl_angular_velocity_y_ = 0.0;
  dvl_angular_velocity_z_ = 0.0;

  if (std::isfinite(msg->altitude) && msg->altitude >= 0.0) {
    dvl_distance_z_ = msg->altitude;
    dvl_confidence_ = sim_dvl_confidence_;
  } else {
    dvl_confidence_ = 0.0;
  }
}
#endif

void SensorsSystem::sim_dvl_altitude_callback(
  const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (std::isfinite(msg->range) && msg->range >= 0.0) {
    dvl_distance_z_ = msg->range;
    dvl_confidence_ = sim_dvl_confidence_;
  } else {
    dvl_confidence_ = 0.0;
  }
}

void SensorsSystem::sim_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  dvl_gps_latitude_ = msg->latitude;
  dvl_gps_longitude_ = msg->longitude;
  dvl_gps_altitude_ = msg->altitude;
  dvl_gps_valid_ = msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX ? 1.0 : 0.0;
}

void SensorsSystem::reset_sensor_state()
{
  orientation_x_ = 0.0;
  orientation_y_ = 0.0;
  orientation_z_ = 0.0;
  orientation_w_ = 1.0;

  angular_velocity_x_ = 0.0;
  angular_velocity_y_ = 0.0;
  angular_velocity_z_ = 0.0;

  linear_acceleration_x_ = 0.0;
  linear_acceleration_y_ = 0.0;
  linear_acceleration_z_ = 0.0;

  magnetic_field_x_ = 0.0;
  magnetic_field_y_ = 0.0;
  magnetic_field_z_ = 0.0;

  fluid_pressure_ = 0.0;

  battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
  battery_current_ = std::numeric_limits<double>::quiet_NaN();
  battery_present_ = 0.0;

  dvl_linear_velocity_x_ = 0.0;
  dvl_linear_velocity_y_ = 0.0;
  dvl_linear_velocity_z_ = 0.0;

  dvl_angular_velocity_x_ = 0.0;
  dvl_angular_velocity_y_ = 0.0;
  dvl_angular_velocity_z_ = 0.0;

  dvl_distance_z_ = 0.0;
  dvl_confidence_ = 0.0;

  dvl_gps_latitude_ = 0.0;
  dvl_gps_longitude_ = 0.0;
  dvl_gps_altitude_ = 0.0;
  dvl_gps_valid_ = 0.0;

  std::lock_guard<std::mutex> lock(real_sensor_cache_mutex_);

  cached_fluid_pressure_ = fluid_pressure_;

  cached_battery_voltage_ = battery_voltage_;
  cached_battery_current_ = battery_current_;
  cached_battery_present_ = battery_present_;

  cached_dvl_linear_velocity_x_ = dvl_linear_velocity_x_;
  cached_dvl_linear_velocity_y_ = dvl_linear_velocity_y_;
  cached_dvl_linear_velocity_z_ = dvl_linear_velocity_z_;
  cached_dvl_angular_velocity_x_ = dvl_angular_velocity_x_;
  cached_dvl_angular_velocity_y_ = dvl_angular_velocity_y_;
  cached_dvl_angular_velocity_z_ = dvl_angular_velocity_z_;
  cached_dvl_distance_z_ = dvl_distance_z_;
  cached_dvl_confidence_ = dvl_confidence_;
  cached_dvl_gps_latitude_ = dvl_gps_latitude_;
  cached_dvl_gps_longitude_ = dvl_gps_longitude_;
  cached_dvl_gps_altitude_ = dvl_gps_altitude_;
  cached_dvl_gps_valid_ = dvl_gps_valid_;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::SensorsSystem,
  hardware_interface::SystemInterface)
