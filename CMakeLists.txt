cmake_minimum_required(VERSION 3.8)
project(sura_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Detect architecture
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SURA_SYSTEM_PROCESSOR)

set(TARGET_RASPBERRY FALSE)

if(SURA_SYSTEM_PROCESSOR MATCHES "aarch64|armv7l|armv7|arm")
  set(TARGET_RASPBERRY TRUE)
endif()

# Stonefish is only needed for desktop/simulation builds.
option(SURA_ENABLE_STONEFISH "Build Stonefish simulation support" ON)

if(TARGET_RASPBERRY)
  message(STATUS "Raspberry/ARM detected: disabling stonefish_ros2 dependency")
  set(SURA_ENABLE_STONEFISH OFF CACHE BOOL "Build Stonefish simulation support" FORCE)
endif()

if(SURA_ENABLE_STONEFISH)
  find_package(stonefish_ros2 REQUIRED)
  add_compile_definitions(SURA_HAS_STONEFISH=1)
else()
  add_compile_definitions(SURA_HAS_STONEFISH=0)
endif()

include(ExternalProject)

set(ZIP_URL "https://github.com/bluerobotics/navigator-lib/releases/latest/download/cpp.zip")
set(NAVIGATOR_DIR "${CMAKE_BINARY_DIR}/navigator_zip_contents")

if(TARGET_RASPBERRY)
  if(SURA_SYSTEM_PROCESSOR MATCHES "armv7")
    set(NAVIGATOR_ARCH "armv7-unknown-linux-gnueabihf")
  elseif(SURA_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(NAVIGATOR_ARCH "aarch64-unknown-linux-gnu")
  else()
    message(FATAL_ERROR "Unsupported Raspberry/ARM architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  ExternalProject_Add(
    navigator_lib
    URL ${ZIP_URL}
    DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/navigator_download"
    SOURCE_DIR ${NAVIGATOR_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND ""
  )

  set(NAVIGATOR_LIB "${NAVIGATOR_DIR}/${NAVIGATOR_ARCH}/libbluerobotics_navigator.so")
else()
  message(WARNING "Building sura_hardware_interface in desktop mode on architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/thrusters/thrusters_system.cpp
  src/thrusters/thruster_mapper.cpp
  src/actuators/servo_actuator_system.cpp
  src/sensors/sensors_system.cpp
  src/sensors/imu_interface.cpp
  src/sensors/magnetometer_interface.cpp
  src/sensors/pressure_interface.cpp
  src/sensors/battery_interface.cpp
  src/sensors/dvl75_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

if(TARGET_RASPBERRY)
  add_dependencies(${PROJECT_NAME} navigator_lib)

  target_include_directories(${PROJECT_NAME} PRIVATE
    ${NAVIGATOR_DIR}
  )

  target_link_libraries(${PROJECT_NAME}
    ${NAVIGATOR_LIB}
  )

  target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_RASPBERRY)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_DESKTOP)
endif()

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

if(SURA_ENABLE_STONEFISH)
  ament_target_dependencies(${PROJECT_NAME}
    stonefish_ros2
  )
endif()

pluginlib_export_plugin_description_file(
  hardware_interface
  sura_hardware_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES sura_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

if(SURA_ENABLE_STONEFISH)
  ament_export_dependencies(stonefish_ros2)
endif()

ament_package()