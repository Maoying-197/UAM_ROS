cmake_minimum_required(VERSION 3.16)
project(mujoco_arm_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# ── MuJoCo ───────────────────────────────────────────────────────────────────
# MuJoCo >= 2.1.5 ships a proper CMake package (mujoco::mujoco).
# Install from: https://github.com/deepmind/mujoco/releases
find_package(mujoco REQUIRED)

# ── Shared library ───────────────────────────────────────────────────────────
add_library(mujoco_arm_hardware SHARED
  hardware/mujoco_arm_hardware.cpp
)
target_compile_features(mujoco_arm_hardware PUBLIC cxx_std_17)

target_include_directories(mujoco_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mujoco_arm_hardware PUBLIC
  mujoco::mujoco
)

ament_target_dependencies(mujoco_arm_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export the pluginlib plugin description
pluginlib_export_plugin_description_file(hardware_interface mujoco_arm_hardware.xml)

# ── Install ──────────────────────────────────────────────────────────────────
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY mujoco/
  DESTINATION share/${PROJECT_NAME}/mujoco
)
install(
  TARGETS mujoco_arm_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
