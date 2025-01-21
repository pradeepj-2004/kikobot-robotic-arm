cmake_minimum_required(VERSION 3.8)
project(mesko_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set dependencies

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  mesko_hardware
  SHARED
  src/mesko_hardware.cpp
)
target_compile_features(mesko_hardware PUBLIC cxx_std_17)
target_include_directories(mesko_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/mesko_hardware>
)

ament_target_dependencies(
  mesko_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  DIRECTORY launch 
  DESTINATION share/mesko_hardware/launch
)

install(
  DIRECTORY
    config
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

target_link_libraries(mesko_hardware PUBLIC serial)
target_compile_definitions(${PROJECT_NAME} PRIVATE "blue_arm_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mesko_hardware.xml)
 
# INSTALL
install(
  DIRECTORY include/mesko_hardware
  DESTINATION include/mesko_hardware
)

install(TARGETS mesko_hardware
  EXPORT export_mesko_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_mesko_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

