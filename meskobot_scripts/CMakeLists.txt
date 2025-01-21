cmake_minimum_required(VERSION 3.8)
project(meskobot_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(include ${EIGEN3_INCLUDE_DIRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Sol.srv"
)


#include_directories(${CMAKE_BINARY_DIR}/rosidl_generator_cpp/meskobot_scripts/include)


add_executable(ik_publisher src/ik_publisher.cpp)
add_executable(ik src/ik.cpp)
add_executable(ik_service src/ik_service.cpp)


ament_target_dependencies(
  ik_publisher
  rclcpp
  std_msgs															
)


# Link the necessary dependencies to ik_service
ament_target_dependencies(
  ik_service
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_default_runtime
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(ik_service ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  ik_publisher
  ik
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS ik_service
    DESTINATION lib/${PROJECT_NAME}
    )
# Installation steps
install(
  DIRECTORY
    scripts
    srv
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/ik_wrapper.py
  scripts/pick_and_place.py
  scripts/object_detect.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

