cmake_minimum_required(VERSION 3.8)
project(mocap_pose_enu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mocap_pose_enu_node src/mocap_pose_enu_node.cpp)
ament_target_dependencies(mocap_pose_enu_node
  geometry_msgs
  mocap4r2_msgs
  rclcpp
)

install(TARGETS mocap_pose_enu_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
