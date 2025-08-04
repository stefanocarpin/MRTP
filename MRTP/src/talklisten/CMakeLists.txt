cmake_minimum_required(VERSION 3.5)
project(talklisten)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
ament_target_dependencies(talker rclcpp example_interfaces)
ament_target_dependencies(listener rclcpp example_interfaces)

install(TARGETS  talker DESTINATION lib/${PROJECT_NAME})
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})

ament_package()
