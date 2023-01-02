cmake_minimum_required(VERSION 3.5)
project(multipletopics)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(multipublish src/multipublish.cpp)
add_executable(multisub src/multisub.cpp)
ament_target_dependencies(multipublish rclcpp std_msgs)
ament_target_dependencies(multisub rclcpp std_msgs)

install(TARGETS  multipublish DESTINATION lib/${PROJECT_NAME})
install(TARGETS multisub DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
