cmake_minimum_required(VERSION 3.5)
project(examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sendarray src/sendarray.cpp)
add_executable(subarray src/subarray.cpp)
add_executable(sendmatrix src/sendmatrix.cpp)
add_executable(submatrix src/submatrix.cpp)
add_executable(pubsub src/pubsub.cpp)
add_executable(pubscan src/pubscan.cpp)
add_executable(drawsquare src/drawsquare.cpp)


ament_target_dependencies(sendarray rclcpp std_msgs)
ament_target_dependencies(subarray rclcpp std_msgs)
ament_target_dependencies(sendmatrix rclcpp std_msgs)
ament_target_dependencies(submatrix rclcpp std_msgs)
ament_target_dependencies(pubsub rclcpp std_msgs sensor_msgs)
ament_target_dependencies(pubscan rclcpp sensor_msgs)
ament_target_dependencies(drawsquare rclcpp geometry_msgs)


install(TARGETS sendarray DESTINATION lib/${PROJECT_NAME})
install(TARGETS subarray DESTINATION lib/${PROJECT_NAME})
install(TARGETS sendmatrix DESTINATION lib/${PROJECT_NAME})
install(TARGETS submatrix DESTINATION lib/${PROJECT_NAME})
install(TARGETS pubsub DESTINATION lib/${PROJECT_NAME})
install(TARGETS pubscan DESTINATION lib/${PROJECT_NAME})
install(TARGETS drawsquare DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()