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
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(sendarray src/sendarray.cpp)
add_executable(subarray src/subarray.cpp)
add_executable(sendmatrix src/sendmatrix.cpp)
add_executable(submatrix src/submatrix.cpp)
add_executable(pubsub src/pubsub.cpp)
add_executable(pubsubstl src/pubsubstl.cpp)
add_executable(pubscan src/pubscan.cpp)
add_executable(drawsquare src/drawsquare.cpp)
add_executable(drawsquarefb src/drawsquarefb.cpp)
add_executable(geom src/geom.cpp)
add_executable(republishpose src/republishpose.cpp)
add_executable(tflistener src/tflistener.cpp)
add_executable(tfbroadcaster src/tfbroadcaster.cpp)
add_executable(paramclient src/paramclient.cpp)
add_executable(paramevent src/paramevent.cpp)
add_executable(servicecall src/servicecall.cpp)
add_executable(listeneroop src/listeneroop.cpp)
add_executable(talkeroop src/talkeroop.cpp)
add_executable(convert src/convert.cpp)
add_executable(talkerooptimer src/talkerooptimer.cpp)
add_executable(actioncall src/actioncall.cpp)
add_executable(resetparticles src/resetparticles.cpp)



ament_target_dependencies(sendarray rclcpp example_interfaces)
ament_target_dependencies(subarray rclcpp example_interfaces)
ament_target_dependencies(sendmatrix rclcpp example_interfaces)
ament_target_dependencies(submatrix rclcpp example_interfaces)
ament_target_dependencies(pubsub rclcpp example_interfaces sensor_msgs)
ament_target_dependencies(pubsubstl rclcpp std_msgs sensor_msgs)
ament_target_dependencies(pubscan rclcpp sensor_msgs)
ament_target_dependencies(drawsquare rclcpp geometry_msgs)
ament_target_dependencies(drawsquarefb rclcpp geometry_msgs turtlesim)
ament_target_dependencies(geom rclcpp geometry_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(republishpose rclcpp geometry_msgs tf2 turtlesim tf2_geometry_msgs)
ament_target_dependencies(tflistener rclcpp geometry_msgs tf2_ros)
ament_target_dependencies(tfbroadcaster rclcpp geometry_msgs tf2_ros)
ament_target_dependencies(paramclient rclcpp)
ament_target_dependencies(paramevent rclcpp rcl_interfaces)
ament_target_dependencies(servicecall rclcpp nav2_msgs)
ament_target_dependencies(listeneroop rclcpp std_msgs)
ament_target_dependencies(talkeroop rclcpp std_msgs)
ament_target_dependencies(convert rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(talkerooptimer rclcpp std_msgs)
ament_target_dependencies(actioncall rclcpp rclcpp_action nav2_msgs)
ament_target_dependencies(resetparticles rclcpp std_srvs)






install(TARGETS sendarray DESTINATION lib/${PROJECT_NAME})
install(TARGETS subarray DESTINATION lib/${PROJECT_NAME})
install(TARGETS sendmatrix DESTINATION lib/${PROJECT_NAME})
install(TARGETS submatrix DESTINATION lib/${PROJECT_NAME})
install(TARGETS pubsub DESTINATION lib/${PROJECT_NAME})
install(TARGETS pubsubstl DESTINATION lib/${PROJECT_NAME})
install(TARGETS pubscan DESTINATION lib/${PROJECT_NAME})
install(TARGETS drawsquare DESTINATION lib/${PROJECT_NAME})
install(TARGETS drawsquarefb DESTINATION lib/${PROJECT_NAME})
install(TARGETS geom DESTINATION lib/${PROJECT_NAME})
install(TARGETS republishpose DESTINATION lib/${PROJECT_NAME})
install(TARGETS tflistener DESTINATION lib/${PROJECT_NAME})
install(TARGETS tfbroadcaster DESTINATION lib/${PROJECT_NAME})
install(TARGETS paramclient DESTINATION lib/${PROJECT_NAME})
install(TARGETS paramevent DESTINATION lib/${PROJECT_NAME})
install(TARGETS servicecall DESTINATION lib/${PROJECT_NAME})
install(TARGETS listeneroop DESTINATION lib/${PROJECT_NAME})
install(TARGETS talkeroop DESTINATION lib/${PROJECT_NAME})
install(TARGETS convert DESTINATION lib/${PROJECT_NAME})
install(TARGETS talkerooptimer DESTINATION lib/${PROJECT_NAME})
install(TARGETS actioncall DESTINATION lib/${PROJECT_NAME})
install(TARGETS resetparticles DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
