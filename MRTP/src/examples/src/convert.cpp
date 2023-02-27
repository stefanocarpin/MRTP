/*
Copyright 2023 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2::Quaternion q;
bool valid;

void odomReceived(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // convertes a message quaternion into an instance of quaternion
  tf2::convert(msg->pose.pose.orientation,q);
  valid = true;
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("convert");

  auto sub = nodeh->create_subscription<nav_msgs::msg::Odometry>
                                  ("p2dx/odom",10,&odomReceived);
    
  valid = false;
  double roll, pitch, yaw;
	  
  while (rclcpp::ok()) {
    rclcpp::spin_some(nodeh);
    if (valid) { // received a quaternion?
      tf2::Matrix3x3 m(q); // quaternion to matrix
      m.getRPY(roll, pitch, yaw); // matrix to roll pitch yaw
      RCLCPP_INFO(nodeh->get_logger(),"Yaw is %f",yaw);
      valid = false; // do not print again until a new one is received
    }
  }
}
