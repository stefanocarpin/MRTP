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
#include <geometry_msgs/msg/twist.hpp>

int main(int argc,char **argv) {
  
  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Rate rate(1);

  nodeh = rclcpp::Node::make_shared("drawsquare");
  auto pub = nodeh->create_publisher<geometry_msgs::msg::Twist>
      ("turtle1/cmd_vel",100);

  geometry_msgs::msg::Twist msg;
  while (rclcpp::ok()) { 
    msg.linear.x = 1;    
    msg.angular.z = 0;
    pub->publish(msg);
    rclcpp::spin_some(nodeh);
    rate.sleep();
    msg.linear.x = 0;
    msg.angular.z = M_PI/2; 
    pub->publish(msg);
    rclcpp::spin_some(nodeh);
    rate.sleep(); 
  }
}
