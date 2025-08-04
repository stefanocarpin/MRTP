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
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

float x;
float y;
float theta;
bool valid;

void poseReceived(const turtlesim::msg::Pose::SharedPtr msg) {
   x = msg->x;
   y = msg->y;
   theta = msg->theta;
   valid = true;
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("republishpose");

  auto sub = nodeh->create_subscription<turtlesim::msg::Pose>
                                  ("turtle1/pose",10,&poseReceived);
  auto pub = nodeh->create_publisher<geometry_msgs::msg::Pose>("pose",1000);
    
  geometry_msgs::msg::Pose poseToPublish;
  tf2::Quaternion q;
  valid = false;
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(nodeh);
    if (valid) {
      poseToPublish.position.x = x;
      poseToPublish.position.y = y;
      poseToPublish.position.z = 0;
      q.setRPY(0,0,theta);
      poseToPublish.orientation = tf2::toMsg(q);   
      pub->publish(poseToPublish);
      valid = false;
    }
  }
}
