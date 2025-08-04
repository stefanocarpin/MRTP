/*
Copyright 2024 Stefano Carpin

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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv){

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("tflistener");

  tf2_ros::Buffer buffer(nodeh->get_clock());
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::msg::TransformStamped transformStamped;

  while (rclcpp::ok()){

    try{
      transformStamped = buffer.lookupTransform(
		 "odom", "base_link", tf2::TimePointZero);
					      
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(nodeh->get_logger(),"%s",ex.what());
      rclcpp::Rate(1.0).sleep();
      continue;
    }
    RCLCPP_INFO(nodeh->get_logger(),"Obtained transformation");
    RCLCPP_INFO(nodeh->get_logger(),"Translation: %f %f %f",
		transformStamped.transform.translation.x,
		transformStamped.transform.translation.y,
		transformStamped.transform.translation.z);
    RCLCPP_INFO(nodeh->get_logger(),"Rotation: %f %f %f %f",
		transformStamped.transform.rotation.x,
		transformStamped.transform.rotation.y,
		transformStamped.transform.rotation.z,
		transformStamped.transform.rotation.w);
  }
  return 0;
}
