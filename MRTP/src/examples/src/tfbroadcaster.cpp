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
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("tfbroadcaster");

  tf2_ros::TransformBroadcaster broadcaster(nodeh);
  geometry_msgs::msg::TransformStamped transformStamped;
  tf2::Quaternion q;
  
  while (rclcpp::ok()){
    transformStamped.header.stamp = nodeh->get_clock()->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "myframe";
    transformStamped.transform.translation.x = 4.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 2.0;
    q.setRPY(0,0,0);
   
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();   
    broadcaster.sendTransform(transformStamped);
  }
  return 0;
}
