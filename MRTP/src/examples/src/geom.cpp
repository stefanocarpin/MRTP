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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("geom");

  tf2::Quaternion q1,q2,q3;
  q1.setRPY(0,0,M_PI/2);
  q2.setRPY(0,M_PI/4,0);
  q3 = q1*q2;
  RCLCPP_INFO(nodeh->get_logger(), "%f %f %f %f",
            q3.x(), q3.y(), q3.z(), q3.w());
  tf2::Matrix3x3 r;
  r.setRotation(q3);
  tf2::Vector3 o1(0,1,3),o2(1,4,0);
  tf2::Transform t1(q2,o1);
  tf2::Transform t2(r,o2);
  tf2::Transform t3= t1*t2;
  tf2::Vector3 ori = t3.getOrigin();
  RCLCPP_INFO(nodeh->get_logger(),"%f %f %f",ori.x(),ori.y(),ori.z());

}
