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
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

rclcpp::Node::SharedPtr nodeh;

void stringCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received: %s",msg->data.c_str());
}

void intCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received: %d",msg->data);
}


int main(int argc,char **argv) {

  rclcpp::init(argc,argv);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subs;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subi;
  nodeh = rclcpp::Node::make_shared("multisub");
  subs = nodeh->create_subscription<std_msgs::msg::String>
                                         ("stringm",10,&stringCallback);
  subi = nodeh->create_subscription<std_msgs::msg::Int32>
                                         ("intm",10,&intCallback);
  rclcpp::spin(nodeh);
 
  rclcpp::shutdown();
  return 0;
  
}
