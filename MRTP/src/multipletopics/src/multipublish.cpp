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

int main(int argc,char **argv) {
  
  rclcpp::init(argc,argv);
  
  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubs;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pubi;
  rclcpp::Rate rate(2);

  nodeh = rclcpp::Node::make_shared("multipublish");
  pubs = nodeh->create_publisher<std_msgs::msg::String>("stringm",1);
  pubi = nodeh->create_publisher<std_msgs::msg::Int32>("intm",1);
  
  int value=0;
  std_msgs::msg::Int32 intToSend;
  std_msgs::msg::String stringToSend;
  stringToSend.data = "CSE180-Robotics";
  
  while (rclcpp::ok()) {
    intToSend.data = value++;
    pubi->publish(intToSend);
    pubs->publish(stringToSend);
    rclcpp::spin_some(nodeh);
    RCLCPP_INFO(nodeh->get_logger(),"Completed iteration  #%d",value);    	
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;

}
