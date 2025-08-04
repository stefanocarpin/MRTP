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

#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <example_interfaces/msg/int32.hpp> // to publish integers
#include <example_interfaces/msg/string.hpp> // to publish strings

int main(int argc,char **argv) {
  
  rclcpp::init(argc,argv); // initialize the ROS subsystem
  
  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr pubs;
  rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr pubi;
  rclcpp::Rate rate(2);

  nodeh = rclcpp::Node::make_shared("multipublish"); // create node
  // create publisher to topic "strigm" of strings
  pubs = nodeh->create_publisher<example_interfaces::msg::String>("stringm",1);
    // create publisher to topic "intm" of integers
  pubi = nodeh->create_publisher<example_interfaces::msg::Int32>("intm",1);
  
  int value=0;
  example_interfaces::msg::Int32 intToSend; // integer message to send
  example_interfaces::msg::String stringToSend; // string message to send
  stringToSend.data = "CSE180-Robotics"; // constant string to send
  
  while (rclcpp::ok()) {
    intToSend.data = value++; // update message to send
    pubi->publish(intToSend); // publish the integer message
    pubs->publish(stringToSend); // publish the string message
    RCLCPP_INFO(nodeh->get_logger(),"Completed iteration  #%d",value);    	
    rate.sleep(); // wait
  }
  rclcpp::shutdown(); // unreachable in the current form
  return 0;

}
