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

#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <std_msgs/msg/int32_multi_array.hpp> // we receive arrays of ints

rclcpp::Node::SharedPtr nodeh;

// callback function to process incoming messages
void arrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received new message");
  // just print everything to the screen
  for(unsigned int i = 0 ; i <msg->data.size() ; i++)
    RCLCPP_INFO(nodeh->get_logger(),"%d",msg->data[i]);
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  nodeh = rclcpp::Node::make_shared("arraysubscriber"); // create node

  // create subscriber and register callback function
  auto sub = nodeh->create_subscription<std_msgs::msg::Int32MultiArray>
    ("arrayint",10,&arrayCallback);
  
  // receive all messages
  rclcpp::spin(nodeh);
}
