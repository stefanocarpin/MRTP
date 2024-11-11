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
#include <std_msgs/msg/string.hpp> // needed because we publish strings

int main(int argc,char **argv) {

  rclcpp::init(argc,argv); // initialize the ROS subsystem

  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::Rate rate(1); // create rate object
  
  nodeh = rclcpp::Node::make_shared("talker"); // create node instance
  // create publisher to topic "message" of strings
  pub = nodeh->create_publisher<std_msgs::msg::String>("message",1);

  int counter = 0;
  while ( (counter++ < 100) && (rclcpp::ok())  ) {
    RCLCPP_INFO(nodeh->get_logger(),"Sending message #%d",counter);
    std_msgs::msg::String stringtosend;
    // prepare message to send
    stringtosend.data = "Message # " + std::to_string(counter);
    pub->publish(stringtosend); // publish message
    rate.sleep(); // wait
  } 
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
