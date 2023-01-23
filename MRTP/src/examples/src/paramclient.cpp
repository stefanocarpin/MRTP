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

int main(int argc,char **argv) {
  
  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("paramclient");
  nodeh->declare_parameter<std::string>("sensorport","/dev/tty0");
  std::string port;
  while (rclcpp::ok()) {
    port = nodeh->get_parameter("sensorport").get_parameter_value().get<std::string>();
    RCLCPP_INFO(nodeh->get_logger(), "Parameter value: %s", port.c_str());
    rclcpp::spin_some(nodeh);
  }
 
}

