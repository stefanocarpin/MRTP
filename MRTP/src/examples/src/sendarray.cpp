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
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#define SIZE 10

int main(int argc,char **argv) {
  
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    rclcpp::Rate rate(1);

    nodeh = rclcpp::Node::make_shared("sendarray");
    auto pubA = nodeh->create_publisher<std_msgs::msg::Int32MultiArray>
      ("arrayint",10);
    
    int value = 0;
    std_msgs::msg::Int32MultiArray toSend;

    // setup data structure to send
    toSend.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    toSend.layout.dim[0].size = SIZE;
    toSend.layout.dim[0].stride = 1;
    toSend.layout.dim[0].label = "row";
    toSend.data.resize(toSend.layout.dim[0].size);
    
    while (rclcpp::ok()) {
	for (int i = 0; i < SIZE ; i++)
	      toSend.data[i] = i+value;
	pubA->publish(toSend);
	value++;
	rclcpp::spin_some(nodeh);
	rate.sleep();
    }

}
