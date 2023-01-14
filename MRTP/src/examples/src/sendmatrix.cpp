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

#define ROWS 4
#define COLS 5

int main(int argc,char **argv) {  
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    rclcpp::Rate rate(1);

    nodeh = rclcpp::Node::make_shared("sendmatrix");
    auto pubA = nodeh->create_publisher<std_msgs::msg::Int32MultiArray>
      ("matrixint",10);

    std_msgs::msg::Int32MultiArray toSend;
    int value;
    
    // setup layout for a matrix of size ROWS * COLS
    toSend.layout.dim.resize(2); 
    toSend.layout.dim[0].size = ROWS;
    toSend.layout.dim[0].stride = ROWS * COLS;
    toSend.layout.dim[0].label = "row";
    toSend.layout.dim[1].size = COLS;
    toSend.layout.dim[1].stride = COLS;
    toSend.layout.dim[1].label = "col";
    toSend.layout.data_offset = 0;
    toSend.data.resize(toSend.layout.dim[0].stride);
    while (rclcpp::ok()) {
      // fills entry (i,j) with i*j+value
      for (int i = 0; i < ROWS ; i++) {
	for ( int j = 0 ; j < COLS ; j++ ) {
	  toSend.data[i*toSend.layout.dim[1].stride + j] = i*j+value;
	}
      }
      value++;
      pubA->publish(toSend);
      rclcpp::spin_some(nodeh);
      rate.sleep();
    }
}
