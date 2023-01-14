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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubf;

void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std_msgs::msg::Float32 out;
  out.data = msg->ranges[0];
  for(unsigned int i = 1 ; i < msg->ranges.size() ; i++ ) {
    if ( msg->ranges[i] < out.data )
      out.data = msg->ranges[i];
  }
  pubf->publish(out);
}


int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;

  nodeh = rclcpp::Node::make_shared("pubsub");

  pubf = nodeh->create_publisher<std_msgs::msg::Float32>("closest",1000);
  auto sub = nodeh->create_subscription<sensor_msgs::msg::LaserScan>
    ("scan",10,&processScan);

  rclcpp::spin(nodeh);
}
