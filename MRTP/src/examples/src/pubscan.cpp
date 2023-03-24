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
#include <cstdlib>

#define NDATA 181 // number of readings

int main(int argc,char **argv) {
  
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    rclcpp::Rate rate(1);

    nodeh = rclcpp::Node::make_shared("pubscan"); // create node
    // create publisher
    auto pubs = nodeh->create_publisher<sensor_msgs::msg::LaserScan>
      ("scan",10);
    int iteration = 1; // just for output purposes
    // instance of message to be sent
    sensor_msgs::msg::LaserScan toSend;
    // setup data structure to send
    toSend.ranges.resize(NDATA);
    // other fields in toSend should be initialized, too...
    
    while (rclcpp::ok()) {
      // generate random distances in range 0-2 
      for (int i = 0; i < NDATA ; i++)
	toSend.ranges[i] = (2*float(rand()))/RAND_MAX;
      RCLCPP_INFO(nodeh->get_logger(),"Publishing scan #%d",iteration++);
      pubs->publish(toSend); // publish
      rate.sleep();
    }

}
