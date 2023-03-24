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
#include <sensor_msgs/msg/laser_scan.hpp> // to receive laser scans
#include <std_msgs/msg/float32.hpp> // to send floating point numbers
#include <algorithm> // for stl min algorithm

class FindClosest : public rclcpp::Node {
public:
  FindClosest():Node("pubsubstl") {
    pubf = this->create_publisher<std_msgs::msg::Float32>("closest",1000);
    sub = this->create_subscription<sensor_msgs::msg::LaserScan>
      ("scan",10,std::bind(&FindClosest::processScan,this,std::placeholders::_1));
  }
  
private:
  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<float>::const_iterator minval =
      min(msg->ranges.begin(),msg->ranges.end());
    std_msgs::msg::Float32 msg_to_send;
    msg_to_send.data = *minval;
    pubf->publish(msg_to_send); // publish result
  }
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubf;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
};

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<FindClosest>());  // create and spin
  rclcpp::shutdown();
  return 0;
}
