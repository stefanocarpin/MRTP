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

#include <chrono>
#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <std_msgs/msg/string.hpp> // needed because we publish strings


using namespace std::chrono_literals;

class TalkerTimer : public rclcpp::Node {
public:
  TalkerTimer() : Node("talkerooptimer") {
    // create publisher
    pub = this->create_publisher<std_msgs::msg::String>("message",1);
    // create timer
    timer = this->create_wall_timer(1s,std::bind(&TalkerTimer::callback,this));
    counter = 0;
  }

private:
  void callback() {
    std_msgs::msg::String stringtosend;
    RCLCPP_INFO(this->get_logger(),"Sending message #%d",counter);
    stringtosend.data = "Message # " + std::to_string(counter);
    pub->publish(stringtosend); // publish message
    counter++;
  }
  
  int counter;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc,char **argv) {

  rclcpp::init(argc,argv); // initialize the ROS subsystem
  rclcpp::spin(std::make_shared<TalkerTimer>());
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
