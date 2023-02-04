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
#include <std_msgs/msg/string.hpp> // needed because we receive strings


class Listener : public rclcpp::Node  {
public:
  Listener():Node("listeneroop") {
    sub = this->create_subscription<std_msgs::msg::String>
      ("message",10,std::bind(&Listener::callback,this,std::placeholders::_1));
  }
  
private:
  // callback function called every time a message is received from the
  // topic "message"
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    // process the message: just print it to the screen
    RCLCPP_INFO(this->get_logger(),"Received: %s",msg->data.c_str());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  
};

int main(int argc,char **argv) {

  rclcpp::init(argc,argv); // initialize ROS subsystem
  rclcpp::spin(std::make_shared<Listener>());  // create and spin
  rclcpp::shutdown();
  return 0;
  
}
