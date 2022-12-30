#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

rclcpp::Node::SharedPtr nh_;

void callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(nh_->get_logger(),"Received: %s",msg->data.c_str());
}

int main(int argc,char **argv) {

  rclcpp::init(argc,argv);
  std::cout<< "Starting..." << std::endl ;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  
  nh_ = rclcpp::Node::make_shared("listener");
  string_sub_ = nh_->create_subscription<std_msgs::msg::String>("message",10,&callback);
  rclcpp::spin(nh_);
 
  rclcpp::shutdown();
  return 0;
  
}
