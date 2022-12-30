#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

int main(int argc,char **argv) {

  rclcpp::init(argc,argv);
  std::cout<< "Starting..." << std::endl ;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Rate rate(1);
  
  nh_ = rclcpp::Node::make_shared("talker");
  string_pub_ = nh_->create_publisher<std_msgs::msg::String>("message",1);

  int counter = 0;
  while ( (counter++ < 100) && (rclcpp::ok())  ) {
    RCLCPP_INFO(nh_->get_logger(),"Sending message #%d",counter);
    std_msgs::msg::String stringtosend;
    stringtosend.data = "Message # " + std::to_string(counter);
    string_pub_->publish(stringtosend);
    rclcpp::spin_some(nh_);
    rate.sleep();
  } 
  rclcpp::shutdown();
  return 0;
}
