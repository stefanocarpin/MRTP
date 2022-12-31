#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

rclcpp::Node::SharedPtr nodeh;

void stringCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received: %s",msg->data.c_str());
}

void intCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received: %d",msg->data);
}


int main(int argc,char **argv) {

  rclcpp::init(argc,argv);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subs;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subi;
  nodeh = rclcpp::Node::make_shared("multisub");
  subs = nodeh->create_subscription<std_msgs::msg::String>
                                         ("stringm",10,&stringCallback);
  subi = nodeh->create_subscription<std_msgs::msg::Int32>
                                         ("intm",10,&intCallback);
  rclcpp::spin(nodeh);
 
  rclcpp::shutdown();
  return 0;
  
}
