#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

rclcpp::Node::SharedPtr nodeh;

void callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received: %s",msg->data.c_str());
}

int main(int argc,char **argv) {

  rclcpp::init(argc,argv);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  nodeh = rclcpp::Node::make_shared("listener");
  sub = nodeh->create_subscription<std_msgs::msg::String>
                                             ("message",10,&callback);
  rclcpp::spin(nodeh);
 
  rclcpp::shutdown();
  return 0;
  
}
