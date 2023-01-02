#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

rclcpp::Node::SharedPtr nodeh;

void arrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  RCLCPP_INFO(nodeh->get_logger(),"Received new message");
  for(unsigned int i = 0 ; i <msg->data.size() ; i++)
    RCLCPP_INFO(nodeh->get_logger(),"%d",msg->data[i]);
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  nodeh = rclcpp::Node::make_shared("arraysubscriber");

  auto sub = nodeh->create_subscription<std_msgs::msg::Int32MultiArray>
    ("arrayint",10,&arrayCallback);

  rclcpp::spin(nodeh);
}
