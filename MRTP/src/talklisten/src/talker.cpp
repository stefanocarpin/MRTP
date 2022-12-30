#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc,char **argv) {

  rclcpp::init(argc,argv);

  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::Rate rate(1);
  
  nodeh = rclcpp::Node::make_shared("talker");
  pub = nodeh->create_publisher<std_msgs::msg::String>("message",1);

  int counter = 0;
  while ( (counter++ < 100) && (rclcpp::ok())  ) {
    RCLCPP_INFO(nodeh->get_logger(),"Sending message #%d",counter);
    std_msgs::msg::String stringtosend;
    stringtosend.data = "Message # " + std::to_string(counter);
    pub->publish(stringtosend);
    rclcpp::spin_some(nodeh);
    rate.sleep();
  } 
  rclcpp::shutdown();
  return 0;
}
