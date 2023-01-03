#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubf;

void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std_msgs::msg::Float32 out;
  out.data = msg->ranges[0];
  for(unsigned int i = 1 ; i < msg->ranges.size() ; i++ ) {
    if ( msg->ranges[i] < out.data )
      out.data = msg->ranges[i];
  }
  pubf->publish(out);
}


int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;

  nodeh = rclcpp::Node::make_shared("pubsub");

  pubf = nodeh->create_publisher<std_msgs::msg::Float32>("closest",1000);
  auto sub = nodeh->create_subscription<sensor_msgs::msg::LaserScan>
    ("scan",10,&processScan);

  rclcpp::spin(nodeh);
}
