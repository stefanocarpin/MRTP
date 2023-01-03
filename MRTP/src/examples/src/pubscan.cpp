#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstdlib>

#define NDATA 181

int main(int argc,char **argv) {
  
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    rclcpp::Rate rate(1);

    nodeh = rclcpp::Node::make_shared("pubscan");
    auto pubs = nodeh->create_publisher<sensor_msgs::msg::LaserScan>
      ("scan",10);
    int iteration = 1;
    sensor_msgs::msg::LaserScan toSend;
    // setup data structure to send
    toSend.ranges.resize(NDATA);
    // other fields in toSend should be initialized, too...
    
    while (rclcpp::ok()) {
	for (int i = 0; i < NDATA ; i++)
	  toSend.ranges[i] = rand()/RAND_MAX;
	RCLCPP_INFO(nodeh->get_logger(),"Publishing scan #%d",iteration++);
	pubs->publish(toSend);
	rclcpp::spin_some(nodeh);
	rate.sleep();
    }

}
