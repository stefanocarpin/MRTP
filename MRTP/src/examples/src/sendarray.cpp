#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#define SIZE 10

int main(int argc,char **argv) {
  
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    rclcpp::Rate rate(1);

    nodeh = rclcpp::Node::make_shared("sendarray");
    auto pubA = nodeh->create_publisher<std_msgs::msg::Int32MultiArray>
      ("arrayint",10);
    
    int value = 0;
    std_msgs::msg::Int32MultiArray toSend;

    // setup data structure to send
    toSend.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    toSend.layout.dim[0].size = SIZE;
    toSend.layout.dim[0].stride = 1;
    toSend.layout.dim[0].label = "row";
    toSend.data.resize(toSend.layout.dim[0].size);
    
    while (rclcpp::ok()) {
	for (int i = 0; i < SIZE ; i++)
	      toSend.data[i] = i+value;
	pubA->publish(toSend);
	value++;
	rclcpp::spin_some(nodeh);
	rate.sleep();
    }

}
