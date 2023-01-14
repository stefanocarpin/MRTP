#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv){

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("tflistener");

  tf2_ros::Buffer buffer(nodeh->get_clock());
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::msg::TransformStamped transformStamped;
  
  while (rclcpp::ok()){

    try{
      transformStamped = buffer.lookupTransform("odom_husky", "base_link",
                               rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(nodeh->get_logger(),"%s",ex.what());
      rclcpp::Rate(1.0).sleep();
      continue;
    }
    RCLCPP_INFO(nodeh->get_logger(),"Obtained transformation");
    RCLCPP_INFO(nodeh->get_logger(),"Translation: %f %f %f",
		transformStamped.transform.translation.x,
		transformStamped.transform.translation.y,
		transformStamped.transform.translation.z);
    
  }
  return 0;
}
