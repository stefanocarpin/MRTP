#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float x;
float y;
float theta;
bool valid;

void poseReceived(const turtlesim::msg::Pose::SharedPtr msg) {
   x = msg->x;
   y = msg->y;
   theta = msg->theta;
   valid = true;
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("republishpose");

  auto sub = nodeh->create_subscription<turtlesim::msg::Pose>
                                  ("turtle1/pose",10,&poseReceived);
  auto pub = nodeh->create_publisher<geometry_msgs::msg::Pose>("pose",1000);
    
  geometry_msgs::msg::Pose poseToPublish;
  tf2::Quaternion q;
  valid = false;
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(nodeh);
    if (valid) {
      poseToPublish.position.x = x;
      poseToPublish.position.y = y;
      poseToPublish.position.z = 0;
      q.setRPY(0,0,theta);
      poseToPublish.orientation = tf2::toMsg(q);   
      pub->publish(poseToPublish);
      valid = false;
    }
  }
}
