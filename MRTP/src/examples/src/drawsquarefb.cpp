#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

float x, y, theta;
bool init;

void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
    init = true;
}

int main(int argc,char **argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Rate rate(1);

  nodeh = rclcpp::Node::make_shared("drawsquarefb");
  auto pub = nodeh->create_publisher<geometry_msgs::msg::Twist>
      ("turtle1/cmd_vel",1000);
  auto sub = nodeh->create_subscription<turtlesim::msg::Pose>
    ("turtle1/pose",1000,&poseCallback);

  geometry_msgs::msg::Twist msg;
  init = false;

  bool spin = false;
  float start_x = x;
  float start_y = y;
  double DIRS[] = {0,M_PI/2,-M_PI,-M_PI/2};
  int direction = 0;

  while (!init)
      rclcpp::spin_some(nodeh);
  
  while (rclcpp::ok()) {
      rclcpp::spin_some(nodeh);
      if (spin) {
	if ( ( ( direction == 0  ) && ( theta < DIRS[0] ) ) ||
	     ( ( direction == 1  ) && ( theta < DIRS[1] ) ) ||
	     ( ( direction == 2  ) && ( theta  >0 ) ) ||
	     ( ( direction == 3  ) && ( theta < DIRS[3] ) ) ) {
	    msg.linear.x = 0; msg.angular.z = M_PI/8;
	  }
	else {
	  msg.linear.x = 0; msg.angular.z = 0;
	  spin = false;
	  start_x = x; start_y = y;
	}
      }
      else {
	  if (hypotf((x-start_x),(y-start_y))<2) {
	      msg.linear.x = 0.5; msg.angular.z = 0; 
	  }
	  else {
	      spin = true;
	      msg.linear.x = 0; msg.angular.z = 0;
	      ++direction %= 4;
	  }
      }
      pub->publish(msg);
  }
}
