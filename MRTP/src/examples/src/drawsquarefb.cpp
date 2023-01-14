/*
Copyright 2023 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

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

  bool rotate = false;
  float start_x = x;
  float start_y = y;
  double DIRS[] = {0,M_PI/2,-M_PI,-M_PI/2}; // desired directions
  int direction = 0; // current direction

  while (!init) // wait for at least one sensor reading
    rclcpp::spin_some(nodeh);
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(nodeh); //get sensor reading, if available
    if (rotate) { // rotating?
      // reached desired heading?
      if ( ( ( direction == 0  ) && ( theta < DIRS[0] ) ) ||
	   ( ( direction == 1  ) && ( theta < DIRS[1] ) ) ||
	   ( ( direction == 2  ) && ( theta  > 0 ) ) ||
	   ( ( direction == 3  ) && ( theta < DIRS[3] ) ) ) {
	msg.linear.x = 0; msg.angular.z = M_PI/8; // no; keep rotating
      }
      else { // yes
	msg.linear.x = 0; msg.angular.z = 0; //stop the robot
	rotate = false; // switch to translating
	start_x = x; start_y = y; // record current location
      }
    }
    else { // translating?
      if (hypotf((x-start_x),(y-start_y))<2) { // moved less than 2 units?
	msg.linear.x = 0.5; msg.angular.z = 0;  // no, keep moving forward
      }
      else { // moved 2 units
	rotate = true; // switch to rotate
	msg.linear.x = 0; msg.angular.z = 0; // stop the robot
	++direction %= 4; // track next direction
      }
    }
    pub->publish(msg); // send motion command
  }
}
