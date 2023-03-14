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

#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <navigation/navigation.hpp>
#include <iostream>

using namespace std;

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  navigator.WaitUntilNav2Active();
  navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  navigator.CancelTask();
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  /*
  goal_pos->position.x = 0;
  goal_pos->position.y = -2;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  goal_pos->position.x = 0;
  goal_pos->position.y = 2;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  */
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
