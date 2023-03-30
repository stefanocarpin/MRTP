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
#include <navigation/navigation.hpp>
#include <iostream>
#include <memory>

/* This file tests all functionalities in the navigation library */


int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); 
  Navigator navigator(true); // create node with no debug info and no verbose messages

  // First test initialization methods
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init); // test SetInitialPose
  navigator.WaitUntilNav2Active(); // test WaitUntilActive

  // Now start testing functionalities
  
  navigator.Spin(); // test Spin action
  while ( ! navigator.IsTaskComplete() ) {  // test IsTaskComplete
    auto feedback_ptr = navigator.GetFeedback(); // test GetFeedback
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;
     
  }
  auto result = navigator.GetResult(); // test GetResult
  if ( result == rclcpp_action::ResultCode::SUCCEEDED )
    std::cout << "Spin action succeeded" << std::endl;
  else
    std::cout << "Spin goal was not achieved" << std::endl;
  
  navigator.Spin(-1.57); // execute Spin action again to cancel it
  int i = 0;
  while ( ( ! navigator.IsTaskComplete() ) && (i < 3) ) { // wait for 3 feedback messages and then cancel
    i++;
  }
  navigator.CancelTask(); // test CancelTask
  result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::CANCELED )
    std::cout << "Spin action was canceled as intended" << std::endl;
  else
    std::cout << "Cancel task did not return the expected result." << std::endl;
  

  // test GoToPose
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED )
    std::cout << "GoToPose action succeeded" << std::endl;
  else
    std::cout << "GoToPose goal was not achieved" << std::endl;

  // test clearLocalCostMap
  std::cout << "Clearing local costmap" << std::endl;
  navigator.ClearLocalCostmap();
  
  // test clearLocalCostMap
  std::cout << "Clearing global costmap" << std::endl;
  navigator.ClearGlobalCostmap();

  // test clearAllCostMaps
  std::cout << "Clearing all costmaps" << std::endl;
  navigator.ClearAllCostmaps();

  
  // test Backup
  navigator.Backup(); // use default distance and speed
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED )
    std::cout << "Backup action succeeded" << std::endl;
  else
    std::cout << "Backup goal was not achieved" << std::endl;

  // test GetGlobalCostmap
  std::shared_ptr<nav2_msgs::msg::Costmap> global_costmap = navigator.GetGlobalCostmap();
  std::cout << "Global costmap has dimensions " << global_costmap->metadata.size_x << "," << global_costmap->metadata.size_y << std::endl;

  // test GetLocalCostmap
  std::shared_ptr<nav2_msgs::msg::Costmap> local_costmap = navigator.GetLocalCostmap();
  std::cout << "Global costmap has dimensions " << local_costmap->metadata.size_x << "," << local_costmap->metadata.size_y << std::endl;

  // test GetPath
  goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  // move to new pose
  auto path = navigator.GetPath(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED ) {
    std::cout << "GetPath action succeeded" << std::endl;
    std::cout << "Received a path with " << path->poses.size() << " intermediate poses" << std::endl;
  }
  else
    std::cout << "GetPath goal was not achieved" << std::endl;

  // test FollowPath (but only if a path was returned)
  if ( result == rclcpp_action::ResultCode::SUCCEEDED ) {   
    navigator.FollowPath(path);
    while ( ! navigator.IsTaskComplete() ) {
      
    }
    result = navigator.GetResult(); 
    if ( result == rclcpp_action::ResultCode::SUCCEEDED ) 
      std::cout << "FollowPath action succeeded" << std::endl;
    else
      std::cout << "FollowPath goal was not achieved" << std::endl;
  }

  // test FollowWaypoints
  geometry_msgs::msg::PoseStamped p1,p2,p3;
  p1.pose.position.x = 2;
  p1.pose.position.y = 1;
  p2.pose.position.x = -2;
  p2.pose.position.y = 1;
  p3.pose.position.x = -2;
  p3.pose.position.y = -1;
  std::vector<geometry_msgs::msg::PoseStamped> pointList;
  pointList.push_back(p1);
  pointList.push_back(p2);
  pointList.push_back(p3);
  navigator.FollowWaypoints(pointList);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED ) 
    std::cout << "FollowWaypoints action succeeded" << std::endl;
  else
    std::cout << "FollowWaypoints goal was not achieved" << std::endl;

  // test ChangeMap -- should fail
  navigator.ChangeMap("bogusmap.png");
  result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED ) 
    std::cout << "ChangeMap action succeeded" << std::endl;
  else
    std::cout << "ChangeMap goal was not achieved" << std::endl;
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
