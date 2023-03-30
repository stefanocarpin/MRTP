/*
Copyright 2023 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
debugWITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <chrono>

#define NONE 0
#define SPIN 1
#define GO_TO_POSE 2
#define BACKUP 3
#define COMPUTE_PATH 4
#define FOLLOW_PATH 5
#define FOLLOW_WAYPOINTS 6

const char *server_names[] = { "None", "Spin", "Go to Pose", "Backup" , "Compute Path" , "Follow Path" , "Follow Waypoints"};

Navigator::Navigator(bool debug,bool verbose) : rclcpp::Node("navigator")
{
  initial_pose.header.frame_id = "map";
  initial_pose_received = false;
  this->debug = debug;
  this->verbose = verbose;
  current_executing = NONE;
  status = rclcpp_action::ResultCode::UNKNOWN;
  
  initial_pose_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",10);
  localization_pose_subscriber = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
    ("amcl_pose",10,std::bind(&Navigator::amcl_pose_callback,this,std::placeholders::_1));
  spin_client = rclcpp_action::create_client<nav2_msgs::action::Spin>(this,"spin");
  nav_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"navigate_to_pose");
  follow_path_client = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this,"follow_path");
  backup_client = rclcpp_action::create_client<nav2_msgs::action::BackUp>(this,"back_up");
  compute_path_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this,"compute_path_to_pose");
  follow_waypoints_client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this,"FollowWaypoints");
  
  clear_global_costmap_srv = create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
  clear_local_costmap_srv = create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
  
  get_local_costmap_srv = create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");
  get_global_costmap_srv = create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
  change_map_srv = create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

  feedback_ptr = nullptr;
  
  if (debug)
    RCLCPP_INFO(get_logger(),"Created instance of Navigator...");
}

Navigator::~Navigator() {
  CancelTask();
}

void Navigator::SetInitialPose(const geometry_msgs::msg::Pose::SharedPtr pose)
{
  if (debug)
    RCLCPP_INFO(get_logger(),"Inside SetInitialPose");
  initial_pose.pose.position = pose->position;
  initial_pose.pose.orientation = pose->orientation;
  
  initial_pose_received = false;
  set_initial_pose();
}

void Navigator::WaitUntilNav2Active()
{
  if (debug)
    RCLCPP_INFO(this->get_logger(),"Waiting for Nav2");
  
  wait_for_node_to_activate("amcl");
  wait_for_initial_pose();
  wait_for_node_to_activate("bt_navigator");

  if (debug)
    RCLCPP_INFO(this->get_logger(),"Nav2 is ready");
}


bool Navigator::Spin(double spin_dist)
{
  using namespace std::placeholders;
  if (debug)
    RCLCPP_INFO(get_logger(),"Waiting for Spin action server");
  spin_client->wait_for_action_server();
  auto goal_message = nav2_msgs::action::Spin::Goal();
  goal_message.target_yaw = spin_dist;

  current_executing = SPIN;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();


  send_goal_options.goal_response_callback =
    std::bind(&Navigator::generic_goal_response_callback<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr>>, this, _1);
  
  send_goal_options.feedback_callback =
   std::bind(&Navigator::generic_feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr,
	      const std::shared_ptr<const nav2_msgs::action::Spin::Feedback> >, this, _1, _2);
  
  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult>, this, _1);
  
  auto send_goal_future = spin_client->async_send_goal(goal_message,send_goal_options);
  rclcpp::spin_until_future_complete(get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();

  if ( goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"Spin request was rejected");
      current_executing = NONE;
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"Spin request was rejected");
    current_executing = NONE;
    return false;
  }
  future_spin = spin_client->async_get_result(goal_handle);

  return true;
}

bool Navigator::GoToPose(const geometry_msgs::msg::Pose::SharedPtr pose)
{
  using namespace std::placeholders;
  if (debug)
    RCLCPP_INFO(get_logger(),"Waiting for NavigateToGoal action server");
  nav_to_pose_client->wait_for_action_server();
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.pose.position = pose->position;
  goal_msg.pose.pose.orientation = pose->orientation;
  goal_msg.behavior_tree = "";

  current_executing = GO_TO_POSE;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&Navigator::generic_goal_response_callback<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>>, this, _1);
  
 send_goal_options.feedback_callback =
   std::bind(&Navigator::generic_feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
	     const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>>, this, _1, _2);
  
 send_goal_options.result_callback =
   std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>, this, _1);

 auto send_goal_future = nav_to_pose_client->async_send_goal(goal_msg,send_goal_options);
 rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
 auto goal_handle = send_goal_future.get();
 if (goal_handle != NULL ) {
   if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
     if (debug)
       RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
     current_executing = NONE;
     return false;
   }
 }
 else {
   if (debug)
     RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
   current_executing = NONE;    
   return false;
 }
 if (debug)
   RCLCPP_INFO(this->get_logger(),"GoToPose request was accepted.");
 future_go_to_pose = nav_to_pose_client->async_get_result(goal_handle);
 
 return true;
}

bool Navigator::FollowPath(const nav_msgs::msg::Path::SharedPtr path)
{
  using namespace std::placeholders;
  if (debug)
    RCLCPP_INFO(get_logger(),"Waiting for FollowPath action server");
  follow_path_client->wait_for_action_server();
  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = nav_msgs::msg::Path(*path);

  current_executing = FOLLOW_PATH;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&Navigator::generic_goal_response_callback<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr>>, this, _1);
  
  send_goal_options.feedback_callback =
    std::bind(&Navigator::generic_feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr,
	      const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>>, this, _1, _2);
  
  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult>, this, _1);

  auto send_goal_future = follow_path_client->async_send_goal(goal_msg,send_goal_options);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();
  if (goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"FollowPath request was rejected");
      current_executing = NONE;
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"FollowPath request was rejected");
    current_executing = NONE;    
    return false;
  }
  future_follow_path = follow_path_client->async_get_result(goal_handle);

  return true;
}


bool Navigator::FollowWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  using namespace std::placeholders;
  if (debug)
    RCLCPP_INFO(get_logger(),"Waiting for FollowWaypoint action server");
  follow_waypoints_client->wait_for_action_server();
  auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
  goal_msg.poses = poses;

  current_executing = FOLLOW_WAYPOINTS;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&Navigator::generic_goal_response_callback<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr>>, this, _1);
  
  send_goal_options.feedback_callback =
    std::bind(&Navigator::generic_feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr,
	      const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback>>, this, _1, _2);
  
  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult>, this, _1);

  auto send_goal_future = follow_waypoints_client->async_send_goal(goal_msg,send_goal_options);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();
  if (goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"FollowWaypoints request was rejected");
      current_executing = NONE;
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"FollowWaypoints request was rejected");
    current_executing = NONE;    
    return false;
  }
  future_follow_waypoints = follow_waypoints_client->async_get_result(goal_handle);

  return true;
}



bool Navigator::Backup(double backup_dist,double backup_speed)
{
  using namespace std::placeholders;
  if (debug)
    RCLCPP_INFO(get_logger(),"Waiting for backup action server");
  backup_client->wait_for_action_server();
  auto goal_msg = nav2_msgs::action::BackUp::Goal();
  goal_msg.target.x = -backup_dist; 
  goal_msg.speed = backup_speed;

  current_executing = BACKUP;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::BackUp>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&Navigator::generic_goal_response_callback<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr>>, this, _1);
  
  send_goal_options.feedback_callback =
    std::bind(&Navigator::generic_feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr,
	      const std::shared_ptr<const nav2_msgs::action::BackUp::Feedback>>, this, _1, _2);  
  
  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::WrappedResult>, this, _1);

  auto send_goal_future = backup_client->async_send_goal(goal_msg,send_goal_options);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();
  if (goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"BackUp request was rejected");
      current_executing = NONE;
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"BackUp request was rejected");
    current_executing = NONE;
    return false;
  }
  future_backup = backup_client->async_get_result(goal_handle);

  
  return true;
}

std::shared_ptr<nav_msgs::msg::Path> Navigator::GetPath(const geometry_msgs::msg::Pose::SharedPtr pose) {
  if  ( get_path_internal(pose)==false )
    return nullptr; // if call fails, return null pointer
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_compute_path_to_pose); // wait for result
  auto result = future_compute_path_to_pose.get();
  current_executing = NONE;
  return std::make_shared<nav_msgs::msg::Path>(result.result->path); 
}

bool Navigator::get_path_internal(const geometry_msgs::msg::Pose::SharedPtr pose){

  using namespace std::placeholders;
  if (debug)
    RCLCPP_INFO(get_logger(),"Waiting for compute_path_to_pose action server");
  compute_path_to_pose_client->wait_for_action_server();
  
  current_executing = COMPUTE_PATH;    
  auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
  goal_msg.pose.pose.position = pose->position;
  goal_msg.pose.pose.orientation = pose->orientation;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&Navigator::generic_goal_response_callback<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr>>, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Navigator::generic_feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr,
	      const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback>>, this, _1, _2);  

  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult>, this, _1);

  auto send_goal_future = compute_path_to_pose_client->async_send_goal(goal_msg,send_goal_options);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();

  if (goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"ComputePathToPose request was rejected");
      current_executing = NONE;
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"ComputePathToPose request was rejected");
    current_executing = NONE;
    return false;
  }
  future_compute_path_to_pose = compute_path_to_pose_client->async_get_result(goal_handle);

  return true;
}

bool Navigator::IsTaskComplete() {
  using namespace std::chrono_literals;
  
  if ( current_executing == NONE  )
    return true;
  if (verbose)
    RCLCPP_INFO(get_logger(),"IsTaskComplete waiting for %s",server_names[current_executing]);
  switch(current_executing) {
  case SPIN: {
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult>>(future_spin);
    break;
  }
  case GO_TO_POSE: {
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>>(future_go_to_pose);
    break;
  }
  case BACKUP: {
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::WrappedResult>>(future_backup);
    break;
  }
  case COMPUTE_PATH: { // useless; but for completeness..
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult>>(future_compute_path_to_pose);
    break;
  }
  case FOLLOW_PATH: { 
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult>>(future_follow_path);
    break;
  }
  case FOLLOW_WAYPOINTS: { 
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult>>(future_follow_waypoints);
    break;
  }
  default:
    RCLCPP_ERROR(get_logger(),"Undefined task in progress");
  }
  if (verbose)
    RCLCPP_INFO(get_logger(),"Leaving IsTaskComplete");
  if  ( current_executing == NONE )
    return true;
  else
    return false;
}

template<typename T>
void Navigator::check_complete(T future) {
  using namespace std::chrono_literals;
  if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(),future,0.1s) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    status = result.code;
  }
}

void Navigator::ClearGlobalCostmap() {
  using namespace std::chrono_literals;
  while ( ! clear_global_costmap_srv->wait_for_service(1s) )
    if ( debug )
      RCLCPP_INFO(get_logger(),"Clear global costmap service not available, waiting...");
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto response = clear_global_costmap_srv->async_send_request(request);
  auto future_return = rclcpp::spin_until_future_complete(this->get_node_base_interface(),response);
  if (debug) {
    if ( future_return == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(get_logger(),"Clear global costmap service successfully called");
    else
      RCLCPP_INFO(get_logger(),"Clear global costmap service failed");
  }
}

void Navigator::ClearLocalCostmap() {
  using namespace std::chrono_literals;
  while ( ! clear_local_costmap_srv->wait_for_service(1s) )
    if ( debug )
      RCLCPP_INFO(get_logger(),"Clear local costmap service not available, waiting...");
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto response = clear_local_costmap_srv->async_send_request(request);
  auto future_return = rclcpp::spin_until_future_complete(this->get_node_base_interface(),response);
   if (debug) {
    if ( future_return == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(get_logger(),"Clear local costmap service successfully called");
    else
      RCLCPP_INFO(get_logger(),"Clear local costmap service failed");
  }
}

void Navigator::ClearAllCostmaps() {
  ClearLocalCostmap();
  ClearGlobalCostmap();
}

std::shared_ptr<nav2_msgs::msg::Costmap> Navigator::GetGlobalCostmap() {
  using namespace std::chrono_literals;
  while ( ! get_global_costmap_srv->wait_for_service(1s) )
    if ( debug )
      RCLCPP_INFO(get_logger(),"Get global costmap service not available, waiting...");
  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto future_get_map = get_global_costmap_srv->async_send_request(request);
  auto future_return = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_get_map);

  if (debug) {
    if ( future_return == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(get_logger(),"Get global costmap service successfully called");
    else
      RCLCPP_INFO(get_logger(),"Get global costmap service failed");
  }
  if ( future_return == rclcpp::FutureReturnCode::SUCCESS ) {
    auto result = future_get_map.get();
    return std::make_shared<nav2_msgs::msg::Costmap>(result->map);
  }
  else
    return nullptr;
}

std::shared_ptr<nav2_msgs::msg::Costmap> Navigator::GetLocalCostmap() {
  using namespace std::chrono_literals;
  while ( ! get_local_costmap_srv->wait_for_service(1s) )
    if ( debug )
      RCLCPP_INFO(get_logger(),"Get local costmap service not available, waiting...");
  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto future_get_map = get_local_costmap_srv->async_send_request(request);
  auto future_return = rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_get_map);

  if (debug) {
    if ( future_return == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(get_logger(),"Get local costmap service successfully called");
    else
      RCLCPP_INFO(get_logger(),"Get local costmap service failed");
  }
  if ( future_return == rclcpp::FutureReturnCode::SUCCESS ) {
    auto result = future_get_map.get();
    return std::make_shared<nav2_msgs::msg::Costmap>(result->map);
  }
  else
    return nullptr;
}

void Navigator::ChangeMap(const std::string& map_filepath) {
  using namespace std::chrono_literals;
  while ( ! change_map_srv->wait_for_service(1s) )
    if ( debug )
      RCLCPP_INFO(get_logger(),"Change map service not available, waiting...");
  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_filepath;
  auto future_change_map = change_map_srv->async_send_request(request);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_change_map);
  auto result = future_change_map.get();
  if (debug) {
    if (result->result != result->RESULT_SUCCESS)
      RCLCPP_INFO(get_logger(),"Change map request failed");
    else
      RCLCPP_INFO(get_logger(),"Change map request was successful");
  }
  if (result->result == result->RESULT_SUCCESS)
    status = rclcpp_action::ResultCode::SUCCEEDED;
  else
    status = rclcpp_action::ResultCode::CANCELED;
}

void Navigator::CancelTask() {
  
  if (current_executing == NONE)
    return;
  if (debug)
    RCLCPP_INFO(get_logger(),"Canceling current task %s",server_names[current_executing]);
  
   switch(current_executing) {
   case SPIN: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr>(spin_client);
     break;
   }
   case GO_TO_POSE: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr>(nav_to_pose_client);
     break;
   }
   case BACKUP: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr>(backup_client);
     break;
   }
   case COMPUTE_PATH: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr>(compute_path_to_pose_client);
     break;
   }
   case FOLLOW_PATH: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr>(follow_path_client);
     break;
   }
   case FOLLOW_WAYPOINTS: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr>(follow_waypoints_client);
     break;
   } 
   default:
     RCLCPP_ERROR(get_logger(),"Undefined task in progress");
   }
   if (status == rclcpp_action::ResultCode::CANCELED) // if cancel request was accepted
     while (current_executing != NONE ) // waits for the task to end
       rclcpp::spin_some(get_node_base_interface());
}

void Navigator::set_initial_pose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.pose.pose = initial_pose.pose;
  msg.header.frame_id = initial_pose.header.frame_id;
  msg.header.stamp = get_clock()->now();
  msg.header.stamp.sec += 1; // shift 1 second ahead -- a hack
  initial_pose_publisher->publish(msg);
}

void Navigator::wait_for_initial_pose()
{
  if (debug)
    RCLCPP_INFO(this->get_logger(),"Waiting for initial pose");
  using namespace std::chrono_literals;
  set_initial_pose();
  while ( ! initial_pose_received ) {
    rclcpp::spin_some(this->get_node_base_interface());
  }
}

void Navigator::wait_for_node_to_activate(const std::string& node_name)
{
  using namespace std::chrono_literals;

  if (debug)
    RCLCPP_INFO(this->get_logger(),"Waiting for %s",node_name.c_str());
  
  std::string node_service = "/" + node_name + "/get_state";
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client =
    this->create_client<lifecycle_msgs::srv::GetState>(node_service);

  while (! client->wait_for_service(1s)) 
    RCLCPP_INFO(this->get_logger(),"Waiting for %s to be available",node_name.c_str());

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  std::string state = "unknown";
  while ( state != "active" ) {
    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(),future);
    if ( future.get() ) {
      state = future.get()->current_state.label;
    }
  }
}

void Navigator::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
  if (verbose)
    RCLCPP_INFO(get_logger(),"Received pose from amcl");
  initial_pose_received = true;
}


template<typename T>
void Navigator::generic_goal_response_callback(T future)
{
  if (debug) {
    RCLCPP_INFO(this->get_logger(), "In task %s response callback",server_names[current_executing]);
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
}

template<typename T>
void Navigator::generic_result_callback(const T result)
{
  status = result.code;
  if (debug) {
    RCLCPP_INFO(get_logger(),"Task %s got result",server_names[current_executing]);
    print_result_diagnostic();
  }
  current_executing = NONE;
}

template<typename T,typename Q>
void Navigator::generic_feedback_callback(T,Q feedback) {
  if (verbose) 
    RCLCPP_INFO(get_logger(),"In feedback executing action %s",server_names[current_executing]);
  feedback_ptr = feedback;
}

template<typename T>
void Navigator::cancel_generic_goal(T client)
{
  auto future = client->async_cancel_all_goals();
  rclcpp::spin_until_future_complete(get_node_base_interface(),future);
  auto result = future.get();
  auto return_code = result->return_code;
  if (return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) 
    status = rclcpp_action::ResultCode::CANCELED;
  else
    status = rclcpp_action::ResultCode::UNKNOWN;
}



void Navigator::print_result_diagnostic() {
  switch (status) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_ERROR(this->get_logger(), "Goal was achieved");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
}
