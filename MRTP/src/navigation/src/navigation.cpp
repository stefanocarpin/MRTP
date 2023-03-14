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

#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <navigation/navigation.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>

#define NONE 0
#define SPIN 1
#define GO_TO_POSE 2

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

  if (debug)
    RCLCPP_INFO(get_logger(),"Created instance of Navigator...");
}

Navigator::~Navigator() {

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


  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Navigator::spin_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Navigator::spin_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult>, this, _1);
  
  auto send_goal_future = spin_client->async_send_goal(goal_message,send_goal_options);
  rclcpp::spin_until_future_complete(get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();

  if ( goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"Spin request was rejected");
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"Spin request was rejected");
    return false;
  }
  future_spin = spin_client->async_get_result(goal_handle);
  current_executing = SPIN;
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

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Navigator::go_to_pose_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Navigator::go_to_pose_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&Navigator::generic_result_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>, this, _1);

  auto send_goal_future = nav_to_pose_client->async_send_goal(goal_msg,send_goal_options);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
  auto goal_handle = send_goal_future.get();
  if (goal_handle != NULL ) {
    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      if (debug)
	RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
      return false;
    }
  }
  else {
    if (debug)
      RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
    return false;
  }
  future_go_to_pose = nav_to_pose_client->async_get_result(goal_handle);
  current_executing = GO_TO_POSE;
  return true;
}

bool Navigator::IsTaskComplete() {
  using namespace std::chrono_literals;
  
  if ( current_executing == NONE  )
    return true;
  if (verbose)
    RCLCPP_INFO(get_logger(),"IsTaskComplete waiting for %d",current_executing);
  switch(current_executing) {
  case 1: {
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult>>(future_spin);
    break;
  }
  case 2: {
    check_complete<std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>>(future_go_to_pose);
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


void Navigator::CancelTask() {
  
  if (current_executing == NONE)
    return;
  if (debug)
    RCLCPP_INFO(get_logger(),"Canceling current task %d",current_executing);
  
   switch(current_executing) {
   case 1: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr>(spin_client);
     break;
   }
   case 2: {
     cancel_generic_goal<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr>(nav_to_pose_client);
     break;
   }
   default:
     RCLCPP_ERROR(get_logger(),"Undefined task in progress");
   }
   current_executing = NONE;
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

void Navigator::wait_for_node_to_activate(const std::string node_name)
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

void Navigator::spin_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr> future)
{
  if (debug) {
    RCLCPP_INFO(get_logger(), "In Spin response callback");
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by spin server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by spin server, waiting for result");
    }
  }
}

void Navigator::spin_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr,
				       const std::shared_ptr<const nav2_msgs::action::Spin::Feedback>)
{
  if (verbose) {
    RCLCPP_INFO(get_logger(),"In spin feedback");
    RCLCPP_INFO(get_logger(),"Leaving spin feedback");
  }
}

void Navigator::go_to_pose_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
{
  if (debug) {
    RCLCPP_INFO(this->get_logger(), "In the GoToPose response callback");
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by navigate_to_pose server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by navigate_to_pose server, waiting for result");
    }
  }
}


template<typename T>
void Navigator::generic_result_callback(const T result)
{
  status = result.code;
  if (debug) {
    RCLCPP_INFO(get_logger(),"Task %d got result",current_executing);
    print_result_diagnostic();
  }
  current_executing = NONE;
}

template<typename T>
void Navigator::cancel_generic_goal(T client)
{
  auto future = client->async_cancel_all_goals();
  rclcpp::spin_until_future_complete(get_node_base_interface(),future);
}

void Navigator::go_to_pose_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
				       const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>)
{
  if (verbose)
    RCLCPP_INFO(get_logger(), "In GoToPose feedback");
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
