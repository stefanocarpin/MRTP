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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>  // needed for actions
#include <rclcpp_action/client_goal_handle.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <string>

class Navigator : public rclcpp::Node {

public:

  Navigator(bool=false,bool=false);
  
  ~Navigator();
  
  void SetInitialPose(const geometry_msgs::msg::Pose::SharedPtr);
  void WaitUntilNav2Active();
  bool Spin(double=1.57);
  bool GoToPose(const geometry_msgs::msg::Pose::SharedPtr);
  bool IsTaskComplete();
  rclcpp_action::ResultCode GetResult() { return status; }
  void CancelTask();
  
  
private:
  geometry_msgs::msg::PoseStamped initial_pose;
  bool initial_pose_received;
  bool debug;
  bool verbose;
  int current_executing;
  rclcpp_action::ResultCode status;
  
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult> future_spin; 	
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> future_go_to_pose; 	
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_subscriber;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;

  
  void set_initial_pose();
  void wait_for_initial_pose();
  void wait_for_node_to_activate(const std::string);
  
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  
  void spin_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr>);
  // void spin_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult &);
  void spin_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr,
			      const std::shared_ptr<const nav2_msgs::action::Spin::Feedback>);
  
  void go_to_pose_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>);
  //void go_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &);
  void go_to_pose_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
			      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>);
  
  void print_result_diagnostic(void);
  template<typename T>
  void check_complete(T);
  template<typename T>
  void generic_result_callback(T);
  template<typename T>
  void cancel_generic_goal(T);
};
