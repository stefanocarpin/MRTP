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

#include <rclcpp_action/rclcpp_action.hpp>  
#include <rclcpp_action/client_goal_handle.hpp>

#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/load_map.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>

#include <string>
#include <vector>

class Navigator : public rclcpp::Node {

public:

  Navigator(bool=false,bool=false);
  
  ~Navigator();
  
  void SetInitialPose(const geometry_msgs::msg::Pose::SharedPtr);
  void WaitUntilNav2Active();
  bool Spin(double=1.57);
  bool GoToPose(const geometry_msgs::msg::Pose::SharedPtr);
  bool FollowPath(const nav_msgs::msg::Path::SharedPtr);
  bool FollowWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>&);
  bool Backup(double=0.15,double=0.25);
  std::shared_ptr<nav_msgs::msg::Path> GetPath(const geometry_msgs::msg::Pose::SharedPtr);
  bool IsTaskComplete();
  rclcpp_action::ResultCode GetResult() { return status; }
  void CancelTask();
  std::shared_ptr<const void> GetFeedback() const { return feedback_ptr; };
  void ClearGlobalCostmap();
  void ClearLocalCostmap();
  void ClearAllCostmaps();
  std::shared_ptr<nav2_msgs::msg::Costmap> GetGlobalCostmap();
  std::shared_ptr<nav2_msgs::msg::Costmap> GetLocalCostmap();
  void ChangeMap(const std::string&);
  
  
private:
  geometry_msgs::msg::PoseStamped initial_pose;
  bool initial_pose_received;
  bool debug;
  bool verbose;
  int current_executing;
  rclcpp_action::ResultCode status;
  
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult> future_spin; 	
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> future_go_to_pose;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult> future_follow_path;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult> future_follow_waypoints;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::WrappedResult> future_backup;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult> future_compute_path_to_pose;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_subscriber;
  
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client;

  std::shared_ptr<const void> feedback_ptr;
  
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_srv;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_srv;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_local_costmap_srv;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_global_costmap_srv;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr change_map_srv;
  
  
  void set_initial_pose();
  void wait_for_initial_pose();
  void wait_for_node_to_activate(const std::string&);
  
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  bool get_path_internal(const geometry_msgs::msg::Pose::SharedPtr);
  bool smooth_internal(const geometry_msgs::msg::Pose::SharedPtr,float,bool);

  void print_result_diagnostic(void);

  template<typename T>
  void check_complete(T);
  
  template<typename T>
  void generic_result_callback(T);

  template<typename T>
  void generic_goal_response_callback(T);

  template<typename T,typename Q>
  void generic_feedback_callback(T,Q);
  
  template<typename T>
  void cancel_generic_goal(T);

  
};
