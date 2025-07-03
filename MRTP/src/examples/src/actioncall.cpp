/*
Copyright 2025 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
debug WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclcpp/rclcpp.hpp> 
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <action_msgs/msg/goal_status.hpp>

class ActionCaller : public rclcpp::Node {

public:
  using GoalHandleSpin =
   rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>;
  ActionCaller() : Node("actioncaller") {
    spin_client = rclcpp_action::create_client<nav2_msgs::action::Spin>
      (this,"spin");
    rotating = false;
  }

  void RotateRobot(double target_yaw) {
    using namespace std::placeholders;
    spin_client->wait_for_action_server();
    auto goal_message = nav2_msgs::action::Spin::Goal();
    goal_message.target_yaw = target_yaw;
    
    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionCaller::response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionCaller::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionCaller::result_callback, this, _1);
  
    auto send_goal_future =
      spin_client->async_send_goal(goal_message,send_goal_options);
    rclcpp::spin_until_future_complete(get_node_base_interface(),
				       send_goal_future);
  }

  bool RobotIsRotating() { return rotating; }
  
private:
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client;
  bool rotating;
  void response_callback(const GoalHandleSpin::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by action server");
      rotating = true;
    }
  }

  void result_callback(const rclcpp_action::ClientGoalHandle
		       <nav2_msgs::action::Spin>::WrappedResult & result)
  {
    if (int(result.code) == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      RCLCPP_INFO(get_logger(),"Rotation completed with success");
    rotating = false;
  }
  
  void feedback_callback(rclcpp_action::ClientGoalHandle
			 <nav2_msgs::action::Spin>::SharedPtr,
			 const std::shared_ptr
			 <const nav2_msgs::action::Spin::Feedback> f)
  {
    RCLCPP_INFO(get_logger(),"Angle %f",f->angular_distance_traveled);
  }
};

int main(int argc,char **argv) {

  rclcpp::init(argc,argv); // initialize the ROS subsystem
  ActionCaller node; // create node
  node.RotateRobot(2.0); // turn ~ 114 degrees
  while ( node.RobotIsRotating() )
    rclcpp::spin_some(node.get_node_base_interface());
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
