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
#include <nav2_msgs/action/spin.hpp>
#include <rclcpp_action/rclcpp_action.hpp> 

class ActionCaller : public rclcpp::Node {

public:
  ActionCaller() : Node("actioncaller") {
    spin_client = rclcpp_action::create_client<nav2_msgs::action::Spin>
      (this,"spin");
    spinning = false;
  }

  void Spin(double target_yaw) {
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

  bool isSpinning() { return spinning; }
  
private:
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client;
  bool spinning;
  void response_callback(std::shared_future<rclcpp_action::ClientGoalHandle
			 <nav2_msgs::action::Spin>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by spin server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by spin server");
      spinning = true;
    }
  }

  void result_callback(const rclcpp_action::ClientGoalHandle
		       <nav2_msgs::action::Spin>::WrappedResult result)
  {
    RCLCPP_INFO(get_logger(),"Spin got result %d",result.code);
    spinning = false;
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
  node.Spin(2.0); // turn ~ 114 degrees
  while ( node.isSpinning() )
    rclcpp::spin_some(node.get_node_base_interface());
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
