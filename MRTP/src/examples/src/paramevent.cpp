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
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

rclcpp::Node::SharedPtr nodeh;

// callback function called when a parameter event is received
void processEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
  if (msg->node == "/paramevent") {   // is this event for the current node?
    if ( msg->changed_parameters.size() > 0 ) { // any parameter changed?
      // scan all changed parameters
      for ( unsigned int i = 0 ; i < msg->changed_parameters.size() ; i++ )
	if (msg->changed_parameters[i].name == "order"){ // changed order?
	  if(msg->changed_parameters[i].value.type == // is it an integer?
	     rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
	    int order;  // get it
	    order = nodeh->get_parameter("order").get_parameter_value().
	      get<int>();
	    // print it
	    RCLCPP_INFO(nodeh->get_logger(), "Parameter order has changed");
	    RCLCPP_INFO(nodeh->get_logger(), "New value value: %d", order);
	  }
      }
    }
  }
}

int main(int argc,char **argv) {
  
  rclcpp::init(argc,argv);
  nodeh = rclcpp::Node::make_shared("paramevent");
  // subscriber to be notified of changes to parameters
   auto sub = nodeh->create_subscription<rcl_interfaces::msg::ParameterEvent>
    ("parameter_events",10,&processEvent);
  // declare a parameter of type integer called order
  nodeh->declare_parameter<int>("order",5);
  int order;
  order = nodeh->get_parameter("order").get_parameter_value().get<int>();
  RCLCPP_INFO(nodeh->get_logger(), "Initial value: %d", order);
  RCLCPP_INFO(nodeh->get_logger(), "Waiting...");
  // just wait for events to happen...
  rclcpp::spin(nodeh);
  
 
}

