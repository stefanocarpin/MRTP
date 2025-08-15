/*
Copyright 2025 Stefano Carpin

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
#include <std_srvs/srv/empty.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc,char **argv) {

    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    nodeh = rclcpp::Node::make_shared("resetparticles"); // create node

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
    nodeh->create_client<std_srvs::srv::Empty>
                           ("/reinitialize_global_localization");
    // wait...
    while (! client->wait_for_service()) 
      RCLCPP_INFO(nodeh->get_logger(),"Waiting for service to be available");

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto response = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(nodeh, response) ==
	rclcpp::FutureReturnCode::SUCCESS) {  
      RCLCPP_INFO(nodeh->get_logger(), "Particles reset");
    }
    else // Error: 
      RCLCPP_ERROR(nodeh->get_logger(),"Error while resetting particles");

    rclcpp::shutdown();
    return 0; 
}
