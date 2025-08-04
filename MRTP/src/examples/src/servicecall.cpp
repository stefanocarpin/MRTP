/*
Copyright 2024 Stefano Carpin

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
#include <nav2_msgs/srv/save_map.hpp>

int main(int argc,char **argv) {

    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeh;
    nodeh = rclcpp::Node::make_shared("servicecall"); // create node

    // create client for service
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr client =
    nodeh->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
    
    // wait indefinitely for service to become available
    while (! client->wait_for_service()) 
      RCLCPP_INFO(nodeh->get_logger(),"Waiting for service to be available");
    
    // create a request object for the SetCameraInfo service
    auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
    request->map_topic = "/map";
    request->map_url = "./mymap";
    request->image_format = "png";
    request->map_mode = "trinary";
    request->free_thresh = 0.2f;
    request->occupied_thresh = 0.8f;
    // send request to server
    auto response = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(nodeh, response) ==
	rclcpp::FutureReturnCode::SUCCESS) {  // waited and got success?
      // print result
      RCLCPP_INFO(nodeh->get_logger(), "Success? %d",
		  response.get()->result);
    }
    else // Error: 
      RCLCPP_ERROR(nodeh->get_logger(),
		    "Error calling service save_map");

    rclcpp::shutdown();
    return 0;

}
