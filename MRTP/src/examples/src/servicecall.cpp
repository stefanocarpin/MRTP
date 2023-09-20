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
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

rclcpp::Node::SharedPtr nodeh;
sensor_msgs::msg::CameraInfo buffer;
bool initialized = false;

// process CameraInfo messages
void processCamera(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
  // store it in the buffer only the first time it is received
  if ( not initialized ) {
    RCLCPP_INFO(nodeh->get_logger(),"Received initial configuration");
    buffer = *msg;
    initialized = true;
  }
}

int main(int argc,char **argv) {

    rclcpp::init(argc,argv);

    nodeh = rclcpp::Node::make_shared("servicecall"); // create node
    auto sub = nodeh->create_subscription<sensor_msgs::msg::CameraInfo>
      ("/camera/camera_info",10,&processCamera); // create subscriber

    while (! initialized ) // wait for configuration to come in
      rclcpp::spin_some(nodeh);

    // create client for service
    rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr client =
    nodeh->create_client<sensor_msgs::srv::SetCameraInfo>("/set_camera_info");
    
    // wait indefinitely for service to become available
    while (! client->wait_for_service()) 
      RCLCPP_INFO(nodeh->get_logger(),"Waiting for service to be available");
    
    // create a request object for the SetCameraInfo service
    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::
                   				    Request>();
    // copy existing configuration
    request->camera_info = buffer;
    // set hypothetical region of interest
    request->camera_info.roi.x_offset = 10;
    request->camera_info.roi.y_offset = 20;
    request->camera_info.roi.height = 30;
    request->camera_info.roi.width = 50;
    request->camera_info.roi.do_rectify = true;
    // send request to server
    auto response = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(nodeh, response) ==
	rclcpp::FutureReturnCode::SUCCESS) {  // waited and got success?
      // print result
      RCLCPP_INFO(nodeh->get_logger(), "Success? %d",
		  response.get()->success);
      RCLCPP_INFO(nodeh->get_logger(), "Status_message: %s",
		  response.get()->status_message.c_str());
    }
    else // Error: 
      RCLCPP_ERROR(nodeh->get_logger(),
		    "Error calling service set_camera_info");
        
    rclcpp::shutdown();
    return 0;

}
