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
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>

rclcpp::Node::SharedPtr nodeh;

// callback function called when a matrix is received
void matrixCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  // first extracts dimensions from message 
  int nrows =  msg->layout.dim[0].size;
  int ncols =  msg->layout.dim[1].size;
  RCLCPP_INFO(nodeh->get_logger(),"Received %dx%d matrix",nrows,ncols);
  // allocate local matrix to copy received data
  std::vector<std::vector<int> > matrix(nrows, std::vector<int>(ncols));
  for(int i = 0 ; i < nrows ; i++)
    for (int j = 0 ; j < ncols ; j++)
      // note how element (i,j) is extracted and copied in matrix[i][j]
      matrix[i][j] = msg->data[i*msg->layout.dim[1].stride + j];
}


int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  nodeh = rclcpp::Node::make_shared("matrixsubscriber"); // create node
  // create subscriber and register callback function
  auto sub = nodeh->create_subscription<std_msgs::msg::Int32MultiArray>
    ("matrixint",10,&matrixCallback);

  rclcpp::spin(nodeh);

}
