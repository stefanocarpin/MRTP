#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>

rclcpp::Node::SharedPtr nodeh;

void matrixCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  int nrows =  msg->layout.dim[0].size;
  int ncols =  msg->layout.dim[1].size;
  RCLCPP_INFO(nodeh->get_logger(),"Received %dx%d matrix",nrows,ncols);
    std::vector<std::vector<int> > matrix(nrows, std::vector<int>(ncols));
    for(int i = 0 ; i < nrows ; i++)
	for (int j = 0 ; j < ncols ; j++) 
	    matrix[i][j] = msg->data[i*msg->layout.dim[1].stride + j];
}


int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  nodeh = rclcpp::Node::make_shared("matrixsubscriber");

  auto sub = nodeh->create_subscription<std_msgs::msg::Int32MultiArray>
    ("matrixint",10,&matrixCallback);

  rclcpp::spin(nodeh);

}
