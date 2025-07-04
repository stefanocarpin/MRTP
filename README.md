# MRTP
This repository contains material related to the textbook "Mobile Robotics Theory and Practice" (aka MRTP).

See https://github.com/stefanocarpin/MRTP/wiki/Installation-Intructions for instructions on what to install to run the examples presented in the textbook.

To build the code examples, clone the repository, 

    git clone https://github.com/stefanocarpin/MRTP

move to the folder MRTP/MRTP, and run

     colcon build
     
Important: as pointed out in the documentation, to use ROS you must first source the setup. See also https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#setup-environment

## Docker
For use in Docker, we've provided a container for each simulation environment. 
To access these, simply `make bash ROBOT=<your_desired_robot>`.
Currently, we support `husky` and `turtlebot`, as desired robot environments.
We recommend `turtlebot` if you're just wanting to play around in the ROS2 environment without any simulation.

If you've successfully pulled the image and run with `make bash`, you may now follow along with the book from this point.