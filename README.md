# MRTP
This repository contains material related to the textbook "Mobile Robotics: Theory and Practice" (aka MRTP -- https://robotics.ucmerced.edu/MRTP).
If you cite the book, please use the following:

    @misc{MRTP,
    author = {Stefano Carpin},
    title = {Mobile Robotics: Theory and Practice},
    url = {http://robotics.ucmerced.edu/MRTP},
    year={2025}
    } 

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

You can pull and run the image with `make bash ROBOT=<your_desired_robot>` and may now follow along with the book from this point.
If you want to run multiple shells within the container, run `make shell ROBOT=<your_desired_robot>` from outside after you've spawned the container.

### Husky
For Clearpath Husky, we've provided a script to spawn the Gazebo simulation using their provided environment.
To run, execute `make husky` once inside your container.

### Turtlebot
For Turtlebot4, we've provided a script to spawn the Gazebo simulation using their provided environment.
To run, execute `make turtlebot` once inside your container.
