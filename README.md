# MRTP
This repository contains material related to the textbook "Mobile Robotics Theory and Practice."

See https://github.com/stefanocarpin/MRTP/wiki/Installation-Intructions for instructions on what to install to run the examples presented in the textbook.

To build the code examples, download or clone the repository, move to the folder MRTP/MRTP, and run

     colcon build

Important: to run the examples using Gazebo, you need to set the environment variable GAZEBO_MODEL_PATH. Assuming you downloaded the files in your home, you should set
     export GAZEBO_MODEL_PATH=~MRTP/MRTP/src/gazeboenvs/models
