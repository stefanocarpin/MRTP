# MRTP
This repository contains material related to the textbook "Mobile Robotics Theory and Practice" (aka MRTP).

See https://github.com/stefanocarpin/MRTP/wiki/Installation-Intructions for instructions on what to install to run the examples presented in the textbook.

To build the code examples, clone the repository, 

    git clone https://github.com/stefanocarpin/MRTP

move to the folder MRTP/MRTP, and run

     colcon build

Important: to run the examples using Gazebo, you need to set the environment variable GAZEBO_MODEL_PATH. Assuming you cloned the repository in your home, you should set it as follows:

     export GAZEBO_MODEL_PATH=~/MRTP/MRTP/src/gazeboenvs/models:$GAZEBO_MODEL_PATH
     

