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
If for some reason you can't install ROS2 on your machine (working with Windows or Mac) you can use a container to provide the necessary environment through Docker.

[See Docker README](docker/README.md).