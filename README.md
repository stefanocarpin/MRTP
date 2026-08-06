# MRTP
This repository contains material related to the textbook "Mobile Robotics: Theory and Practice" (aka MRTP -- https://robotics.ucmerced.edu/MRTP).
If you cite the book, please use the following:

    @misc{MRTP,
    author = {Stefano Carpin},
    title = {Mobile Robotics: Theory and Practice},
    url = {http://robotics.ucmerced.edu/MRTP},
    year={2025}
    } 

## Sample Code
All code presented in the textbook (and more) is available in this github repository. To run it you will need ROS2 Jazzy. See below for installation instructions matching your operating system.

## Linux - Ubuntu 24

If you run Ubuntu 24 (noble), the most straightforward path is to install ROS 2 Jazzy native packages.
See https://github.com/stefanocarpin/MRTP/wiki/Installation-Intructions for instructions on what to install to run the examples presented in the textbook.

Once the installation is complete, to build the code examples open a shell, clone the repository, 

    git clone https://github.com/stefanocarpin/MRTP

move to the folder MRTP/MRTP, and run

     colcon build
     
Important: as pointed out in the documentation, to use ROS you must first source the setup. See also https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#setup-environment

## Other operating systems (Win, Mac, or other Linux distributions)
If for some reason you cannot install ROS2 Jazzy on your machine (working with Windows or Mac or a different Linux Distribution) you can use a container to provide the necessary environment through Docker.

[See Docker README](docker/README.md).

Note that you can also use the docker distribution on a system using Ubuntu Noble, though this will make things slower.  Some students in the past reported being able to run everything under Windows using [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) but this is not supported.
