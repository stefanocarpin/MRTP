# Docker
# Installation
Setting yourself up with Docker usually means you don't have a Linux machine or you prefer to manage your workspace in a container.
In either case, you can find the instructions on how to install Docker for your preferred OS [here](https://docs.docker.com/desktop/?_gl=1*f1d4un*_gcl_au*MTQ5ODc3MjEwOC4xNzU2NzEyODAz*_ga*MjU1MDAwMDU4LjE3NTY3MTI3OTk.*_ga_XJWPQMJYHQ*czE3NTY3MTI3OTgkbzEkZzEkdDE3NTY3MTI4MDMkajU1JGwwJGgw).

## Usage
For use in Docker, we've provided a container for each simulation environment. 
1. First `make build-image`. This will create the container image.
2. Next, pull the image for using noVNC. This can be done by running `make vnc`. See [noVNC](#novnc).
3. To access this image, simply `make bash`.
4. If you want to run multiple shells within the container, run `make shell` from outside after you've spawned the container.

From within you can follow instructions set out in the book such as `colcon build` and `source install/setup.bash`.

## noVNC
noVNC will be the method from which you will visualize all of your simulations. 
This forwards all display information outside a Docker image and onto a web application.
After you've set up the above, log into your browser of choice and go to http://localhost:8080/vnc.html. 
From here you can view all of your Gazebo/RViz simulations.

## Windows Users
Windows requires a few extra steps.
1. Install VcXsrv (Windows X Server)

   """
      Download and install VcXsrv from:
      https://github.com/marchaesen/vcxsrv/releases/download/21.1.10/vcxsrv-64.21.1.10.0.installer.exe

      After installation: - Launch “XLaunch” from Start Menu - Choose:
      Multiple Windows >> Next - Start no client >> Next - (Optional) Disable
      access control >> Next - Finish. You should see the X icon in the system
      tray.
   """
2. Install prerequisites

choco install make -y

3. Clone MRTP repo

4. 3. Build ROS2 Jazzy image

make build-image

4. Start container

make bash

5. Inside container: set DISPLAY for GUI (add to ~/.bashrc for persistence)

export DISPLAY=host.docker.internal:0.0

6. Verify ROS2 works

ros2 -h && echo $ROS_DISTRO

7. Build workspace (only first time or after code changes)

mkdir -p /MRTP/src && cd /MRTP/src && git clone
https://github.com/stefanocarpin/MRTP cd /MRTP && colcon build
–packages-skip unsorted source install/setup.bash


8. Test GUI apps

sudo apt-get update && sudo apt-get install -y x11-apps # (only once, for xclock test) 
xclock # test GUI 
ros2 run turtlesim turtlesim_node # turtlesim GUI 
