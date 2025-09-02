# Docker
# Installation
Setting yourself up with Docker usually means you don't have a Linux machine or you prefer to manage your workspace in a container.
In either case, you can find the instructions on how to install Docker Desktop for your preferred OS [here](https://docs.docker.com/desktop/?_gl=1*f1d4un*_gcl_au*MTQ5ODc3MjEwOC4xNzU2NzEyODAz*_ga*MjU1MDAwMDU4LjE3NTY3MTI3OTk.*_ga_XJWPQMJYHQ*czE3NTY3MTI3OTgkbzEkZzEkdDE3NTY3MTI4MDMkajU1JGwwJGgw).

In order for noVNC to work, you must enable host networking in Docker for your machine.
To do this, open Docker Desktop (Linux has this enabled by default), click on the settings wheel on the top right >> resources >> network >> enable host networking.
Then, apply and restart your Docker app for this to take effect. 

## Usage
For use in Docker, we've provided a container for each simulation environment.  First clone the MRTP repo

`git clone https://github.com/stefanocarpin/MRTP`

1. Move to the folder MRTP/docker and run `make build-image`. This will create the container image.
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
Windows requires a few extra steps. First install [Docker](#docker) (see instructions above).
1. Install prerequisites: `choco install make -y`
2. Then follow the remaining steps in [Usage](#usage) and [noVNC](#novnc).
