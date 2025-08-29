# Docker
For use in Docker, we've provided a container for each simulation environment. 
1. First `make build-image`. This will create the container image.
2. To access this image, simply `make bash`.
3. If you want to run multiple shells within the container, run `make shell` from outside after you've spawned the container.

From within you can follow instructions set out in the book such as `colcon build` and `source install/setup.bash`.