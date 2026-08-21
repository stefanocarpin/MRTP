WORKSPACE:=MRTP
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
IMAGE:=ghcr.io/ucmercedrobotics/mrtp

# Automatic Docker platform detection based on host architecture
ifeq ($(OS),Windows_NT)
    # Windows - use PROCESSOR_ARCHITECTURE environment variable
    ARCH := $(PROCESSOR_ARCHITECTURE)
    ifeq ($(ARCH),AMD64)
        PLATFORM := linux/amd64
    else ifeq ($(ARCH),ARM64)
        PLATFORM := linux/arm64
    else
        PLATFORM := linux/amd64
    endif
	CURRENT_DIR := $(shell cd)
    HOME_DIR := $(USERPROFILE)
else
    # Unix/Linux - use uname
    UNAME_M := $(shell uname -m)
    ifeq ($(UNAME_M),x86_64)
        PLATFORM := linux/amd64
    else ifeq ($(UNAME_M),arm64)
        PLATFORM := linux/arm64
    else ifeq ($(UNAME_M),aarch64)
        PLATFORM := linux/arm64
    else
        PLATFORM := linux/amd64
    endif
	CURRENT_DIR := $(shell pwd)
    HOME_DIR := $(HOME)
endif

PLATFORM_ARG := --platform $(PLATFORM)

shell:
	docker exec -it $(shell docker ps -aq --filter ancestor=${IMAGE}) bash

build-image:
	docker build -f Dockerfile . -t ${IMAGE} --target base

push:
	docker push ${IMAGE}

vnc:
	docker run -d --rm --net=host \
	--platform=${PLATFORM} \
	--name=novnc \
	${NOVNC}

bash:
	cd .. && \
	docker run -it --rm \
	--net=host \
	--privileged \
	-v "$(CURRENT_DIR)/..":/${WORKSPACE}:Z \
	${IMAGE} bash

clean:
	rm -rf build/ install/ log/

husky:
	ros2 launch gazeboenvs gazebo_husky.launch.py

turtlebot:
	ros2 launch gazeboenvs tb4_simulation.launch.py

amiga:
	ros2 launch gazeboenvs amiga_orchard.launch.py

.PHONY: repo-init shell build-image vnc bash clean husky turtlebot amiga