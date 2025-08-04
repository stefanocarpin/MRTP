WORKSPACE:=MRTP
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
ROBOT:=husky
IMAGE:=ghcr.io/ucmercedrobotics/mrtp

# Automatic Docker platform detection based on host architecture
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

PLATFORM_ARG := --platform $(PLATFORM)

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}:${ROBOT}) && \
	docker exec -it $${CONTAINER_PS} bash

build-image:
	docker build -f docker/Dockerfile . -t ${IMAGE}:${ROBOT} --target ${ROBOT}

push:
	docker push ${IMAGE}:${ROBOT}

vnc:
	docker run -d --rm --net=host \
	--platform=${PLATFORM} \
	--name=novnc \
	${NOVNC}

bash:
	docker run -it --rm \
	--net=host \
	--privileged \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	${IMAGE}:${ROBOT} bash

clean:
	rm -rf build/ install/ log/

husky:
	ros2 launch clearpath_gz simulation.launch.py

turtlebot:
	ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

.PHONY: repo-init shell build-image vnc bash clean husky