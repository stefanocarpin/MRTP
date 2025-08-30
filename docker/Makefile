WORKSPACE:=MRTP
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
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

# GPU detection - check if nvidia-docker or nvidia-container-runtime is available
GPU_AVAILABLE := $(shell command -v nvidia-docker >/dev/null 2>&1 && echo "true" || (docker info 2>/dev/null | grep -q nvidia && echo "true" || echo "false"))

# GPU flags - only set if GPU is available
ifeq ($(GPU_AVAILABLE),true)
    GPU_FLAGS := --gpus=all
else
    GPU_FLAGS :=
endif

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}) && \
	docker exec -it $${CONTAINER_PS} bash

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
	$(GPU_FLAGS) \
	-v $(PWD)/../MRTP:/${WORKSPACE}/MRTP:Z \
	-v $(HOME)/.ssh:/root/.ssh:ro \
	${IMAGE} bash

clean:
	rm -rf build/ install/ log/

husky:
	ros2 launch clearpath_gz simulation.launch.py

turtlebot:
	ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

.PHONY: repo-init shell build-image vnc bash clean husky