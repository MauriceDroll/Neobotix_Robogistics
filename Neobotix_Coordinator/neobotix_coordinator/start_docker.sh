#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color


is_image_build() {
    if [ "$(docker images -q neobotix_coordinator/ros:humble 2> /dev/null)" == "" ]; then
        echo -e "${RED}Image not found. Have you build the image?${NC}"
        echo -e "${BLUE}Try running: ./build_docker.sh${NC}"
        return 1
    else
        echo -e "${GREEN}Image found.${NC}"
        return 0
    fi
}
run_docker() {
    is_image_build
    if [ $? -eq 0 ]; then
        echo -e "${YELLOW}Running docker...${NC}"
        xhost + local:root
        docker run \
            --name neobotix_coordinator \
            -it \
            --net host \
            -e DISPLAY=$DISPLAY \
            --env-file .env \
            --rm \
            -v $PWD/neobotix_coordinator:/home/docker/ros2_ws/src/neobotix_coordinator \
            -v $PWD/iras_interfaces:/home/docker/ros2_ws/src/iras_interfaces \
            -v $PWD/.vscode:/home/docker/ros2_ws/src/.vscode \
            neobotix_coordinator/ros:humble \
            # ros2 run groot Groot
    fi
}

run_docker
