#!/bin/sh
# Builds and runs the remote development container
# If this fails, try running the following command:
# sudo ssh-keygen -f "$HOME/.ssh/known_hosts" -R 127.0.0.1:"$PORT"

IMAGE_NAME=clion/cpp-remote-dev/citrified
CONTAINER_NAME=citrified-remote-dev
PORT=2222

docker build -f ../Dockerfile.remote-dev -t $IMAGE_NAME .

docker container stop "$CONTAINER_NAME" > /dev/null 2>&1
docker rm --force "$CONTAINER_NAME" > /dev/null 2>&1

docker run -d --cap-add sys_ptrace -p127.0.0.1:"$PORT":22 --name $CONTAINER_NAME $IMAGE_NAME