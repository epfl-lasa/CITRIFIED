#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

MULTISTAGE_TARGET="dev-user"

path=$(echo "${PWD}" | rev | cut -d'/' -f-3 | rev)
if [ "${path}" != "CITRIFIED/ROS/docker" ]; then
  echo "Run this script from within the directory CITRIFIED/ROS/docker !"
  echo "You are currently in ${path}"
  exit 1
fi

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

IMAGE_NAME=citrified/ros-ws

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")

if [[ "${OSTYPE}" != "darwin"* ]]; then
  UID="$(id -u "${USER}")"
  GID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${UID}")
  BUILD_FLAGS+=(--build-arg GID="${GID}")
fi
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${MULTISTAGE_TARGET}")

if [ "${REBUILD}" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .. || exit

docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/../" \
    --opt o="bind" \
    "citrified_ros_pkg_vol"

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="host" \
  --volume="citrified_ros_pkg_vol:/home/ros/ros_ws/src/citrified" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${XAUTH}:${XAUTH}" \
  --env XAUTHORITY="${XAUTH}" \
  --env DISPLAY="${DISPLAY}" \
  "${IMAGE_NAME}:${MULTISTAGE_TARGET}"

#  --add-host lasapc21:128.178.145.15 \
