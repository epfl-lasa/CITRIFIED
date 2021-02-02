NAME=$(echo "${PWD##*/}" | tr _ -)
TAG=melodic-desktop

# create a shared volume to store the ros_ws
docker volume create --driver local \
  --opt type=none \
  --opt device=$PWD/../data/ \
  --opt o=bind \
  ${NAME}_data_vol

docker volume create --driver local \
  --opt type=none \
  --opt device=$PWD/citrified_visualization/ \
  --opt o=bind \
  ${NAME}_package_vol

xhost +
docker run \
  --privileged \
  --net=host \
  -it \
  --rm \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${NAME}_data_vol:/home/ros/data/:rw" \
  --volume="${NAME}_package_vol:/home/ros/ros_ws/src/citrified_visualization/:rw" \
  $NAME:$TAG
