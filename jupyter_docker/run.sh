NAME=$(echo "${PWD##*/}" | tr _ -)

# create a shared volume to store the ros_ws
docker volume create --driver local \
  --opt type=none \
  --opt device=$PWD/../data/ \
  --opt o=bind \
  ${NAME}_data_vol

docker volume create --driver local \
  --opt type=none \
  --opt device=$PWD/../notebooks/ \
  --opt o=bind \
  ${NAME}_notebooks_vol

xhost +
docker run \
  --net=host \
  -it \
  --rm \
  --env="DISPLAY" \
  --volume="${NAME}_data_vol:/home/anaconda/data:rw" \
  --volume="${NAME}_notebooks_vol:/home/anaconda/notebooks:rw" \
  $NAME
