REBUILD=0

while getopts 'r' opt; do
  case $opt in
  r) REBUILD=1 ;;
  *)
    echo 'Error in command line parsing' >&2
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

BASE_IMAGE=continuumio/anaconda3

docker pull ${BASE_IMAGE}

NAME=$(echo "${PWD##*/}" | tr _ -)

UID="$(id -u $USER)"
GID="$(id -g $USER)"

if [ "$REBUILD" -eq 1 ]; then
  docker build \
    --no-cache \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg UID=${UID} \
    --build-arg GID=${GID} \
    -t ${NAME} .
else
  docker build \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg UID=${UID} \
    --build-arg GID=${GID} \
    -t ${NAME} .
fi
