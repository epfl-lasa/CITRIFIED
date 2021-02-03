#!/bin/sh
# Builds the runtime image (compiled applications)
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

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

IMAGE_NAME=citrified/control/runtime

if [ "$REBUILD" -eq 1 ]; then
  docker build --no-cache --target runtime -f ../Dockerfile --tag $IMAGE_NAME ..
else
  docker build --target runtime -f ../Dockerfile --tag $IMAGE_NAME ..
fi
