#!/usr/bin/env bash

NAME=citrified/learning/runtime
TAG="latest"

docker run --net=host -it --rm $NAME:$TAG
