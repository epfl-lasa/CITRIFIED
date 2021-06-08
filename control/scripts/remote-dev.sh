#!/usr/bin/env bash

IMAGE_NAME=citrified/control/remote-dev
STAGE_NAME=remote-development
CONTAINER_NAME=citrified-control-remote-development-ssh
CONTAINER_HOSTNAME=citrified-control-remote-development

SSH_PORT=2222
SSH_KEY_FILE="$HOME/.ssh/id_rsa.pub"

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

HELP_MESSAGE="Usage: ./remote-dev.sh [--port <port>] [--key-file </path/to/id_rsa.pub>] [-r]
Build and run a docker container as an SSH toolchain server for remote development.
The server is bound to the specified port on localhost (127.0.0.1)
and uses passwordless RSA key-pair authentication. The host public key
is read from the specified key file and copied to the server on startup.
The server will run in the background as ${CONTAINER_NAME}.
You can connect with 'ssh remote@localhost -p <port>'.
Close the server with 'docker container stop ${CONTAINER_NAME}'.
Options:
  -p, --port [XXXX]        Specify the port to bind for SSH
                           connection.
                           (default: ${SSH_PORT})
  -k, --key-file [path]    Specify the path of the RSA
                           public key file.
                           (default: ${SSH_KEY_FILE})
  -r                       Rebuild the image with the '--no-cache' option
  -h, --help               Show this help message."


REBUILD=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    -p|--port) SSH_PORT=$2; shift 1;;
    -k|--key-file) SSH_KEY_FILE=$2; shift 1;;
    -r) REBUILD=1; shift 1;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;
    -*) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done

PORT_OPTITRACK=5511
PORT_GPR=7777
PORT_JOY=8888
PORT_BRIDGE=9999

if [ "$REBUILD" -eq 1 ]; then
  DOCKER_BUILDKIT=1 docker build --no-cache --target "${STAGE_NAME}" -f ../Dockerfile --tag "$IMAGE_NAME" ..
else
  DOCKER_BUILDKIT=1 docker build --target "${STAGE_NAME}" -f ../Dockerfile --tag "$IMAGE_NAME" ..
fi

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

docker network inspect citrinet >/dev/null 2>&1 || \
  docker network create --subnet=172.20.0.0/16 --gateway=172.20.0.1 citrinet

docker run -d --cap-add sys_ptrace \
  --publish 127.0.0.1:"${SSH_PORT}":22 \
  --network=citrinet \
  -p1601:1601 -p1602:1602 \
  -p1701:1701 -p1702:1702 \
  -p"$PORT_OPTITRACK":"$PORT_OPTITRACK" \
  -p"$PORT_GPR":"$PORT_GPR" \
  -p"$PORT_JOY":"$PORT_JOY" \
  -p"$PORT_BRIDGE":"$PORT_BRIDGE" \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_HOSTNAME}" \
  "${IMAGE_NAME}" "$(cat ${SSH_KEY_FILE})"
