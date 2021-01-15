#!/bin/sh
# This script contains any necessary setup steps after
# cloning the repository. This includes fetching the
# latest header from the franka_lightweight_interface

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

(
  mkdir -p ../include/franka_lwi && \
  cd ../include/franka_lwi && \
  curl -o franka_lwi_communication_protocol.h \
  "https://raw.githubusercontent.com/epfl-lasa/franka_lightweight_interface/main/include/franka_lightweight_interface/franka_lwi_communication_protocol.h?token=AHXSTB437OAZQKPIAJMHSE3ABBCMM"
)