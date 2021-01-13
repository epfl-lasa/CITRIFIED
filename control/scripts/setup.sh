#!/bin/sh

(
  mkdir -p ../include/franka_lwi && \
  cd ../include/franka_lwi && \
  curl -o franka_lwi_communication_protocol.h \
  "https://raw.githubusercontent.com/epfl-lasa/franka_lightweight_interface/main/include/franka_lightweight_interface/franka_lwi_communication_protocol.h?token=AHXSTB437OAZQKPIAJMHSE3ABBCMM"
)