# ZMQ Bridge

The ZMQ bridge simply subscribes to a ZMQ socket to receive the joint state of the robot and the pose of an attractor
and republishes those to ROS topic in order to have a visualization of the robot in RViz.

## Running the bridge node and RViz

To run the bridge, execute the following commands:

```bash
cd ../ROS/docker
bash build-run.sh
# this builds and runs a docker container in interactive mode
# inside the container, run
roslaunch zmq_bridge zmq_bridge.launch
```

Note that if you have NVIDIA graphics card, you need to set the flag `USE_NVIDIA_TOOLKIT` to `true` in
the `build-run.sh` script.

This will launch RViz and if you correctly publish the robot state and attractor pose to the ZMQ socket with same URI
from C++ (as it is done [here](../../control/executables/simple_joy_attractor.cpp)), you should see the robot in RViz.

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))