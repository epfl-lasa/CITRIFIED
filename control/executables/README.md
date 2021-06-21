# Executables

Find here instructions to run the executables currently present in CITRIFIED.

## System components

First, it is important to know how to run the different components of the whole system such that the communication over
the desired interfaces doesn't fail.

### `FRANKA_PAPA_16` and `FRANKA_QUEBEC_17`

The most important interface is the robot interface. In order to run any executable, you either

- have to run the [franka_lightweight_interface](https://github.com/epfl-lasa/franka_lightweight_interface) with the
  real robots, or
- if you don't have access to the real robots, run the simulation for `FRANKA_PAPA_16` (currently only this robot
  supported, no dual robot simulation):
  ```bash
  cd CITRIFIED && cd ..
  git clone --branch citrified --single-branch https://github.com/epfl-lasa/pybullet_robot.git
  cd pybullet_robot
  bash build.sh
  bash run.sh
  # this spins up an interactive container, inside the container run
  python tests/torque_control_test.py 
  ```
  NOTE: The control gains of the controller have to be decreased by a factor of ~10 such that the simulation runs
  smoothly. Also, the simulator is under development and serves for visualization purposes only.

### Force Torque Sensor

The force torque sensor is just plug & play. Connect the NETFT Box in the robot room to the ethernet port with
number `D211.3`, make sure you can ping the FT sensor under its IP, and then configure the IP in the C++ executable
accordingly. It's also possible to run a mock FT sensor, be aware that this always returns zero force and torque though.

See how to use (and test) the FT sensor [here](tests/test_ft_sensor.cpp).

### Optitrack

With the help of the Optitrack interface, it is possible to get the pose of markers from the [*
OptitrackZMQBridge*](../../optitrack/source/OptiTrackZMQBridge.cpp). Just make sure

- Motive is [streaming the data](https://v22.wiki.optitrack.com/index.php?title=Data_Streaming),
- the ZMQ bridge is [up and running](../../optitrack/README.md) and configured with the right IP,
- you defined the desired rigid bodies in Motive and you know their ID.

Then, you can just start a tracker, define the rigid bodies' ID

```cpp
sensors::RigidBodyTracker tracker;
tracker.start();
int robotBaseID = 1;  // the OptiTrack streaming ID for the robot base frame
int attractorID = 2;  // the OptiTrack streaming ID for the attractor frame
```

and get their state with

```cpp
state_representation::CartesianState attractor("attractor", "optitrack");
state_representation::CartesianPose robotInOptitrack("robot_base", "optitrack");
tracker.getState(robotInOptitrack, robotBaseID);
tracker.getState(attractor, attractorID);
```

as it is showcased [here](tests/mocap_point_attractor.cpp).

### GPR

The GPR class is a simple interface to communicate with the [GPR server implemented here](../../learning/python). For
instructions and further details please refer to its [README](../../learning/python/README.md).

### Joy and Bridge

These two interfaces are used with the corresponding [ROS packages implemented here](../../ROS). They simply subscribe
to ROS topics and republish to a ZMQ socket, and the other way round. The READMEs for [*pure_zmq_joy*](../../ROS/pure_zmq_joy/README.md)
and [*zmq_bridge*](../../ROS/zmq_bridge/README.md) provide instructions for setting up and running the ROS packages.

## Executables

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Enrico Eberhard ([enrico.eberhard@epfl.ch](mailto:enrico.eberhard@epfl.ch))
- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))