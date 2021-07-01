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
  git clone --branch citrified --single-branch https://github.com/domire8/pybullet_robot.git
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

With the help of the Optitrack interface, it is possible to get the pose of markers from the
[*OptiTrackZMQBridge*](../../optitrack/source/OptiTrackZMQBridge.cpp). Just make sure

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
to ROS topics and republish to a ZMQ socket, and the other way round. The READMEs for
[*pure_zmq_joy*](../../ROS/pure_zmq_joy/README.md) and [*zmq_bridge*](../../ROS/zmq_bridge/README.md) provide
instructions for setting up and running the ROS packages. Make sure that both setup scripts (the one
in [control](../scripts/setup.sh) and the one in [ROS](../../ROS/docker/setup.sh)) are executed prior to the usage such
that all the headers with the communication protocols are in sync.

## Executables

Name | Needed components | Remarks
------|-------------------|---------
circular_cut.cpp | robot (real or sim) | A circular cut by blending two DS together.
dual_franka.cpp | robots (real) | The robots will try to maintain a constant offset between their end-effectors (given by *target_in_papa_ee*) but *FRANKA_PAPA_16* has zero control gains so can essentially be moved around by hand and *FRANKA_QUEBEC_17* will try to follow.
dual_franka_incision_trials.cpp | robots (real), FT, GPR | Incision trials with *FRANKA_QUEBEC_17* holding the base and moving around with a circular DS while *FRANKA_PAPA_16* probes the fruit surface, inserts and cuts. The parameters can be modified in `trial_config/dual_incision_trials_parameters.yaml`.
dual_franka_joy_incision_trials.cpp | robots (real), FT, GPR, joy | Essentially the same as above but *FRANKA_QUEBEC_17* is controlled with the joystick. The parameters can be modified in `trial_config/dual_incision_trials_parameters.yaml`.
optitrack_incision_trials.cpp | robot (real), FT, GPR, Optitrack | Incision trials where the task base is tracked by Optitrack while *FRANKA_PAPA_16* probes the fruit surface, inserts and cuts. The parameters can be modified in `trial_config/incision_trials_parameters.yaml`.
silicon_insertion_trials.cpp | robot (real), FT | Silicon insertions where a human needs to puncture tubes in silicon layers. The parameters can be modified in `trial_config/silicon_trials_parameters.yaml`.
simple_ik_controller.cpp | robot (real or sim) | Move to an attractor in space using an inverse kinematics controller.
simple_joy_attractor.cpp | robot (real or sim), joy, bridge (optionally) | Move attractor of end-effector around with the joystick and optionally visualize the robot in RViz.
simple_point_attractor.cpp | robot (real or sim) | Move to an attractor in space passed in command line (for example 0.5 0 0.5 0 1 0 0).
surface_prober.cpp | robot (real), FT | Probe surface of fruit to get a surface shape estimation. The parameters can be modified in `trial_config/surface_probe_parameters.yaml`.
tests/mocap_point_attractor.cpp | robot (real), Optitrack | Move to an attractor that is tracked by Optitrack with an offset defined by the variable *offset*.
tests/test_ft_sensor.cpp | FT | Get measurements from Force Torque sensor to test connection.
tests/test_gpr_server.cpp | GPR (test_interface.py) | Test connection with the GPR server.
tests/test_interface.cpp | robot (real or sim) | Test connection with the robot.
tests/test_joy.cpp | joy | Test connection with the joystick.

Notes: pickles, config files yamls, optitrack markers.

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Enrico Eberhard ([enrico.eberhard@epfl.ch](mailto:enrico.eberhard@epfl.ch))
- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))