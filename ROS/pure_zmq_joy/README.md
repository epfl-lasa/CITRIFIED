# Pure ZMQ Joy

This ROS package is used to republish the joystick commands from a PS3 (or any other compatible) controller to a ZMQ
socket to use the controller for tele-operation.

## Running the joy node

If the controller cannot be connected to the machine that runs the executables (as it was the case during the
development of CITRIFIED), please continue
reading [here](#controller-is-connected-to-another-machine-in-the-same-network).

### Controller is connected to the host machine

1. Connect the controller to the computer (cable or bluetooth).
2. Build and run the docker image:
   ```bash
   cd ../docker
   bash setup.sh
   bash build-run.sh
   # this builds and runs a docker container in interactive mode
   # inside the container, run
   roslaunch pure_zmq_joy get_controller_name.launch
   ```
   Check the output to see what the name of the detected controller is.

3. In the same interactive container
   ```bash
   roslaunch pure_zmq_joy pure_zmq_joy.launch launch_joy:=true controller_name:="<controller_name>"
   ```
   where you put the controller name from step 2. You should see the `joy` message in the terminal when you press a
   button on the controller.

### Controller is connected to another machine in the same network

If the controller is connected to another machine in the same LAN (preferably to computers connected to the ethernet
ports in the robot room), there are a few extra steps:

1. On the machine that the controller is connected to (and has ROS installed):
    - Clone this repository, copy the `joy` package
      from [joystick drivers](ttps://github.com/ros-drivers/joystick_drivers.git) into a ROS workspace and `catkin_make`
      it.
    - In a first terminal, run `roscore`.
    - In a second terminal, run `rosrun joy joy_node`.
    - In a third terminal, run `rostopic echo /joy` to be sure that the topic is published.

2. On the machine that is running the executables:
    - In the `docker` directory of the ROS packages, make sure that in the `update_bashrc` file, the `ROS_MASTER_URI` is
      correctly exported (not commented out and correct URI).
    - In the `docker` directory of the ROS packages, make sure that in the `build-run.sh` script, you add a
      host (`--add-host <hostname:hosturi>`) in the `docker run` command.
    - Run
      ```bash
      bash build-run.sh
      # this builds and runs a docker container in interactive mode
      # inside the container, run
      rosrun pure_zmq_joy pure_zmq_joy_node
      ```
      Again, you should see the `joy` message in the terminal when you press a button on the controller.

## Running an example

If the node from the step before is running and you can see the `joy` message in the terminal,
execute [test_joy.cpp](../../control/executables/tests/test_joy.cpp). In the terminal, you should see how the pose is
changing when you give some inputs with the controller.

Note that the ZMQ socket URI (8888) is hardcoded and has to correspond to the URI in the interface in the executable.

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))