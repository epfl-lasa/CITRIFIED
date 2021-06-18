# Control implementation

This directory contains all source code and build variables for robot control implementations.

## Structure

- `executables`
    - The place for main application code. Each application has one source file with a main function. There is
      a [REAMDE](executables/README.md) that explains how to run the executables.
- `include`
    - The header files, grouped in subdirectories by module.
- `scripts`
    - Shell scripts for configuring environments and other tasks. There is a setup script that has to be run to pull a
      franka_lwi header from a remote repository.
- `source`
    - The source files of various project modules, grouped in subdirectories by module. Each module has a CMakeLists
      file to link and build the module source, with the toplevel file adding each module.

### Modules

There are several modules:

- `controllers`:
    - The low level robot controllers are included from the `control_libraries`. The `controller` module consists of
      task-based state-machine-like controllers as well as one kinematic controller using KDL.
    - Moreover, the `FrankaController.h` communicates with a robot in a separate thread to allow for dual robot setups.

- `filters`:
    - Implementation of a digital Butterworth filter (low-pass)
      with [time-domain difference equations](https://ch.mathworks.com/help/matlab/ref/filter.html). For examples on how
      to use and configure, see [here](source/filters/tests/test_digital_butter_filter.cpp)
      and [here](executables/optitrack_incision_trials.cpp).

- `franka_lwi` (header only):
    - Header files related to the `franka_lwi` communication, e.g. the protocol, conversion helpers, and a simple logger
      for the robot state. Note that you have to run [setup.sh](scripts/setup.sh) to get the protocol header file.

- `learning`:
    - Implementation of the echo-state-network (ESN) algorithm as well as the `ESNWrapper.h` to start and run an ESN
      classification in a separate thread. For examples on how to use and load (the ESN class needs a YAML configuration
      file with specific keys), see [here](source/learning/tests/).
    - Implementation of a gaussian process regression (GPR) client, that communicates with a Python server to get the
      predictions (see [here](../learning/python) for the server and [here](executables/tests/test_gpr_server.cpp)
      for an example on how to use it).

- `logger`:
    - Implementation of our very own logger to record robot state, sensor data, control parameters and trial
      configuration. Used in all `*_trials.cpp` executables and tested [here](source/logger/tests/test_json_logger.cpp).

- `network` (header only):
    - `zmq_interface.h` contains the implementation of the ZMQ communication (send, receive, poll) and different socket
      tpyes
      (e.g. publisher-subscriber, publisher, subscriber, and pair sockets).
    - `interface.h` defines all the interfaces present in this repository (robots, Optitrack, GPR, etc.) with
      an `enum InterfaceType`.

- `sensors`:
    - Wrapper classes for the FT sensor, Optitrack, as well as the joystick for tele-operation. Examples can be found in
      almost all executables (Optitrack especially [here](executables/optitrack_incision_trials.cpp) and Joy
      [here](executables/simple_joy_attractor.cpp).

To add new modules, follow the existing patterns:
make the appropriate folders in both the source and include directories. Then, amend the `source/CMakeLists.txt` file to
add the new subdirectory and library. Finally, create a new `CMakeLists.txt` file in the `source/[module]` folder you
created and add the relevant target sources, includes and libraries.

### Executables

To add new executables, simply create a `.cpp` file in the `executables` directory, and amend the CMakeLists with
linking instructions. For detailed usage instructions and explanations on the executables,
see [here](executables/README.md).

## Usage

### Setup

After cloning this repository, run the script [setup.sh](scripts/setup.sh) from inside the `scripts` directory to
complete any setup steps.

## Interfacing with the robot

This implementation is designed with
the [Franka Lightweight Interface](https://github.com/epfl-lasa/franka_lightweight_interface) (LWI)
in mind. The LWI is a real-time process with a 1kHz control loop that publishes robot state information and listens for
commands. These messages are sent over ZMQ sockets, where one unique port has state information published by the LWI and
another unique port has command information consumed by the LWI.

The state and command message types are defined by
a [franka_lwi_communication_protocol.h](https://github.com/epfl-lasa/franka_lightweight_interface/blob/main/include/franka_lightweight_interface/franka_lwi_communication_protocol.h)
header in the LWI repository. For convenience, the ZMQ socket configurations expected by the LWI are wrapped in
the [network module](include/network/interfaces.h).

Finally, it is important that the port numbers for the state and command sockets are correctly mapped between the
controller and the robot. If you change the default port numbers, make sure both sides of the interface (and any Docker
port bindings) are updated accordingly.

### Development Environment

Run the script [remote-dev.sh](scripts/remote-dev.sh) from the inside the `scripts` directory to build and launch a
Docker container as a background daemon process. This environment can be accessed over SSH as a remote host for building
and debugging.

The container runs in the docker bridge network `citrinet`.

See the instructions for CLion
configuration [here](https://github.com/epfl-lasa/control_libraries/blob/main/CONTRIBUTING.md#configuring-the-development-environment)
. Remote host development may also be possible in other IDEs.

By default, the container is launched with the host port 2222 remapped to the container SSH port. The host ports 1601 (1701) 
and 1602 (1702) are bound to the matching container ports for the state and command messages, respectively.

If you need to configure different ports, change the shell variables for SSH, state message and command message port
numbers in the `remote-dev.sh` script.

### Deployment

Run the script [build.sh](scripts/build.sh) from inside the `scripts` directory to compile the project into a Docker
image.

Then run [run.sh](scripts/run.sh) from inside the `scripts` directory to spin up an interactive container shell with the
compiled executables ready to go.

### Simulation

For easy task with no force interaction, there exists a simulation to visualize the robot moving with torque commands
generated from dynamical systems. Read installation
instructions [here](executables/README.md#franka_papa_16-and-franka_quebec_17) and check
the [detailed executables descriptions](executables/README.md) for the possibility of running a specific executable with
the simulator.

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Enrico Eberhard ([enrico.eberhard@epfl.ch](mailto:enrico.eberhard@epfl.ch))
- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))