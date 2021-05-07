# Controller implementation

This directory contains all source code and build variables for robot control implementations.

## Structure

- executables
  - The place for main application code. Each application has one source file with a main function.
- include
  - The header files, grouped in subdirectories by module. There is a CMakeLists file which runs a
  a setup script to pull a franka_lwi header from a remote repository.
- lib
  - Any external dependencies. The CMakeLists currently builds and links passive-ds-control.
- scripts
  - Shell scripts for configuring environments and other tasks.
- source
  - The source files of various project modules, grouped in subdirectories by module. Each module
  has a CMakeLists file to link and build the module source, with the toplevel file adding each module.
  
### Modules

There are currently two main modules: motion_generators and controllers. 

To add new modules, follow the existing patterns:
make the appropriate folders in both the source and include directories.
Then, amend the source/CMakeLists.txt file to add the new subdirectory and library.
Finally, create a new CMakeLists file in the source/[module] folder you created and
add the relevant target sources, includes and libraries.

### Executables
To add new executables, simply create a .cpp file in the executables directory,
and amend the CMakeLists with linking instructions.

## Usage

### Setup
After cloning this repository, run the script [setup.sh](scripts/setup.sh) from the scripts directory
to complete any setup steps.

## Interfacing with the robot

This implementation is designed with the [Franka Lightweight Interface](https://github.com/epfl-lasa/franka_lightweight_interface) (LWI)
in mind. The LWI is a real-time process with a 1kHz control loop that publishes robot state information
and listens for commands. These messages are sent over ZMQ sockets, where one unique port has
state information published by the LWI and another unique port has command information consumed by the LWI.

The state and command message types are defined by a [franka_lwi_communication_protocol.h](https://github.com/epfl-lasa/franka_lightweight_interface/blob/main/include/franka_lightweight_interface/franka_lwi_communication_protocol.h)
header in the LWI repository. For convenience, the ZMQ socket configurations expected by the 
LWI are wrapped in network::configureSockets function of [netutils.h](include/franka_lwi/franka_lwi_utils.h).

Finally, it is important that the port numbers for the state and command sockets are correctly
mapped between the controller and the robot. If you change the default port numbers,
make sure both sides of the interface (and any Docker port bindings) are updated accordingly. 

### Development Environment
Run the script [remote-dev.sh](scripts/remote-dev.sh) from the scripts directory
to build and launch a Docker container as a background daemon process.
This environment can be accessed over SSH as a remote host for building and debugging.

The container runs in the docker bridge network `citrinet`.

See the instructions for CLion configuration [here](https://github.com/eeberhard/docker-clion-cpp-env).
Remote host development may also be possible in other IDEs.

By default, the container is launched with the host port 2222 remapped to the container SSH port.
The host ports 5550 and 5551 are bound to the matching container ports for the state and command
messages, respectively.

If you need to configure different ports, change the shell variables for SSH,
state message and command message port numbers in the remote-dev script.

### Deployment
Run the script [build.sh](scripts/build.sh) from the scripts directory
to compile the project into a Docker image.

Then run [run.sh](scripts/run.sh) from the scripts directory to spin up
an interactive container shell with the compiled executables ready to go.