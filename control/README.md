# Controller implementation

This subfolder contains all source code and build variables for robot control implementations.

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

### Development Environment
Run the script [remote-dev.sh](scripts/remote-dev.sh) from the scripts directory
to build and launch a Docker container as a background daemon process.
This environment can be accessed over SSH as a remote host for building and debugging.

See the instructions for CLion configuration [here](https://github.com/eeberhard/docker-clion-cpp-env).
Remote host development may also be possible in other IDEs.

### Deployment
TODO: Build a release image for all project executables so that they can easily be used for
real data collection and other applications. For now, you can use the debug build artefacts from
within the development environment.