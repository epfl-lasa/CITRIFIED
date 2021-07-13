# CITRIFIED

Controlled Incision Task Recordings In Fruits, to Inhibit Excessive Depth

Repository to host the data processing, learning and control implementation for the fruit cutting task.

## Structure

The repository is divided into the following directories. Check the directories directly to see the usage instructions
in their READMEs.

### Control

The place of all control implementations in C++ specific to recreating the human task of cutting fruits with a robot.

### Data

This directory and any subdirectories don't contain any true data, but rather be relative paths that each user of the
repo can populate with local data. To get the data that is sometimes needed to run scripts or notebooks, contact the
[authors](#authors--maintainers).

See also the [data format specifications](data_format_spec.md) that describes how the data is logged on the robot.

### Data Visualization

Visualization script for logged JSON data.

### Jupyter Docker

Dockerized Jupyter environment to run the [notebooks](notebooks).

### Learning

In the `ESN` directory, MATLAB scripts to train an echo state networks, inspect, and visualize data. The implementation
of the echo state networks is added as submodule. Before running the scripts, unzip `mat_files.zip`.

The `python` directory contains the implementation of the GPR server that is used to evaluate a GPR model and
communicate with C++ control.

### Notebooks

Data pipelines for machine learning models and other statistics to operate on the cleaned dataset. Also contains
visualizations. Simply follow the instructions [here](jupyter_docker/README.md) to spin up a Jupyter container that
allows to run the notebooks. Before running the notebooks, unzip `mat_files.zip`.

### OptiTrack

Resources and installation instructions for a OptiTrack bridge.

### Preprocessing

Data process scripts to process raw experimental data into clean data ready for analysis.

### ROS

Two ROS packages for tele-operation and streaming ZMQ data over a ROS network.

### RViz Visualization

Visualization of rosbags that were recorded during on of the early experiments.

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Enrico Eberhard ([enrico.eberhard@epfl.ch](mailto:enrico.eberhard@epfl.ch))
- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))