# CITRIFIED
Controlled Incision Task Recordings In Fruits, to Inhibit Excessive Depth

Work in progress repository to host the data processing, analysis and control implementation
for the fruit cutting task.

## Setting up

First, you need to create the python environment with `conda`:

```bash
conda env create -f jupyter_docker/environment.yml
```

This package requires the library from [surgeon_recording](https://github.com/epfl-lasa/surgeon_recording/tree/master/source/surgeon_recording). To install it, after the environment creation:

```bash
conda activate citrified
mkdir lib && cd lib
git clone https://github.com/epfl-lasa/surgeon_recording.git
cd surgeon_recording/source/surgeon_recording && python setup.py install
```

## Structure

### Data

This directory and any subdirectories will, for the time being, not contain
any true data, but rather be relative paths that each user of the repo can
populate with local data.
Eventually it should
contain the final data versions by git LFS reference or by linking to 
other long-term storage locations.

### Analysis

Data pipelines for machine learning models and other statistics to operate
on the cleaned dataset. Also contains visualizations.

### Preprocessing

Data pipeline to process raw experimental data into clean data ready for 
analysis. This includes filtering, transformations, resampling and segmenting.

### Control

A place for control implementation specific to recreating the human task 
of cutting fruits with a robot.
