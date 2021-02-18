# CITRIFIED

Controlled Incision Task Recordings In Fruits, to Inhibit Excessive Depth

Work in progress repository to host the data processing, analysis and control implementation for the fruit cutting task.

## Setting up of data processing and learning environment

If you want to develop new or run available Jupyter notebooks, you first need to set up your desired the python
environment. You have two possibilities:

1. Create a local Anaconda environment on your machine with `conda`:

    ```bash
    cd path/to/CITRIFIED
    conda env create -f jupyter_docker/environment.yml
    ```

   Additionally, the library
   from [surgeon_recording](https://github.com/epfl-lasa/surgeon_recording/tree/master/source/surgeon_recording) is
   required. To install it, after the environment creation:

    ```bash
    conda activate citrified
    mkdir lib && cd lib
    git clone https://github.com/epfl-lasa/surgeon_recording.git
    cd surgeon_recording/source/surgeon_recording && python setup.py install
    ```
   Afterwards, start a Jupyter notebooks server to develop and run notebooks locally with
    ```bash
    jupyter notebook
    ```

2. Use the dockerized Jupyter environment (recommended): Go to the subdirectory `jupyter_docker` and follow the
   instructions there.

## Structure

### Data

This directory and any subdirectories will, for the time being, not contain any true data, but rather be relative paths
that each user of the repo can populate with local data. Eventually it should contain the final data versions by git LFS
reference or by linking to other long-term storage locations.

### Analysis

Data pipelines for machine learning models and other statistics to operate on the cleaned dataset. Also contains
visualizations.

### Preprocessing

Data pipeline to process raw experimental data into clean data ready for analysis. This includes filtering,
transformations, resampling and segmenting.

### Control

A place for control implementation specific to recreating the human task of cutting fruits with a robot.
