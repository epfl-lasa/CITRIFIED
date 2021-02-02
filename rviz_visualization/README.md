# RVIZ Visualization of Experiment Data (Docker)

## Preparation

1. Make sure that the experiment data is available in the data folder. By default the visualization is done based on
   preprocessed and transformed data (folder *CITRIFIED/data/preprocessed_transformed_data*). The folder structure
   should look like that:
   ```bash
   data
   └──preprocessed_transormed_data
      └──experiment
         └──fruit
            └──cut_quality
               ├──fruit_cutquality_1.csv
               ├──fruit_cutquality_2.csv
               ├──...
               └──...
   ```
2. Build the docker image with

```bash
bash build.sh
```

## Usage

### Generate the rosbag files

To generate the rosbag files for a whole directory, do the following:

```bash
bash run.sh
rosrun citrified_visualization write_data_to_rosbag experiment fruit cut_quality
```

where you substitue the desired values for *experiment*, *fruit*, and *cut_quality*. If you like to generate the rosbag
file only for a specific file, add the number of the run at the end of the `rosrun` command, e.g.

```bash
rosrun citrified_visualization write_data_to_rosbag experiment fruit cut_quality run
```

The rosbag files are saved in the data directory under *rosbags*, e.g. *CITRIFIED/data/rosbags/* with the same folder
structure as above.

**ATTENTION:** This script expects a certain structure in the data files. Use the intended jupyter notebook to generate
them.

### Visualize a rosbag in RVIZ

To see the generated rosbags in RVIZ, open two terminals. In the first terminal, run

```bash
bash run.sh
roslaunch citrified_visualization citrified_visualization.launch
```

In the second terminal, run

```bash
bash run.sh
cd ~/data/rosbags/
rosbag play experiment/fruit/cut_quality/fruit_cutquality_run.bag
```

where you substitute *experiment*, *fruit*, *cut_quality*, and *run* with your desired combination.