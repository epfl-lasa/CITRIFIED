# CITRIFIED
Controlled Incision Task Recordings In Fruits, to Inhibit Excessive Depth

Work in progress repository to host the data processing, analysis and control implementation
for the fruit cutting task.


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
on the cleaned dataset. Also can contains visualsations.

### Preprocessing

Data pipeline to process raw experimental data into clean data ready for 
analysis. This includes filtering, transformations, resampling and segmenting.

### Control

A place for control implementation specific to recreating the human task 
of cutting fruits with a robot.
