#!/bin/bash
set -e

# start in home directory
cd /home/anaconda/notebooks

exec bash -i -c $@
