#!/bin/sh
# Finds and copies all csv files from the remote host's /tmp directory

scp -P 2222 remote@localhost:/tmp/*.csv "$PWD"/../data/
