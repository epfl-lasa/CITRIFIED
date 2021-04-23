#!/bin/sh
# Finds and copies all json files from the remote host's /tmp directory

scp -P 2222 remote@localhost:/tmp/*.json "$PWD"/../data/
