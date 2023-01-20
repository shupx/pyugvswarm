#!/bin/bash

# exit immediately if a command exits with a non-zero status.
set -e

python ../example.py --sim --dt=0.1 --vis=mpl
