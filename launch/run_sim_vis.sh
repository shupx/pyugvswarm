#!/bin/bash

# exit immediately if a command exits with a non-zero status.
set -e

### for matplot 3d display (slower simulation)
python ../example.py --sim --dt=0.1 --vis=mpl

### for none display (faster simulation)
# python ../example.py --sim --dt=0.1 --vis=null
