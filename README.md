# pyugvswarm

### Introduction
A python wrapper around unmanned ground vehicle (UGV) swarm positioning/control ROS1 nodes. We offer python APIs which simplifies the development of UGV swarm. A lightweight python simulator is also provided to validate your program before the real experiment.

### Install

- #### Method 1 (recommended): 

Install `pyugvswarm` python package from source:

```bash
git clone https://gitee.com/shu-peixuan/pyugvswarm.git
cd pyugvswarm/pyugvswarm
pip install -e . # use pip3 if you use python3
```
Check you have pyugvswarm package in your PYTHON environment:

```bash
pip list |grep pyugvswarm
# should list the version and location of pyugvswarm
```

- #### Method 2: 

You can simply copy the `pyugvswarm/pyugvwarm/` folder and optionally `config/` folder into your project:

```bash
git clone https://gitee.com/shu-peixuan/pyugvswarm.git
cp -r pyugvswarm/pyugvswarm ${your_scripts_path}
```

The folder structure should be like this:

```bash
|__ Your scripts folder
   |_ pyugvswarm/
   |_ config/
   |  |_ ugvs.yaml
   |_ your_python_script.py
```

- #### Method 3 (simple): 

Write your python scripts directly in this repository like the `example.py`.

```bash
git clone https://gitee.com/shu-peixuan/pyugvswarm.git
cd pyugvswarm/
touch your_python_script.py
```


### Usage

1. Specify your ugvs information in a configuration file similar to `config/ugvs.yaml`. 


2. Import the `UGVswarm` class of this python package in your project. See the example of `example.py`.

```python
# in your python script
from pyugvswarm import *

swarm = UGVswarm('your config file path') # absolute path is recommended
timeHelper = swarm.timeHelper
allugvs = swarm.allugvs

ugvs = allugvs.ugvs
ugv_0 = ugvs[0]
```

### API reference

TODO.


### Contributor

Peixuan Shu (shupeixuan@qq.com)

### Acknowledgement

This project is mainly motivated by the `pycrazyswarm` of [crazyswarm](https://github.com/USC-ACTLab/crazyswarm/tree/master/ros_ws/src/crazyswarm/scripts/pycrazyswarm), which is a python wrapper around crazyswarm ROS nodes to control crazyflie UAV swarm. Thanks for their remarkable work!

Crazyswarm1 documentation: https://crazyswarm.readthedocs.io/en/latest/
