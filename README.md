# pyugvswarm

### Introduction
A python wrapper around unmanned ground vehicle (UGV) swarm positioning/control ROS nodes. We offer python APIs which simplifies the development of UGV swarm. A lightweight python simulator is also provided to validate your program before the real experiment.

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

This method requires copying the source code.

You can simply copy the `pyugvswarm/pyugvwarm/` folder and optionally `config/` folder into your project as the following structure:
```bash
|__ Your scripts folder
   |_ pyugvswarm/
   |_ config/
   |  |_ ugvs.yaml
   |_ your_python_script.py
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

Peixuan Shu(shupeixuan@qq.com)

### Thanks

This project
