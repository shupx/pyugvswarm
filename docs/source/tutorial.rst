.. _tutorial:

Tutorial
=========

1. Specify your ugvs information in a configuration file similar to ``config/ugvs.yaml``:

.. code:: xml

  ugvs:
  - id: 7
    pos_source: mocap # mocap, nluwb, cfuwb
    yaw_source: mocap # mocap, imu
    initialPosition: [0.25, 0.55, 0.0, -1.8] # [x(m),y(m),z(m),yaw(rad)]

  - id: 10
    pos_source: mocap # mocap, nluwb, cfuwb
    yaw_source: mocap # mocap or imu
    initialPosition: [0.0, 0.55, 0.0, 0.0] # [x(m),y(m),z(m),yaw(rad)]

The ``initialPosition`` is only for simulation.

2. Write your python script. Import the ``UGVswarm`` class of this python package in your project. See the example of ``example.py``.

.. code:: python

    # in your python script
    from pyugvswarm import *

    swarm = UGVswarm('your config file path') # absolute path is recommended
    timeHelper = swarm.timeHelper
    allugvs = swarm.allugvs

    ugvs = allugvs.ugvs
    ugv_0 = ugvs[0]

3. Run the positioning & communication nodes and your script:

- For simulation, you can run your python script with arg ``--sim``. It does not rely on ROS.

- For experiment, we support 3 types of positioning system: **motion capture, nooploop UWB**, and **crazyflie UWB**. The control commands and UGV states were tranferred by `swarm_ros_bridge <https://gitee.com/shu-peixuan/swarm_ros_bridge.git>`_ through WIFI network. You should modify ``config/ros_topics.yaml`` for the real communication situations.

.. tabs::

    .. tab:: Simulation

        .. code:: bash

            cd launch/
            ./run_sim.sh # or nluwb,cfuwb 

    .. tab:: mocap

        .. code:: bash

            cd launch/
            ./run_exp_mocap.sh # or nluwb,cfuwb 

    .. tab:: nluwb

    .. tab:: cfuwb

