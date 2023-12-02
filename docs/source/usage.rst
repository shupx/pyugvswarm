.. _usage:

Usage
=========

1. Specify your ugvs information in a configuration file similar to ``config/ugvs.yaml``:

.. code:: yaml

  ugvs:
    - id: 7
      pos_source: mocap # mocap, nluwb, cfuwb
      yaw_source: mocap # mocap, imu
      mocap_markerset_name: ugv7 # markerset name in vrpn
      initialPosition: [0.25, 0.55, 0.0, -1.8] # [x(m),y(m),z(m),yaw(rad)] # only for simulation
    
    - id: 10
      pos_source: nluwb # mocap, nluwb, cfuwb
      yaw_source: imu # mocap, imu
      uwb_tag_id: 10 # nooploop uwb tag id
      initialPosition: [0.0, 0.0, 0.0, 1.57] # [x(m),y(m),z(m),yaw(rad)]  # only for simulation

Specify ``mocap_markerset_name`` for mocap or ``uwb_tag_id`` for nluwb. The ``initialPosition`` is only for simulation.

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

          # for matplot 3d display (slower simulation)
          python ../example.py --sim --dt=0.1 --vis=mpl

          # for none display (faster simulation)
          python ../example.py --sim --dt=0.1 --vis=null
        
        or use shell script:

        .. code:: bash
        
          cd launch/
          ./run_sim.sh # for simulation


    .. tab:: mocap

        - On UGV7 for example:

        .. code:: bash

          roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
          cd launch/
          roslaunch swarm_ros_bridge_ugv7.launch # communication

        - On ground station:

        .. code:: bash

          cd cd launch/
          roslaunch swarm_ros_bridge.launch # communication
          roslaunch vrpn.launch # mocap
          python example.py

        or use shell script:

        .. code:: bash

            cd launch/
            ./run_exp_mocap.sh

    .. tab:: nluwb

        - On UGV7 for example:

        .. code:: bash

          roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
          cd launch/
          roslaunch swarm_ros_bridge_ugv7.launch # communication

        - On ground station:

        .. code:: bash
        
          cd cd launch/
          roslaunch swarm_ros_bridge.launch # communication
          roslaunch linktrack.launch # uwb
          python example.py

        or use shell script:

        .. code:: bash

            cd launch/
            ./run_exp_nluwb.sh

    .. tab:: cfuwb