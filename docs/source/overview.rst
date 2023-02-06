.. _overview:

Overview
=========

This is a python package wrapper around unmanned ground vehicle (UGV) swarm positioning / control ROS1 nodes. We offer python APIs which simplifies the development of UGV swarm. A lightweight python simulator is also provided to  validate your program before the real experiment.

|  github repository: 
|  https://github.com/shupx/pyugvswarm.git
|  gitee repository: 
|  https://gitee.com/shu-peixuan/pyugvswarm.git

This python wrapper should be used with other submodules like `swarm_ros_bridge <https://gitee.com/shu-peixuan/swarm_ros_bridge>`_ (for communication) and positioning ROS packages. The framework is shown below:

.. image:: ../../pictures/pyugvswarm_framework.png

pyugvswarm runs on **Ubuntu Linux** in one of the following configurations:

====== ====== =======
Ubuntu Python ROS
------ ------ -------
20.04  3.7    Noetic
18.04  2.7    Melodic
====== ====== =======