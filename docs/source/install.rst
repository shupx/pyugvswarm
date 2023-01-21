.. _install:

Install
=========

Pyugvswarm runs on **Ubuntu Linux** in one of the following configurations:

====== ====== =======
Ubuntu Python ROS
------ ------ -------
20.04  3.7    Noetic
18.04  2.7    Melodic
====== ====== =======

There are three methods to install pyugvswarm python package:

.. tabs::

   .. tab:: Method1 (recommended)
    
        Install ``pyugvswarm`` python package from source:

        .. code:: bash

            git clone https://gitee.com/shu-peixuan/pyugvswarm.git
            cd pyugvswarm/
            pip install -e . # use pip3 if you use python3 in ubuntu18

        Check you have pyugvswarm package in your PYTHON environment:

        .. code:: bash

            pip show pyugvswarm # use pip3 if you use python3 in ubuntu18

        Then you can import ``pyugvswarm`` class in your python scripts in any folders.

        To uninstall pyugvswarm:

        .. code:: bash

            pip uninstall pyugvswarm # use pip3 if you use python3 in ubuntu18

   .. tab:: Method2
    
        You can simply copy the ``pyugvswarm/pyugvwarm/`` folder and optionally ``config/`` folder into your project:

        .. code:: bash

            git clone https://gitee.com/shu-peixuan/pyugvswarm.git
            cp -r pyugvswarm/pyugvswarm ${your_scripts_path}
            cp -r pyugvswarm/config ${your_scripts_path} # optional
        
        The folder structure should be like this:

        .. code:: bash

            |__ Your scripts folder
            |_ pyugvswarm/
            |_ config/
            |  |_ ugvs.yaml
            |_ your_python_script.py

   .. tab:: Method3 (simple)

        Write your python scripts directly in this repository like the ``example.py``.

        .. code:: bash

            git clone https://gitee.com/shu-peixuan/pyugvswarm.git
            cd pyugvswarm/
            touch your_python_script.py


