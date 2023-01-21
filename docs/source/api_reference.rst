.. _api_reference:

API reference
==============

The most commonly used API:

.. code:: python

    ugv.cmdVelBody(vx,vy,w) # send speeds in body frame
    ugv.stop() # stop
    allugvs.stop() # stop all ugvs
    ugv.position() # return xyz array (m)
    ugv.yaw() # return yaw (rad)
    timeHelper.time() # return time (s)
    timeHelper.sleepForRate(rate) # sleep for rate
    timeHelper.sleep(time) # sleep for seconds
    timeHelper.plot_data() # plot x-y, yaw-t (sim only)