#!/usr/bin/env python
# main class of simulation.

import math

import yaml
import numpy as np

from matplotlib.pyplot import *

class RobotState:
    def __init__(self):
        self.pos = np.zeros(3,dtype = float) # m
        self.vel = np.zeros(3,dtype = float) # m/s
        self.acc = np.zeros(3,dtype = float) # m/s^2
        self.yaw = 0.0 # rad
        self.omega = np.zeros(3,dtype = float) # rad/s

# ugvs keep reference to this object to ask what time it is.
# also does the plotting.

class TimeHelper:
    def __init__(self, vis, dt, writecsv, disturbanceSize, maxVel=np.inf, videopath=None):
        if vis == "mpl":
            from .visualizer import visMatplotlib
            self.visualizer = visMatplotlib.VisMatplotlib()
        elif vis == "vispy":
            raise Exception('[ugvSim] unsupported vis = {0}'.format(vis))
            # from .visualizer import visVispy
            # resizable = videopath is None
            # self.visualizer = visVispy.VisVispy(resizable=resizable)
        elif vis == "null":
            from .visualizer import visNull
            self.visualizer = visNull.VisNull()
        else:
            raise Exception("Unknown visualization backend: {}".format(vis))
        self.t = 0.0
        self.dt = dt
        # Since our integration/animation ticks are always the fixed duration
        # dt, any call to sleep() with a non-multiple of dt will have some
        # "leftover" time. Keep track of it here and add extra ticks in future.
        self.sleepResidual = 0.0
        self.ugvs = []
        self.disturbanceSize = disturbanceSize
        self.maxVel = maxVel
        self.writecsv = writecsv
        # if writecsv:
        if True: # always update output in sim, modified by spx
            from . import output
            self.output = output.Output() # self.output.data[id] is the output data
        else:
            self.output = None

        if videopath is not None:
            from .videowriter import VideoWriter
            frame = self.visualizer.render()
            self.videoWriter = VideoWriter(videopath, dt, frame.shape[:2])
        else:
            self.videoWriter = None

    def time(self):
        return self.t

    def step(self, duration):
        self.t += duration
        for car in self.ugvs:
            car.integrate(duration, self.disturbanceSize, self.maxVel)
            car.flip()

    # should be called "animate" or something
    # but called "sleep" for source-compatibility with real-robot scripts
    def sleep(self, duration):
        self.animate(duration)

    # Mock for abstraction of rospy.Rate.sleep().
    def sleepForRate(self, rate):
        # TODO: account for rendering time, or is it worth the complexity?
        self.sleep(1.0 / rate)

    # animate, only for simulation
    def animate(self, duration):
        # operator // has unexpected (wrong ?) behavior for this calculation.
        ticks = math.floor((duration + self.sleepResidual) / self.dt)
        self.sleepResidual += duration - self.dt * ticks
        assert -1e-9 <= self.sleepResidual < self.dt
        for _ in range(int(ticks)):
            self.visualizer.update(self.t, self.ugvs)
            if self.output:
                self.output.update(self.t, self.ugvs)
            if self.videoWriter is not None:
                frame = self.visualizer.render()
                self.videoWriter.writeFrame(frame)
            self.step(self.dt)

    # Mock for abstraction of rospy.is_shutdown().
    def isShutdown(self):
        return False

    def addObserver(self, observer):
        self.observers.append(observer)

    def _atexit(self):
        if self.videoWriter is not None:
            self.videoWriter.close()
    
    # plot data of writecsv output(always enabled), added by spx
    def plot_data(self):
        data = self.output.data # dict {ugv_id: ugv_data}
        # plot x-y curves
        figure()
        for ugv_id, data_id in data.items():
            t, x, y, z, yaw = data_id.T
            plot(x,y, linewidth=1.0, label='UGV'+str(ugv_id))
        xlabel('x/m'); ylabel('y/m'); axis('equal')
        legend(); grid(ls='--')
        title('ugvs trajectory')  
        # plot yaw-t curves
        figure()
        for ugv_id, data_id in data.items():
            t, x, y, z, yaw = data_id.T
            plot(t, yaw, linewidth=1.0, label='UGV'+str(ugv_id)) 
        xlabel('t/s'); ylabel('yaw/rad')
        legend(); grid(ls='--')
        title('ugvs yaw')   
        # show and close figures
        show(block=False)
        try: input('Please press Enter to close all figures... \n')
        except: pass            


class UGV:
    # ugv modes.
    MODE_IDLE = 0
    MODE_HIGH_POLY = 1
    MODE_LOW_FULLSTATE = 2
    MODE_LOW_POSITION = 3
    MODE_LOW_VELOCITY = 4
    MODE_LOW_VELOCITY_BODY = 5

    def __init__(self, id, initialPosition, timeHelper):
        # Core.
        self.id = id
        self.groupMask = 0
        self.initialPosition = np.array(initialPosition,dtype=float)
        self.time = lambda: timeHelper.time()

        # Commander.
        self.mode = UGV.MODE_IDLE
        self.setState = RobotState()

        # State. Public np.array-returning getters below for physics state.
        self.state = RobotState()
        self.state.pos = self.initialPosition[:3]
        self.state.yaw = self.initialPosition[3]

        # Double-buffering: Ensure that all CARs observe the same world state
        # during an integration step, regardless of the order in which their
        # integrate() methods are called. flip() swaps front and back state.
        # See http://gameprogrammingpatterns.com/double-buffer.html for more
        # motivation.
        self.backState = self.state

    def setGroupMask(self, groupMask):
        self.groupMask = groupMask

    def stop(self, groupMask = 0):
        if self._isGroup(groupMask):
            self.mode = UGV.MODE_IDLE

    def position(self):
        return np.array(self.state.pos)

    def yaw(self):
        return float(self.state.yaw)

    def cmdVelBody(self, vx, vy, w, groupMask = 0):
        self.mode = UGV.MODE_LOW_VELOCITY_BODY
        self.setState.vel = np.array([vx, vy, 0.0],dtype=float) 
        self.setState.omega = np.array([0.0, 0.0, w],dtype=float)

    ################# simulation only functions ######################
    def rotBodyToWorld(self):
        '''
        rotation matrix between body(FLU) and world frame(ENU)
        '''
        yaw = self.yaw()
        x_body = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        y_body = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        z_body = np.array([0.0, 0.0, 1.0])
        return np.column_stack([x_body, y_body, z_body])

    def velocity(self):
        return np.array(self.state.vel)

    def acceleration(self):
        return np.array(self.state.acc)

    def integrate(self, time, disturbanceSize, maxVel):
        '''
        Euler integration
        '''
        if self.mode == UGV.MODE_HIGH_POLY:
            # self.setState = firm.plan_current_goal(self.planner, self.time())
            print('[ugvSim] unsupported mode = UGV.MODE_HIGH_POLY')
            return
        
        # get setState command
        setState = self.setState # shallow copy!

        if self.mode == UGV.MODE_IDLE:
            return

        # get velocity
        if self.mode in (UGV.MODE_HIGH_POLY, UGV.MODE_LOW_FULLSTATE, UGV.MODE_LOW_POSITION):
            velocity = (setState.pos.copy() - self.state.pos) / time
        elif self.mode == UGV.MODE_LOW_VELOCITY:
            velocity = setState.vel.copy()
        elif self.mode == UGV.MODE_LOW_VELOCITY_BODY:
            velocity = np.dot(self.rotBodyToWorld(), setState.vel.copy()) # translate into world frame
        else:
            raise ValueError("Unknown flight mode.")

        # Limit velocity for realism.
        '''
        Note: This will result in the state having a different velocity than
        the setState in HIGH_POLY and LOW_FULLSTATE modes even when no
        clamping occurs, because we are essentially getting rid of the
        feedforward commands. We assume this is not a problem.
        '''
        velocity = np.clip(velocity, -maxVel, maxVel) # limit velocity
        disturbance = disturbanceSize * np.random.normal(size=3)
        velocity = velocity + disturbance # add disturbance

        # position integration
        self.backState.pos = self.state.pos.copy() + time * velocity

        # velocity update
        self.backState.vel = velocity

        # yaw(yaw_rate) integration(update)
        if self.mode == UGV.MODE_LOW_POSITION:
            yawRate = (setState.yaw - self.state.yaw) / time
            self.backState.yaw = setState.yaw
            self.backState.omega = np.array([0.0, 0.0, yawRate])
        elif self.mode in (UGV.MODE_LOW_VELOCITY, UGV.MODE_LOW_VELOCITY_BODY):
            self.backState.omega = setState.omega.copy()
            self.backState.yaw = self.state.yaw + time * setState.omega[2]
        elif self.mode in (UGV.MODE_HIGH_POLY, UGV.MODE_LOW_FULLSTATE):
            # In HIGH_POLY and LOW_FULLSTATE, yaw and omega are copied from setState
            self.backState.omega = setState.omega.copy()
            self.backState.yaw = setState.yaw
        else:
            raise ValueError("Unknown flight mode.")

        # acc update
        if self.mode in (UGV.MODE_LOW_POSITION, UGV.MODE_LOW_VELOCITY, UGV.MODE_LOW_VELOCITY_BODY):
            self.backState.acc = (self.backState.vel - self.state.vel) / time # differential
        elif self.mode in (UGV.MODE_HIGH_POLY, UGV.MODE_LOW_FULLSTATE):
            self.backState.acc = setState.acc.copy()
        else:
            raise ValueError("Unknown flight mode.") 

    def flip(self): # update self.state
        # Swap double-buffered state. Called at the end of the tick update,
        # after *all* CARs' integrate() methods have been called.
        self.state, self.backState = self.backState, self.state

    # "private" methods
    def _isGroup(self, groupMask):
        return groupMask == 0 or (self.groupMask & groupMask) > 0


class UGVServer:
    def __init__(self, timeHelper, ugvs_yaml="../launch/ugvs.yaml"):
        """Initialize the server.

        Args:
            timeHelper (TimeHelper): TimeHelper instance.
            ugvs_yaml (str): If ends in ".yaml", interpret as a path and load
                from file. Otherwise, interpret as YAML string and parse
                directly from string.
        """
        if ugvs_yaml.endswith(".yaml"):
            with open(ugvs_yaml, 'r') as ymlfile:
                cfg = yaml.safe_load(ymlfile)
        else:
            cfg = yaml.safe_load(ugvs_yaml)

        self.ugvs = []
        self.ugvsById = dict()
        for ugv in cfg["ugvs"]:
            id = int(ugv["id"])
            initialPosition = ugv["initialPosition"]
            car = UGV(id, initialPosition, timeHelper)
            self.ugvs.append(car)
            self.ugvsById[id] = car

        self.timeHelper = timeHelper
        self.timeHelper.ugvs = self.ugvs

    def stop(self, groupMask = 0):
        for ugv in self.ugvs:
            ugv.stop(groupMask)

    def cmdVelBody(self, vx, vy, w, groupMask = 0):
        for ugv in self.ugvs:
            ugv.cmdVelBody(vx, vy, w, groupMask)

    # def cmdVelWorld(self, groupMask = 0):
    #     for ugv in self.ugvs:
    #         ugv.cmdVelWorld(vx, vy, w, groupMask)
