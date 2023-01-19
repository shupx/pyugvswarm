import os
import math
import numpy as np


class Output:
    def __init__(self):
        self.data = dict()
        self.starttime = None

    def update(self, t, ugvs):
        for car in ugvs:
            x, y, z = car.position()
            # vx, vy, vz = car.velocity()
            # ax, ay, az = car.acceleration()
            yaw = car.yaw()
            if car.id not in self.data:
                self.data[car.id] = np.empty((0, 5), float)
                self.starttime = t
            self.data[car.id] = np.vstack([self.data[car.id], np.array([t - self.starttime, x, y, z, yaw])])
