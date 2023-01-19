#!/usr/bin/env python
# coding:utf-8

import yaml
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

import geometry_msgs.msg
import sensor_msgs.msg

class TimeHelper:
    """Object containing all time-related functionality.

    This class mainly exists to support both real hardware and (potentially
    faster or slower than realtime) simulation with the same script.
    When running on real hardware, this class uses ROS time functions.
    The simulation equivalent does not depend on ROS.
    """
    def __init__(self):
        self.rosRate = None
        self.rateHz = None

    def time(self):
        """Returns the current time in seconds."""
        return rospy.Time.now().to_sec()

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        rospy.sleep(duration)

    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        if self.rosRate is None or self.rateHz != rateHz:
            self.rosRate = rospy.Rate(rateHz)
            self.rateHz = rateHz
        self.rosRate.sleep()

    def isShutdown(self):
        """Returns true if the script should abort, e.g. from Ctrl-C."""
        return rospy.is_shutdown()

    # animate, only for simulation
    def animate(self, duration):
        pass

class UGV:
    """Object representing a single robot.

    The bulk of the module's functionality is contained in this class.
    """

    def __init__(self, id, initialPosition, pos_source, yaw_source):
        """Constructor.

        Args:
            id (int): Integer ID in range [0, 255]. The last byte of the robot's
                radio address.
            initialPosition (iterable of float): Initial position of the robot:
                [x, y, z]. Typically on the floor of the experiment space with
                z == 0.0.
            pos_source: (mocap, nluwb, cfuwb)
            yaw_source: (mocap, imu)
        """
        self.id = id
        prefix = "/car" + str(id)
        self.prefix = prefix
        self.initialPosition = np.array(initialPosition)

        ## cmdvel_topic: /car${id}/cmd_vel # geometry_msgs/Twist type
        self.cmdVelPublisher = rospy.Publisher(prefix + "/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        ## pos subscribe topic: (mocap, nluwb, cfuwb)
        if pos_source == "mocap":
            # mocap_pose_topic: /vrpn_client_node/car${id}/pose  
            if yaw_source == "mocap": # read pose (pos+yaw)
                rospy.Subscriber("/vrpn_client_node" + prefix + "/pose", geometry_msgs.msg.PoseStamped, self.mocap_pose_cb, queue_size=1)
            else: # only read pos
                rospy.Subscriber("/vrpn_client_node" + prefix + "/pose", geometry_msgs.msg.PoseStamped, self.mocap_pos_cb, queue_size=1)
        elif pos_source == "nluwb":
            # nluwb_pos_topic: /nlink_linktrack_anchorframe0
            import nlink_parser.msg
            rospy.Subscriber("/nlink_linktrack_anchorframe0", nlink_parser.msg.LinktrackAnchorframe0, self.nluwb_pos_cb, queue_size=1)
        elif pos_source == "cfuwb":
            # cfuwb_pos_topic: /car${id}/pos
            rospy.Subscriber(prefix + "/pos", geometry_msgs.msg.PoseStamped, self.cfuwb_pos_cb, queue_size=1)
        else:
            raise Exception('[ugv.py] Unknown pos_source: {0}'.format(pos_source))            

        ## yaw subscribe topic: (mocap, imu)
        if yaw_source == "mocap":
            pass
        elif yaw_source == "imu":
            # imu_topic: /car${id}/imu/data
            rospy.Subscriber(prefix + "/imu", sensor_msgs.msg.Imu, self.imu_yaw_cb, queue_size=1)
        else:
            raise Exception('[ugv.py] Unknown yaw_source: {0}'.format(yaw_source))
        
        ## self state
        self.state_pos = np.zeros(3,dtype=float) # (x,y,z) m
        self.state_yaw = 0.0 # rad


    def stop(self, groupMask = 0):
        cmdVelMsg = geometry_msgs.msg.Twist()
        cmdVelMsg.linear.x = 0.0
        cmdVelMsg.linear.y = 0.0
        cmdVelMsg.linear.z = 0.0
        cmdVelMsg.angular.x = 0.0
        cmdVelMsg.angular.y = 0.0
        cmdVelMsg.angular.z = 0.0
        for _ in range(5): # loop for several times
            self.cmdVelPublisher.publish(cmdVelMsg)

    def cmdVelBody(self, vx, vy, w, groupMask = 0):
        cmdVelMsg = geometry_msgs.msg.Twist()
        cmdVelMsg.linear.x = vx
        cmdVelMsg.linear.y = vy
        cmdVelMsg.linear.z = 0.0
        cmdVelMsg.angular.x = 0.0
        cmdVelMsg.angular.y = 0.0
        cmdVelMsg.angular.z = w
        self.cmdVelPublisher.publish(cmdVelMsg)        

    def position(self):
        return np.array(self.state_pos)
    
    def yaw(self):
        return self.state_yaw
    
    def mocap_pose_cb(self, data):
        self.state_pos[0] = data.pose.position.x
        self.state_pos[1] = data.pose.position.y
        self.state_pos[2] = data.pose.position.z
        qw = data.pose.orientation.w
        qx = data.pose.orientation.x
        qy = data.pose.orientation.y 
        qz = data.pose.orientation.z 
        euler = euler_from_quaternion([qx, qy, qz, qw])
        self.state_yaw = euler[2]

    def mocap_pos_cb(self, data):
        self.state_pos[0] = data.pose.position.x
        self.state_pos[1] = data.pose.position.y
        self.state_pos[2] = data.pose.position.z
    
    def nluwb_pos_cb(self, data):
        # TODO
        pass

    def cfuwb_pos_cb(self, data):
        self.state_pos[0] = data.position.x / 1000.0 # pos x (mm -> m)
        self.state_pos[1] = data.position.y / 1000.0 # pos y (mm -> m)

    def imu_yaw_cb(self, data):
        w = data.orientation.w
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        euler = euler_from_quaternion([x, y, z, w])
        self.state_yaw = euler[2]                


class UGVServer:
    """Object for broadcasting commands to all robots at once.

    Also is the container for the individual :obj:`UGV` objects.

    Attributes:
        ugvs (List[UGV]): List of one UGV object per robot,
            as determined by the ugvs.yaml config file.
        ugvsById (Dict[int, UGV]): Index to the same UGV
            objects by their ID number
    """
    def __init__(self, ugvs_yaml="../launch/ugvs.yaml"):
        """Initialize the server. Waits for all ROS topics.

        Args:
            timeHelper (TimeHelper): TimeHelper instance.
            ugv_yaml (str): If ends in ".yaml", interpret as a path and load
                from file. Otherwise, interpret as YAML string and parse
                directly from string.
        """
        try:
            rospy.init_node("ugvAPI", anonymous=False)
        except:
            pass # rosnode is already initialized by other process

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
            pos_source = ugv["pos_source"]
            yaw_source = ugv["yaw_source"]
            car = UGV(id, initialPosition, pos_source, yaw_source)
            self.ugvs.append(car)
            self.ugvsById[id] = car

    def stop(self, groupMask = 0):
        for ugv in self.ugvs:
            ugv.stop(groupMask)

    def cmdVelBody(self, vx, vy, w, groupMask = 0):
        for ugv in self.ugvs:
            ugv.cmdVelBody(vx, vy, w, groupMask)

if __name__ == "__main__":
    # only for test
    UGVServer(ugvs_yaml="../../launch/ugvs.yaml")
    TimeHelper()
    rospy.spin()
