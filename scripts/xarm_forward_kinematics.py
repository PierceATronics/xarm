#!/usr/bin/env python
import rospy
import numpy as np
import time
import threading
from std_msgs.msg import Float64MultiArray
import modern_robotics as mr
#from xarm_parameters import xArmParameters

np.set_printoptions(precision=2)

class xArmParameters:
    '''
    Define the constant parameters of the xarm
    '''
    def __init__(self):

        #Define the xarm length parameters
        #self.L_23 = 97.5; self.L_34 = 97.5; self.L_4E = 170.4;
        self.L_23 = 0.0975; self.L_34=0.0975; self.L_4E=0.1704;
        #home (zero) position of the arm
        self.M = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, self.L_23 + self.L_34 + self.L_4E],
                           [0, 0, 0, 1]])

        #Screw axis representation of each joints configuration.
        self.S1 = np.array([0, 0, 1, 0, 0, 0])
        self.S2 = np.array([0, 1, 0, 0, 0, 0])
        self.S3 = np.array([0, 1, 0, -1*self.L_23, 0, 0])
        self.S4 = np.array([0, 1, 0, -1*(self.L_23 + self.L_34), 0, 0])
        self.S5 = np.array([0, 0, 1, 0, 0, 0])

        self.Slist = np.array([self.S1, self.S2, self.S3, self.S4, self.S5]).T

class xArmForwardKinematics:
    '''
    For each joint state received, compute the forward kinematics
    and publish the Transform T_end-effector.
    '''

    def __init__(self, node_name="xarm_forward_kinematics"):

        self.xarm_params = xArmParameters()

        self.node_name = node_name
        rospy.init_node(self.node_name)

        #subscriber to receive joint_states
        rospy.Subscriber("joint_states", Float64MultiArray, self._joint_states_callback)

        #Publish the end-effector as SE(3) matrix
        self.robot_config = rospy.Publisher("robot_config", Float64MultiArray, queue_size=10)

        self.looping_rate = rospy.Rate(100)

    def _joint_states_callback(self, msg):
        '''
        Callback function for joint state messages. Kickoff computation for
        Forward Kinematics.
        '''
        joint_states = msg.data

        #compute the forward kinematics, do not include the gripper state
        T_end_effector = mr.FKinSpace(self.xarm_params.M,
                                           self.xarm_params.Slist,
                                           joint_states[:4])

        print(repr(T_end_effector))

        msg = Float64MultiArray()
        msg.data = np.ravel(T_end_effector)
        self.robot_config.publish(msg)

    def run(self):
        '''
        '''
        try:
            while not rospy.is_shutdown():
                self.looping_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    xarm_forward_kinematics = xArmForwardKinematics()
    xarm_forward_kinematics.run()
