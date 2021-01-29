#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import serial

class xArmController:
    '''
    Send and receive commands to/from the xarm.
    '''
    def __init__(self, node_name="xarm_controller", port="/dev/ttyACM0"):
        '''
        '''

        self.node_name = node_name

    def run(self):

        try:
            while not rospy.is_shutdown():

                pass

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    xarm = xArmController()

    rospy.init_node(xarm.node_name)

    xarm.run()
