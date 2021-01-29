#!/usr/bin/env python
import rospy
import numpy as np
import xarm_servo_controller
from std_msgs.msg import Float64MultiArray

def RAD_2_DEG(x):
    return(x * 180.0 / np.pi)

def DEG_2_RAD(x):
    return(x * np.pi / 180.0)

class Joint:
    '''
    Basic structure to define each joint
    '''
    def __init__(self, id, name):
        self.id = id
        self.name = name


class xArmController:
    '''
    Send and receive commands to/from the xarm.
    '''
    def __init__(self, node_name="xarm_controller", port="/dev/ttyACM0"):
        '''
        '''
        self.node_name = node_name
        rospy.init_node(node_name)
        self.arm = xarm_servo_controller.Controller(port, debug=False)

        rospy.Subscriber("joint_cmd", Float64MultiArray, self._joint_cmd_callback)

        #The ID orders may be different depending on how the arm is assembled.
        self.J1 = Joint(id=2, name='joint_1')
        self.J2 = Joint(3, 'joint_2')
        self.J3 = Joint(4, 'joint_3')
        self.J4 = Joint(5, 'joint_4')
        self.J5 = Joint(6, 'joint_5')
        self.GRIPPER = Joint(1, 'gripper')
        self.joint_list = [self.J1, self.J2, self.J3, self.J4, self.J5, self.GRIPPER]

        self.current_joint_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        #zero the robotic arm
        self._set_joint_positions(self.current_joint_state)

        self.looping_rate = rospy.Rate(100)


    def _joint_cmd_callback(self, msg):
        '''
        Callback function for recieving published joint commands.
        Execute the command by sending the desired joint positions to the arm.
        '''

        joint_positions = list(msg.data)
        self._set_joint_positions(joint_positions)


    def _set_joint_positions(self, joint_positions, durations=[1000, 1000, 1000, 1000, 1000, 1000]):
        '''
        Set the joint position of the arm. Achieve the desired position for each joint given
        the duration of movement for each joint (measured in milliseconds).
        '''
        for index, joint in enumerate(self.joint_list):
            if joint.name == "joint_4":
                joint_positions[index] *= -1

            self.arm.setPosition(joint.id, RAD_2_DEG(joint_positions[index]), duration=int(durations[index]), wait=False)

        #Do a timer thingy here to estimate the current joint state

        self.current_joint_state = np.array(joint_positions)

    def run(self):

        try:
            while not rospy.is_shutdown():

                self.looping_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    xarm = xArmController()

    xarm.run()
