#!/usr/bin/env python
import rospy
import numpy as np
from xarm.msg import JointCmd
import modern_robotics as mr
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray
from xarm.msg import JointCmd


class xArmWaypointExecutioner:

    def __init__(self, node_name="xarm_waypoint_executioner"):

        #states for collecting waypoints to executing trajectory
        self.COLLECT_WAYPOINTS    = 0
        self.GENERATE_TRAJECTORY  = 1
        self.EXECUTE_TRAJECTORY   = 2
        self.EXIT                 = 3

        self.state = self.COLLECT_WAYPOINTS
        self.exit_program = False
        self.waypoint_cnt = 0

        rospy.init_node(node_name)
        self.joint_cmd_pub = rospy.Publisher("joint_cmd", JointCmd, queue_size=10)
        #rospy.Subscriber("joint_states", Float64MultiArray, joint_state_callback)

        #the execution mode selects if the xarm controller is in execution mode
        # or if the controller is in waypoint collection mode.
        self.mode = 0

        #wait for service to be able to perform mode switching with the xarm controller
        rospy.wait_for_service("xarm_control_mode")
        self.set_xarm_ctrl_mode = rospy.ServiceProxy('xarm_control_mode', SetBool)

        #initially set the xarm_controller to waypoint collection mode.
        resp = self.set_xarm_ctrl_mode(self.mode)

    def run(self):
        '''
        Run the state machine to collect waypoints and generate + execute trajectories.
        '''
        while (not self.exit_program) or (not rospy.is_shutdown()):

            if(self.state == self.COLLECT_WAYPOINTS):
                #press 'r' to record the current end-effector position
                #as a waypoint
                value = input("Press 'r' to record an end-effector waypoint. Else press 'g' to generate trajectory.")
                if(value == 'r'):
                    self.waypoint_cnt += 1
                    print("Waypoint %d collected" % (self.waypoint_cnt))
                    self.state = self.COLLECT_WAYPOINTS
                elif(value == 'g'):
                    self.state = self.GENERATE_TRAJECTORY
                else:
                    self.state = self.EXIT

            elif(self.state == self.GENERATE_TRAJECTORY):
                #given the trajector set W = {w1, w2, ..., wk} (wi is SE(3))
                #generate a joint-space trajectory THETA = {theta1, theta2, ..., thetam}
                #That achieves the given waypoints.
                print("Generating trajectory to achieve the %d recorded waypoints" % self.waypoint_cnt)

                self.waypoint_cnt = 0
                value = input("Trajectory generation successful! Press 'e' to execute.")

                if(value == 'e'):
                    self.state = self.EXECUTE_TRAJECTORY

                else:
                    self.state = self.EXIT

            elif(self.state == self.EXECUTE_TRAJECTORY):
                self.mode = 1
                #set the xarm_controller to execution mode.
                resp = self.set_xarm_ctrl_mode(self.mode)

                #sent the joint angle trajectory THETA to the xarm_controller
                #to drive the servos of the arm.

                self.state = self.EXIT
            else:
                #exit the program!
                self.exit_program = True
                return

"""
def joint_state_callback():
    '''
    Callback function to receive xArm joint_state messages.
    '''

def main():
    '''
    State machine to collect waypoints and generate+execute trajectories.
    '''
    #states for collecting waypoints to executing trajectory
    COLLECT_WAYPOINTS    = 0
    GENERATE_TRAJECTORY  = 1
    EXECUTE_TRAJECTORY   = 2
    EXIT                 = 3

    state = COLLECT_WAYPOINTS
    exit_program = False
    waypoint_cnt = 0

    joint_cmd_pub = rospy.Publisher("joint_cmd", JointCmd, queue_size=10)
    rospy.Subscriber("joint_states", Float64MultiArray, joint_state_callback)

    #the execution mode selects if the xarm controller is in execution mode
    # or if the controller is in waypoint collection mode.
    mode = 0

    #wait for service to be able to perform mode switching with the xarm controller
    rospy.wait_for_service("xarm_control_mode")
    xarm_ctrl_mode = rospy.ServiceProxy('xarm_control_mode', SetBool)

    #initially set the xarm_controller to waypoint collection mode.
    resp = xarm_ctrl_mode(mode)


    while not exit_program:

        if(state == COLLECT_WAYPOINTS):
            #press 'r' to record the current end-effector position
            #as a waypoint
            value = input("Press 'r' to record an end-effector waypoint. Else press 'g' to generate trajectory.")
            if(value == 'r'):
                waypoint_cnt += 1
                print("Waypoint %d collected" % (waypoint_cnt))
                state = COLLECT_WAYPOINTS
            elif(value == 'g'):
                state = GENERATE_TRAJECTORY
            else:
                state = EXIT

        elif(state == GENERATE_TRAJECTORY):
            #given the trajector set W = {w1, w2, ..., wk} (wi is SE(3))
            #generate a joint-space trajectory THETA = {theta1, theta2, ..., thetam}
            #That achieves the given waypoints.
            print("Generating trajectory to achieve the %d recorded waypoints" % waypoint_cnt)

            waypoint_cnt = 0
            value = input("Trajectory generation successful! Press 'e' to execute.")

            if(value == 'e'):
                state = EXECUTE_TRAJECTORY

            else:
                state = EXIT

        elif(state == EXECUTE_TRAJECTORY):
            mode = 1
            #set the xarm_controller to execution mode.
            resp = xarm_ctrl_mode(mode)

            #sent the joint angle trajectory THETA to the xarm_controller
            #to drive the servos of the arm.

            state = EXIT
        else:
            #exit the program!
            exit_program = True
            return
"""

if __name__ == "__main__":
    xarm_waypoint_executioner = xArmWaypointExecutioner()
    xarm_waypoint_executioner.run()
