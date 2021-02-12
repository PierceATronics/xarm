#!/usr/bin/env python
import rospy
import numpy as np
from xarm.msg import JointCmd
import modern_robotics as mr

#states for collecting waypoints to executing trajectory
COLLECT_WAYPOINTS     = 0
GENERATE_TRAJECTORY  = 1
EXECUTE_TRAJECTORY   = 2
EXIT                 = 3

state = COLLECT_WAYPOINT
exit_program = False

def main():

    while not exit_program:

        if(state == COLLECT_WAYPOINTS):
            #press 'r' to record the current end-effector position
            #as a waypoint
            pass

        elif(state == GENERATE_TRAJECTORY):
            #given the trajector set W = {w1, w2, ..., wk} (wi is SE(3))
            #generate a joint-space trajectory THETA = {theta1, theta2, ..., thetam}
            #That achieves the given waypoints.

            pass

        elif(state == EXECUTE_TRAJECTORY):

            #sent the joint angle trajectory THETA to the xarm_controller
            #to drive the servos of the arm.
            pass

        else:
            #exit the program!
            exit_program = True
            return

if __name__ == "__main__":
    main()
