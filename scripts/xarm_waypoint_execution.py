#!/usr/bin/env python
import rospy
import numpy as np
from xarm.msg import JointCmd
import modern_robotics as mr
from std_srvs.srv import SetBool, SetBoolResponse
from xarm.msg import JointCmd



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

if __name__ == "__main__":
    rospy.init_node("xarm_waypoint_execution")
    main()
