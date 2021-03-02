#!/usr/bin/env python
import rospy
import numpy as np
from xarm.msg import JointCmd
import modern_robotics as mr
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray
from xarm.msg import JointCmd
import time
import math

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

        self.xarm_params = xArmParameters()

        rospy.init_node(node_name)
        self.joint_cmd_pub = rospy.Publisher("joint_cmd", JointCmd, queue_size=10)
        rospy.Subscriber("robot_config", Float64MultiArray, self._robot_config_callback)

        #the execution mode selects if the xarm controller is in execution mode
        # or if the controller is in waypoint collection mode.
        self.mode = 0

        #wait for service to be able to perform mode switching with the xarm controller
        rospy.wait_for_service("xarm_control_mode")
        self.set_xarm_ctrl_mode = rospy.ServiceProxy('xarm_control_mode', SetBool)

        #initially set the xarm_controller to waypoint collection mode.
        resp = self.set_xarm_ctrl_mode(self.mode)
        self.robot_config = np.copy(self.xarm_params.M)


    def _robot_config_callback(self, robot_config_msg):
        '''
        Callback function receiving the end-effector configuration via forward
        kinematics.
        '''

        if(self.mode == self.COLLECT_WAYPOINTS):
            #extract the message and reshape to the 4x4 configuration matrix SE(3)
            self.robot_config = np.array(robot_config_msg.data).reshape((4, 4))

    def _generate_trajectory(self, W):
        '''
        Generates Cartesian Trajectory (starting from home position)
        through the recorded waypoint end-effector configurations.
        This method uses cubic time scaling.
        '''
        POINTS_PER_DIST = 50.0 #points per millimeter
        TIME_PER_DIST = 10.0 #points per millimeter

        T_start_i = self.xarm_params.M
        joint_traj = np.array([0.0, 0.0, 0.0, 0.0, 0.0])


        for i, T_end_i in enumerate(W):
            #compute the euclidean distance between consectutive
            #waypoints.
            [x_i, y_i, z_i] = np.ravel(T_end_i[0:3, 3])
            [x_j, y_j, z_j] = np.ravel(T_start_i[0:3, 3])

            dist = np.sqrt((x_i - x_j)**2 + (y_i - y_j)**2 + (z_i - z_j)**2)

            #Apply heuristic to calculate the number of points in a point-to-point
            #trajectory.
            #N must be greater than or equal to 2.
            N = math.ceil(POINTS_PER_DIST * dist)
            if N < 2:
                N = 2

            Tf = TIME_PER_DIST * dist
            print(Tf)
            #Generate a point-to-point trajectory between two consecutive waypoints.
            traj_i = mr.ScrewTrajectory(T_start_i, T_end_i, Tf, N, 3)
            print(len(traj_i))

            #set the initial condition for thetalist.
            if(i == 0):
                thetalist = [0.00, 0.00, 0.00, 0.00, 0.00]

            #perform inverse kinematics for each configuration along the
            #point-to-point trajectory.
            for j, X in enumerate(traj_i[1:]):
                [thetalist, success] = mr.IKinSpace(self.xarm_params.Slist,
                                                    self.xarm_params.M,
                                                    X, thetalist,
                                                    0.01,
                                                    0.01)
                #append the joint values to the joint trajectory list.
                #joint_traj = np.vstack((joint_traj, thetalist))
                print("Success " + str(success))
            joint_traj = np.vstack((joint_traj, thetalist))
            T_start_i = np.copy(T_end_i)
        print(joint_traj)
        return(joint_traj, Tf)



    def run(self):
        '''
        Run the state machine to collect waypoints and generate + execute trajectories.
        '''
        W = []
        while (not self.exit_program) or (not rospy.is_shutdown()):

            if(self.state == self.COLLECT_WAYPOINTS):
                #press 'r' to record the current end-effector position
                #as a waypoint
                value = input("Press 'r' to record an end-effector waypoint. Else press 'g' to generate trajectory.")
                if(value == 'r'):
                    #small waittime to ensure the most current position is read
                    #results from slow position readings from the servos.
                    time.sleep(0.125)
                    self.waypoint_cnt += 1
                    print("Waypoint %d collected" % (self.waypoint_cnt))
                    print(repr(self.robot_config))
                    W.append(self.robot_config)

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

                joint_traj, Tf = self._generate_trajectory(W)

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
                time.sleep(1)
                #sent the joint angle trajectory THETA to the xarm_controller
                #to drive the servos of the arm.
                joint_cmd_msg = JointCmd()
                for index, joint in enumerate(joint_traj):


                    #initially lets do nothing with the end-effector
                    joint_cmd_msg.joint_pos = list(joint) + [0.0]

                    if(index == 0):
                        joint_cmd_msg.duration = [1000, 1000, 1000, 1000, 1000, 1000]
                        self.joint_cmd_pub.publish(joint_cmd_msg)
                        time.sleep(0.90)
                    else:
                        joint_cmd_msg.duration = np.array([Tf, Tf, Tf, Tf, Tf, Tf]) * 1000.0
                        self.joint_cmd_pub.publish(joint_cmd_msg)
                        time.sleep(Tf*0.95)

                W = []
                self.state = self.EXIT
            else:
                #exit the program!
                self.exit_program = True
                return

if __name__ == "__main__":
    xarm_waypoint_executioner = xArmWaypointExecutioner()
    xarm_waypoint_executioner.run()
