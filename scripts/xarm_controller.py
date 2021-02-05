#!/usr/bin/env python
import rospy
import numpy as np
import time
import threading
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

        #Servo control module
        self.arm = xarm_servo_controller.Controller(port, debug=False)

        #subscribe to joint commands
        rospy.Subscriber("joint_cmd", Float64MultiArray, self._joint_cmd_callback)

        #publish the estimated joint states (this are simply interpolated)
        self.joint_state_pub = rospy.Publisher("joint_states", Float64MultiArray, queue_size=10)

        #joint state estimation thread.
        state_estimation_thread = threading.Thread(target=self._state_estimation_thread, daemon=True)

        #The ID orders may be different depending on how the arm is assembled.
        self.J1 = Joint(id=2, name='joint_1')
        self.J2 = Joint(3, 'joint_2')
        self.J3 = Joint(4, 'joint_3')
        self.J4 = Joint(5, 'joint_4')
        self.J5 = Joint(6, 'joint_5')
        self.GRIPPER = Joint(1, 'gripper')
        self.joint_list = [self.J1, self.J2, self.J3, self.J4, self.J5, self.GRIPPER]

        self.joint_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #radians
        self.set_joint_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #radians
        self.set_joint_duration = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) #seconds

        #zero the robotic arm
        self._set_joint_positions(self.set_joint_state)
        self.prev_joint_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        #Update a 100Hz.
        self.looping_rate = rospy.Rate(100)
        self.threading_looping_rate = rospy.Rate(200)

        self.start_state_estimation = False
        self.state_estimation_running = True
        state_estimation_thread.start()


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

        @param duration: The duration time for each joint to go from the current joint state to the desired joint state.
        '''
        for index, joint in enumerate(self.joint_list):
            if joint.name == "joint_4":
                joint_positions[index] *= -1

            self.arm.setPosition(joint.id, RAD_2_DEG(joint_positions[index]), duration=int(durations[index]), wait=False)

        #Do a timer thingy here to estimate the current joint state

        self.set_joint_state = np.array(joint_positions)
        self.set_joint_duration = np.array(durations) / 1000.0
        self.prev_joint_state = np.copy(self.joint_state)
        self.start_state_estimation = True #kickoff the state estimation thread

    def _state_estimation_thread(self):
        '''
        A thread that estimates the state of the joints given the joint command and the runtime since the
        joint command was started.
        '''
        while(self.state_estimation_running):

            if(self.start_state_estimation): #kickoff the predictions of state when a new joint state command is received.

                self.start_state_estimation = False
                start_time = time.time()
                delta_t = 0.0
                state_unreached = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                elapsed_time = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                m = (self.set_joint_state - self.prev_joint_state) / self.set_joint_duration

                #estimate each of the joint states
                while((not self.start_state_estimation) and (delta_t <= np.max(self.set_joint_duration))):

                    self.joint_state = self.prev_joint_state + m * elapsed_time
                    delta_t = time.time() - start_time
                    elapsed_time = state_unreached * delta_t

                    #determine which joints have reached their inteneded states
                    #if a joint has reach is state, don't update its joint state
                    #any more
                    for i in range(6):
                        if(delta_t >= self.set_joint_duration[i]):
                            state_unreached[i] = 0.0
                            self.joint_state[i] = self.set_joint_state[i]
                        else:
                            state_unreached[i] = 1.0

                    self.threading_looping_rate.sleep()

            else:
                self.threading_looping_rate.sleep()

    def run(self):
        '''
        Main running loop. Publishes joint states at set frequency.
        '''
        try:
            while not rospy.is_shutdown():

                #publish the joint states at 100Hz.
                joint_state_msg = Float64MultiArray()
                joint_state_msg.data = list(self.joint_state)
                self.joint_state_pub.publish(joint_state_msg)

                self.looping_rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    xarm = xArmController()

    xarm.run()
