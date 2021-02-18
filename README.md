# xarm

This is a ROS1 project to control the HiWonder xArm 1S 5 degree of freedom robotic arm. This includes forward kinematics, inverse kinematics, and waypoint trajectory generation.

Start by plugging in the xArm (via TTL) and turning it one.
Then run the following command to establish a connection with the
xArm servo controller

```
rosrun xarm xarm_controller
```

Next, the forward kinematics node can be ran to provide the end-effector
configuration expressed at a SE(3) matrix T = (R, p). R is the rotation
matrix and p is the position matrix.

```
rosrun xarm xarm_forward_kinematics
```
