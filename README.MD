GoalieBot README:

This is a repository for the GoalieBot project for ECE470. The robot we plan to base our project off of is the UR3 used in the lab component of this course. 

So far, we have implemented a UR3 robot with a spherical object as a ball.

We can control the robot movements via Python implementation, and have incorporated a proximity sensor for detecting the presence of a ball. We can read the detection data with our python script as well to see if the robot sees the ball or not.

In the python implementation, we first decribe all of the UR3 Robot Joint Positions in terms of position and orientation, then calculate the various screw matrices for each joint. Then, using a forward kinematic model, we can take an input of thetas, or UR3 joint angles, and calculate the final position and orientation of the UR3 end effector. 

In our code, this final position is verified by comparing to the actual position of the end-effector that can be read out from coppeliasim. In the ball_motion branch (soon to be merged), there is code demonstrating our object detection logic using the proximity sensor attached to our robot in the simulator. More sophisticated and robust methods of object detection will be used in the next updates.

Our final plan is to detect the ball with our proximity sensor, get a predicted ball position in the plane of the soccer "goal", and then use inverse kinematics to derive a set of joint angles needed for the UR3 end effector to reach the ball. Then, using forward kinematics, those thetas can be input into the UR3 to move the end effector and block the ball.


