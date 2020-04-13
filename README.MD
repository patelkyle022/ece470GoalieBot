GoalieBot README:

This is a repository for the GoalieBot project for ECE470. The robot we plan to base our project off of is the UR3 used in the lab component of this course. 

So far, we have implemented a UR3 robot with a spherical object as a ball.

We can control the robot movements via Python implementation, and have incorporated a proximity sensor for detecting the presence of a ball. We can read the detection data with our python script as well to see if the robot sees the ball or not.

In the python implementation, we first decribe all of the UR3 Robot Joint Positions in terms of position and orientation, then calculate the various screw matrices for each joint. Then, using a forward kinematic model, we can take an input of thetas, or UR3 joint angles, and calculate the final position and orientation of the UR3 end effector. 

In our code, this final position is verified by comparing to the actual position of the end-effector that can be read out from coppeliasim. In the ball_motion branch (soon to be merged), there is code demonstrating our object detection logic using the proximity sensor attached to our robot in the simulator. More sophisticated and robust methods of object detection will be used in the next updates.

As of Update 3 (April 12th), we have implemented random linear motion of our ball in Python from a fixed starting point. All path points are in the world frame. We do this by generating linearly spaced arrays with random y and z coordinates at the endpoint of the path with the x-coordinate of the path's endpoint being the x-coordinate of the end-effector of the robot in the world frame.

We are also able to predict where in the vertical plane of the goal line the ball will end up to a reasonable degree of accuracy (typically less than 0.1m in error) in the base frame. This calculation requires us to transform the detection points for the ball from the world frame to our robot's base frame, and find the vector that crosses through the goal line to determine the desired point the robot's end-effector will travel to.

Our implementation of Inverse Kinematics-based motion planning does not work as intended in that the output joint angles for our desired end-effector configuration are unreasonably large. For the final update, we will work to figure out why this is the case.



