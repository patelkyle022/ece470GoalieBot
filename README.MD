GoalieBot README:

This is a repository for the GoalieBot project for ECE470. The robot we based our project off of is the UR3 used in the lab component of this course. 

We were able to achieve our goals of using sensor data collection, inverse kinematics, and motion planning to accurately defend a goal from a soccer ball coming towards it in a random direction at a random speed.

We made use of Coppeliasim as our simulation environment and controlled the UR3 using Coppeliasim's remote API to perform sensor readings and joint movements. We were also able to remotely move a primitive sphere object which was chosen to represent a soccer ball. Both actions were performed simultaneously through the use of multithreading.

The most challenging aspects of this project were performing the inverse kinematics to get the robot to move to the desired position, and making sure the simulation ran smoothly. At first, we attempted to use Modern Robotics Python library for its Newton-Raphson iterative inverse kinematics method, to incorrect results that were also difficult to debug. Switching to the analytical inverse kinematics solution finally used was not only faster, but yielded good results. Multiple configurations of the simulation enviroment were used with respect to how ball motion would be performed and how our robot's proximity sensor would be read. We finally decided on using Python's Threading library to operate the proximity sensor and move the ball fluidly.

If we would decide to work on this project in the future, we could use a robot without a fixed base, and allow defensive strategies to include catching the ball and throwing it a random direction as opposed to just blocking the ball. We might also make the ball curve in the simulation and use a more sophisticated algorithm to predict where the ball might end up with greater accuracy.




