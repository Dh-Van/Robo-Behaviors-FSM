# Introduction to Computational Robotics: Warmup Project
By: Zaraius Bilimoria, Dhvan Shah

Introduction/Overview: Neato
This warmup project is an introduction to working with the Neatos, a vacumming robot using ROS2, the robot operating system. Our goal is to control the Neato and take it through different states, a circle, wall following, turning and person following as well as an emergency stop. We implemented these in a scalable way which allows for modifications our goal was to control the Neato
add a picture of a neato here   

### How to run 
```ros2 run ...```


The write-up contains sufficient information to understand what was completed during the project, how to interact with the project elements, what each team member contributed, and shows evidence of what was learned.

For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any design decisions that had to be made to realize a successful implementation.

#### Circle:
We start our neato by driving in a circle with an increasing radius. This is our way to explore the environment. In order to do this we have a we have a subscriber of the current_state and a publisher of the commanded velocity of the neato. When our state machine gives us a signal that it wants to be in circle mode, by subscribing to the current_state and checking its value, the drive_circle_node will start publishing its cmd_vel. The drive_circle does the MATH-MATH-MATH


#### Wall Following:

#### Person Following:

#### Estop:



#### Finite State machine
For the finite state controller, what was the overall behavior? What were the states? What did the robot do in each state? How did you combine behaviors together and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.



Conclusion:

How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
What, if any, challenges did you face along the way?
What would you do to improve your project if you had more time?
What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.