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

The main purpose of this behavior is to drive forward, while maintaining a constant distance to a wall. We made some critical assumptions whilst designign this behavior to make it simpler and easier to implement. The biggest assumption we made was that the wall would be on the left side of the robot. Since we knew this behavior would directly follow the circle behavior we were able to make this assumption. The neato goes in a clockwise circle, so the neato is almost assuredly on the left side of the wall. Another assumption this algorithm makes is that any lidar data on the left side of the neato is a wall. Again we are able to make this assumption because of the previous circle behavior. Also since we decided to do this in just sim, we decide the robot enviroment.

From a high level, this code uses proportional control to minimize distance from the wall, and try to make the angle of the wall as close to parallel as possible. The distance error is based off of the lidar data from a point directly to the left (90 degrees) of the neato. The angle of the wall is more nuanced. We first pick the same left point from the neato. We then pick a point slightly in front of that point, we decided to make this 45 degrees from the neato (With 0 as the front). Then we calculate the angle of the line connecting those 2 points, using atan2(). This tells us the angle of the wall, relative to vertical. Since we want to be parallel to the wall, the angle error is just the angle of the wall, since we ideally want to be a 0. 

We used a simple proportinal controller on rotation of the robot. We don't want to change the linear speed of the robot, since we want to follow the wall at a constant speed, so we do all adjustment using the rotation of the robot. There are 2 components to this: the error in the distance from the wall, and the error from the angle of the wall. We use a naive way to combine these components, just by adding up these errors and using 2 different P values. This ended up working really well for us, but in the future we would want to explore more nuanced ways of combining these components. 


#### Person Following:

The main purpose of this behavior was to follow an object near it. We were able to make assumptions about the enviroment of the neato to make this behavior simpler to implement. The biggest assumption we made was assuming that the closest object to the neato was what we want it to follow. 

From a high level, this code uses proportional control to stay a certain distance away from the closest point, and a different proportional controller to face the object. Having two different proportional controllers that control the robot's position isn't best practice. The distance controller controls the linear positioning of the robot, while the angle controller controls the angular positioning, but combining them leads to unexpected behavior. For the future, we would want a more nuanced controller that determines the error, both angular and linear, from the object and uses one proportional controller to correct for that error.

#### Estop:



#### Finite State machine
For the finite state controller, what was the overall behavior? What were the states? What did the robot do in each state? How did you combine behaviors together and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.



Conclusion:

How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
What, if any, challenges did you face along the way?
What would you do to improve your project if you had more time?
What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.