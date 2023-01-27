# all_in_one
This repository contains the results of work presented in [All-in-One: A DRL-based Control Switch Combining State-of-the-art Navigation Planners](https://arxiv.org/pdf/2109.11636.pdf). The All-In-One Planner (AIO) chooses from a given list of local planners in each iteration with the objective to combine the strengths of learning-based planners and classical planners like TEB / MPC.   

<p align="center">
 <img width="800" height="400" src="https://github.com/ignc-research/all-in-one-DRL-planner/blob/main/videos/git/9b1a44e50f1d82027578431aaa09df6425371531.gif"> 
<p>

Currently we offer a control policy for 4 different robot models:
- Turtlebot3 Burger
- Jackal
- Robotino
- Youbot  

# Installation
1. Add this package to your .rosinstall or clone it manually.
```bash
cd ~/catkin_ws/src/arena-bench # Navigate to your arena-bench location
echo "- git:
    local-name: ../planners/all_in_one
    uri: https://github.com/Arena-Rosnav/all_in_one
    version: master" >> .rosinstall
rosws update ../planners/all_in_one # or rosws update
```
2. Run catkin make
```bash
cd ~/catkin_ws # Navigate to your catkin workspace
catkin_make
```
This code is intended for usage with Arena-Rosnav infrastructure.
For more information regarding AIO please refer to the [original repository](https://github.com/ignc-research/all-in-one-DRL-planner).


