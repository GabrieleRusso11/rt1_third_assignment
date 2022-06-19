# Gabriele Russo's third assignment for the Research Track 1 course (Mat. 5180813)

## Installation and how to run

Firstly, you have to clone this repository into your ROS workspace's src/ folder :

```
https://github.com/GabrieleRusso11/rt1_third_assignment.git
```

Secondly, you have to clone also the final_assignment package, the slam_gmapping 
package and download the ROS navigation stack :

```
git clone https://www.github.com/CarmineD8/final_assignment
git clone https://www.github.com/CarmineD8/slam_gmapping
sudo apt-get install ros-noetic-navigation
```

Lastly, Write on the shell the following command to execute all the project :

```
roslaunch rt1_third_assignment assignment3.launch
```

## Introduction

The aim of this project is to develop a software architecture for the control
of the robot in the environment. The software will rely on the move_base
and gmapping packages for localizing the robot and plan the motion.
The architecture should be able to get the user request, and let the robot 
execute one of the following behaviors (depending on the user’s input):

* Auto Drive Mode : autonomously reach a x,y coordinate inserted by the user

* Manual Drive Mode : let the user drive the robot with the keyboard

* Assisted Drive Mode : let the user drive the robot assisting them to avoid collisions

![Environment](https://github.com/GabrieleRusso11/rt1_third_assignment/blob/main/Environment.png)

## How It works

For the implementation of this project is used ROS (Robot Operating System). ROS works through nodes that communicate with each other using Topic (which is messages transport system that implement the publish/subscribe model) and Service (which implement the request/response model).

In this project there are two nodes :

* The navigation controller node : which is the node that implements all the drive modalites of the robot (and more)

* The User interface node : which is the link between the user and the navigation controller node

### Auto Drive Mode 

![Automode_Flowchart](https://github.com/GabrieleRusso11/rt1_third_assignment/blob/main/Automode_Flowchart.png)

### Manual Drive Mode 

![Manualmode_Flowchart](https://github.com/GabrieleRusso11/rt1_third_assignment/blob/main/Manualmode_Flowchart.png)