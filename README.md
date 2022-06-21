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

Thanks to the User interface node, the user can choose the modality, cancel the actual goal or exit from the interface itself.
The command selected by the user is then passed to the Navigation controller through a custom service called "interface service".
So the navigation controller takes as input the user command and implement a specific modality, in order to satisfy the user request.
The three main modalities are :

* Auto Drive Mode

* Manual Drive Mode

* Assisted Drive Mode

### Auto Drive Mode 

Firstly, to require the Auto drive mode the user has to insert, using the keyboard, the character "a".
Secondly, the user interface node asks to the user to insert the desired goal coordinates x and y.
Thirdly, the user interface node sends to the navigation controller, the user command through the custom service "interface" and sends the desired goal coordinates through the custom service "goal".
Then the navigation controller node through the interface and goal services, calls the callback functions interface and goalPosition.
The first one sets equal to true the boolean variable auto_mode (the others are false), this is useful to manage the remapping of the Teleop twist (see velCallback).
The second one moves the robot towards the goal position.

#### Optional

To move the robot towards the goal position, the navigation controller implement an action client that do a request to the movebase action server, so thanks to the movebase action server the robot moves torwards to the goal position.

The user has the possibility to cancel the goal. When the user insert the command "c" in the user interface, the navigation controller node cancel the goal using the move_base/cancel topic.
The goal is cancelled even when the timeout is elapsed and obviously when the robot reaches the goal position. 

![Automode_Flowchart](https://github.com/GabrieleRusso11/rt1_third_assignment/blob/main/Automode_Flowchart.png)


### Manual Drive Mode 

Firstly, to require the Auto drive mode the user has to insert, using the keyboard, the character "m".
Secondly, the user interface node sends to the navigation controller, the user command through the custom service "interface".
Then the navigation controller node through the interface service, calls the callback function interface that sets equal to true the boolean variable manual_mode (the others are false) in order to bypass the remapping of teleop twist keyboard and so now when the user use the teleop twist keyboard, it publishes the velocity on the cmd_vel topic.
Then the user can drive manually the robot using the teleop twist keyboard.

![Manualmode_Flowchart](https://github.com/GabrieleRusso11/rt1_third_assignment/blob/main/Manualmode_Flowchart.png)

### Assisted Drive Mode 

![Assistedmode_Flowchart](https://github.com/GabrieleRusso11/rt1_third_assignment/blob/main/Assistedmode_Flowchart.png)

### Final consideration