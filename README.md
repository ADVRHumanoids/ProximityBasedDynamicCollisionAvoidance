# ➤ ROS2 Packag For Dynamic Collision Avoidance Based On OnBoard ProximitySensors For Human-Robot Close Interactions

## Introduction
In human-robot shared environments, onboard proximity sensors have been recognized 
as a promising tool for monitoring the surrounding robot workspace.
Unexpected collisions can then be spoiled and online evaded by continuously correcting 
the robot's actual trajectory while maintaining a safe human-robot coexistence 
enabling in this way human-robot closed interactions. 
For this purpose, the present ROS2 package for online dynamical collision avoidance and recovering
based on onboard sensors has been developed. 

## Outline
* [Introduction](#introduction)
* [Outline](#outline)
* [What It Does](#what-it-does)
* [How to Use It](#how-to-use-it)
* [How to Initialize It](#how-to-initialize-it)
* [Installation](#installation)
* [Execution](#execution)
* [Ros2 Versions](#ros2-version)
* [Dependences](#dependences)
* [Maintaner](#maintaner)

## What it does

Here, a brief explanation of what the package does is given. 

As mentioned in the introduction, this package allows you to avoid dinamic collisions. 
To do that, the package computes *local changes* of a preplanned robot trajectory, in the Cartesian space. To compute such a local changes the package uses the measurements retrieved by the *onboard* sensors, in particula it 
requires the measured distances. 
Based on that, the result of the package is a local replanning of the robot preplanned trajectory. 
The avoidance of a dynamic collision performed thanks to the onboard sensors is triggered by a threshold value on the distances measurements. 
In particular, if at least one distance measurement has a value less than the threshold value, then the algorithm to avoid the collision is activated computing the replanning of the preplanned robot 
trajectory. Otherwise, no changes of the pre-planned robot trajectory will be computed and the package will provide the same trajectory as the preplanned one.

## How to Use It

Keeping in mind what the package does, then, the package requires some inputs and initial settings to do to properly works.

*As inputs*, the package *online* requires the robot preplanned trajectory and distance measurements. 
These quantities are supposed to be provided to the package by publishing messages on two different topics. The messages has to be in the form of 'MotionMsg.msg' 
as [here](https://github.com/ADVRHumanoids/ProximityBasedDynamicCollisionAvoidance/blob/master/msg/MotionMsg.msg) and 'SensorsMsg.msg' 
as [here](https://github.com/ADVRHumanoids/ProximityBasedDynamicCollisionAvoidance/blob/master/msg/SensorsMsg.msg). 

*As output*, at each instant of time, the package will provide a replanned robot trajectory to command to the robot by publishing the related message 'RobotMsg.msg' in the form as [here](https://github.com/ADVRHumanoids/ProximityBasedDynamicCollisionAvoidance/blob/master/msg/SensorsMsg.msg). 


The replanning of the preplanned robot trajectory operated by the package is based on onboard sensors measurements, therefore, the package additionally requires to 
know the location of the sensors on the robot body as well. For that, the package assumes a frame associated to each sensor present in the system. 
Then, the package requires these frames are present in the URDF of the robot model (you can do that by fakely adding fixed joints :)).

Additionally to the above, some initializations are required to be done. They are detailed in the next section.

Note: if the preplanned robot trajectory is not provided, the package anyhow starts by knowing the initial configuration of the robot given as parameter. Based on that, if you send to the package 
messages regarding the distances measurements, the package works! In particular, if a potential collision is detected, then the package will provide local changes of the current robot pose, which turns 
to move the robot to avoid the coming collision :)! On the other hand, if no messages are provided to the package, the package will anyway publish a message but its content its just the current pose of the 
robot.

## How to Initialize It

As we have seen, to use the package, it is necessary to

1. Have a URDF file of the robot.
2. Send a message to provide the robot preplanned trajectory.
3. Send a message to provide the measured distances.
4. Retrieve the result, replanned trajectory by listening to a topic.

But, to make everything effectively working, we need to properly initialize the package before using it. 

In particular, it is required to set these quantities when you want to start the package:

```yaml
/robot/dyn_coll_avoid_settings:
  ros__parameters:
    robot_initial_config: [0.0,-1.56,0.9,0.2,-0.5,1.12]
    robot_urdf_model_path: "/home/liana/ros2_ws/src/ROS2UtilityNodes/urdf/inail2arm.urdf"
    robot_base_frame_name: "base_link"
    robot_tip_frame_name: "arm1_6"
    topic_motion_subscriber_name: "/robot/motion_planning"
    topic_sensors_subscriber_name: "/robot/sensors_data"
    topic_robot_publisher_name: "replanning"
    sensors_frame_name: ["teraflex_1_sensor1_link","teraflex_1_sensor2_link","teraflex_1_sensor3_link","teraflex_1_sensor4_link","teraflex_1_sensor5_link","teraflex_1_sensor6_link"]
    distance_threshold: 0.20
    correction_time: 0.4
    n_sensors: 6
    rate: 5
    log_path: "/tmp/replanner"
```
in the file `dynamic_collision_avoidance_settings.yaml` related to the node inside the folder [config](https://github.com/ADVRHumanoids/ProximityBasedDynamicCollisionAvoidance/tree/master/config) of the package.

Explaination:

1. robot_initial_config : initial robot joint configuration. This is the initial robot configuration with which you start with the robot. It is used in the package in order to retrieve the initial pose of the tip frame.
2. robot_urdf_model_path : absolute path of the URDf file.
3. robot_base_frame_name : frame name of the base frame as named in the URDF file.
4. robot_tip_frame_name : frame name of the tip frame as named in the URDF file.
5. topic_motion_subscriber_name : topic name of where to subscribe the robot trajectory.
6. topic_sensors_subscriber_name : topic name of where to subscribe the measured distances.
7. topic_robot_publisher_name : topic name of publisher of the node.
8. sensors_frame_name : frame names of the sensors as present and named in the URDF file.
9. distance_threshold : threshold for the distance within which the avoidance is triggered (in meters).
10. correction_time : time within with to evade the potential collision (in seconds).
11. n_sensors : number of sensors.
12. rate : node rate.
13. log_path : absolute path of the folder where to log the data.

The values here are just an example. You have to insert the ones regarding your case. Please, do not touch any parameter name, just modify the fields :).

## Installation
**To Install** the dynamic collision avoidance package run
```console
git clone https://github.com/ADVRHumanoids/---.git
```
inside the _**src folder**_ of the workspace and then run
```console
colcon build
```
in the main folder of the ros2 workspace

## Execution
To execute the code, just run:
```console
ros2 launch dyn_collision_avoid dynamic_collision_avoidance.launch.py
```

Remember to source `install/local_setup.sh`, by simply
```console
source install/local_setup.sh
```

You should get a similar output

```console
[INFO] [launch]: All log files can be found below /home/liana/.ros/log/2024-06-01-21-45-04-278346-liana-Inspiron-5593-1155608
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [DynamicCollisionAvoidance-1]: process started with pid [1155611]
[DynamicCollisionAvoidance-1] [INFO] [1717271104.387412824] [robot.dyn_coll_avoid_settings]: I am initializing the ROS node...
[DynamicCollisionAvoidance-1] 
[DynamicCollisionAvoidance-1] [INFO] [1717271104.390868768] [robot.dyn_coll_avoid_settings]: I am loading the ROS node params...
[DynamicCollisionAvoidance-1] 
[DynamicCollisionAvoidance-1] frame names : teraflex_1_sensor1_link
[DynamicCollisionAvoidance-1] frame names : teraflex_1_sensor2_link
[DynamicCollisionAvoidance-1] frame names : teraflex_1_sensor3_link
[DynamicCollisionAvoidance-1] frame names : teraflex_1_sensor4_link
[DynamicCollisionAvoidance-1] frame names : teraflex_1_sensor5_link
[DynamicCollisionAvoidance-1] frame names : teraflex_1_sensor6_link
[DynamicCollisionAvoidance-1] q0 :     0 -1.56   0.9   0.2  -0.5  1.12
[DynamicCollisionAvoidance-1] p0 :   -0.579894 -0.00190494  -0.0216102     1.47078   -0.187644    -2.17913
[DynamicCollisionAvoidance-1] Worked for 0.00 sec (0.0 MB flushed)....average load is -nan 
[DynamicCollisionAvoidance-1] Created variable 'TargetPoseReceived' (20 blocks, 500 elem each)
[DynamicCollisionAvoidance-1] Created variable 'TargetPoseCorrectionTerm' (20 blocks, 500 elem each)
[DynamicCollisionAvoidance-1] Created variable 'TargetPoseFinal' (20 blocks, 500 elem each)
[DynamicCollisionAvoidance-1] Created variable 'JointsPositionReference' (20 blocks, 500 elem each)
```

## Ros2 Version
The package has been tested on

- Humble Hawksbill (Debian packages [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- Jazzy Jalisco (source [link](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html))

Both on Ubuntu 22.04.

## Dependences
The Robot Impedance Modulation requires the following dependencies:
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page): to handle with basic algebra;
* [MatLogger2](https://github.com/ADVRHumanoids/MatLogger2): to log data;
* [KDL](https://www.orocos.org/kdl.html): to handle robot quantities.

## Maintaner

|<img src="https://avatars0.githubusercontent.com/u/15608027?s=400&u=aa95697b36504a10aeff4bf95d5d2f355ae94f07&v=4" width="180">|
|:-------------:|
|Liana Bertoni|
|liana.bertoni at iit.it|


