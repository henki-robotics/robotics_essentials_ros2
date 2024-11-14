# Exercises 1 - ROS 2 introduction

In the first set of exercises, we will showcase a demo of running a simulated robot, and control it using ROS 2.
You will also learn the very basics of ROS 2 topics and transforms.

<!-- TOC -->
* [Basic Concepts](#basic-concepts)
  * [ROS 2](#ros-2)
  * [Gazebo](#gazebo)
  * [Rviz](#rviz)
  * [Andino](#andino)
* [Launching the Andino robot in a Gazebo simulation](#launching-the-andino-robot-in-a-gazebo-simulation)
  * [Control the robot in Gazebo](#control-the-robot-in-gazebo)
* [ROS 2 Topics](#ros-2-topics)
  * [Subscribe to a topic](#subscribe-to-a-topic)
  * [Publish to a topic](#publish-to-a-topic)
* [RViz](#rviz-1)
  * [Subscribe to a new data source: Camera](#subscribe-to-a-new-data-source-camera)
* [TFs - The coordinate transforms](#tfs---the-coordinate-transforms)
  * [Commonly used coordinate frames](#commonly-used-coordinate-frames)
  * [TF frames in RViz](#tf-frames-in-rviz-)
    * [Changing the Fixed Frame](#changing-the-fixed-frame)
    * [Visualizing the TF-tree](#visualizing-the-tf-tree)
* [Summary](#summary)
<!-- TOC -->

<img src="/images/gazebo_rviz.png" alt="Andino Simulation Screenshot">


## Basic Concepts
### ROS 2
<img src="images/ros_logo_white_bg.svg" alt="ROS 2 logo" width="200">

[ROS 2 (Robot Operating System 2)](https://www.ros.org/) is an open-source framework for building robot software.
It provides a set of tools, libraries, and conventions, including a middleware for internal communication. 
It is designed to support real-time performance and multi-robot systems.


### Gazebo

<img src="images/gazebo_logo_white_bg.svg" alt="Gazebo logo" width="200">

[Gazebo](https://gazebosim.org/home) is a powerful open-source robotics simulator that allows developers to test and validate robot designs in complex environments, offering realistic physics and sensor models.

### Rviz

<img src="images/rviz_logo.png" alt="RViz logo" width="200">

[RViz](https://github.com/ros2/rviz) is a 3D visualization tool for ROS that enables developers to visualize sensor data, robot models, and environment maps, aiding in debugging and monitoring robot behavior.

### Andino

<img src="images/andino_robot.png" alt="Andino" width="200">

Andino is a fully open-source, educational low-cost robot developed by [Ekumen](https://github.com/Ekumen-OS/andino).
It uses ROS 2 to implement its functionalities and has fully functional [Gazebo simulations](https://github.com/Ekumen-OS/andino_gz) of it available.

## Launching the Andino robot in a Gazebo simulation

If you didn't yet, follow the instructions in [Exercises 0 - Setup](/0-setup/README.md) to setup and launch an Andino Gazebo simulation with RViz.

Here is a quick summary of all the required steps for launching the simulation:

    cd robotics_essentials_ros2/docker/
    docker compose up -d
    docker exec -it robotics_essentials_ros2 bash
    ros2 launch andino_gz andino_gz.launch.py


### Control the robot in Gazebo

**Exercise 1:**
Open the teleop panel and give commands to move the robot around. Try out all the teleoperation menus, and experiment with all the ways in which you can control Andino.
<br> 

<img src="images/gazebo_teleop.png" alt="Gazebo teleoperation" width="800">

## ROS 2 Topics

ROS 2 topics are a core communication mechanism in ROS 2 that enable data exchange in a publish/subscribe model. 
Publishers send messages to a named topic, while subscribers listen to that topic to receive relevant data.

### Subscribe to a topic
By subscribing to a topic, you can read sensor data (lidar, camera) and other useful data (map, odometry) from your robot.

Open a new terminal inside the Docker container and run the following commands ([How to open terminal in Docker container](/0-setup/Docker%20Cheat%20Sheet.md)):

1. List all the available ROS 2 topics
    ```commandline
    ros2 topic list
    ```
   
    <img src="images/ros2_topic_list.png" alt="ROS 2 topic list" width="600">

1. Read the sensor data from the laser scanner (Press CTRL+C to stop after a while). 
    ```commandline
    ros2 topic echo /scan
    ```
   <img src="images/ros2_topic_echo.png" alt="ROS 2 topic echo" width="600">

1. Get more info about the /scan topic to learn the message type
    ```commandline
    ros2 topic info /scan
    ```
   <img src="images/ros2_topic_info.png" alt="ROS 2 topic info" width="600">

We see that there is one publisher for the `/scan` topic, which is the driver node of the simulated lidar on Andino. What node is then subscribing to this topic?

### Publish to a topic
1. Move the robot by publishing to cmd_vel topic

    ```commandline
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
    ```

1. Send a 0-velocity command to stop the robot

    ```commandline
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"
    ```
   
**Exercise 2:**

Publish a message to rotate the robot in its place. First, check what is the message type of the `/cmd_vel` topic using `ros2 topic info` command, and then check the possible message contents with `ros2 interface show <msg_type>`



<details>
    <summary>Solution:</summary>
    
- `ros2 topic info /cmd_vel` shows us that the message type is `geometry_msgs/msg/Twist`
- `ros2 interface show geometry_msgs/msg/Twist` shows us that we have the "Angular" `Vector3` field that has the `z` field available, to make the robot rotate.
- We can rotate the robot with a command:

    ```
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
    ```
</details>


## RViz

RViz is a useful visualization tool that allows us to display data from ROS 2 topics.
With these examples, you will learn how to do that.

### Subscribe to a new data source: Camera

Our robot is constantly publishing images from the simulated camera. Let's see how those images look like!

1. Click "Add" -button from the bottom left corner of RViz

1. Choose to create visualization "By topic"

1. Choose `Camera` under the `/image_raw` and Press ok.
    <br><br>
    <img src="images/rviz_add_camera.png" alt="RViz add camera source" width="350">

    **Tip:** Set overlay alpha to 1 to hide the artifacts on top of the image:
    
    <img src="images/rviz_add_camera_2.png" alt="RViz camera source" width="350">


## TFs - The coordinate transforms

In ROS 2, transforms are used to describe the spatial relationships between different coordinate frames in a robotic system.
You can think about TFs as the coordinate frames that sit in the most important locations in your robot and in the environment.
They sit at the center of the robot, at the center of sensors, at joints, and there are so the "Map" and "Odometry" frames.
They allow you to convert positions and orientations from one frame to another, and basically keep track of how each part of 
your robot moves in relation to the other parts of the robot. This is crucial for tasks like navigation, sensor fusion, and 
manipulation.

The main component for handling transforms in ROS 2 is the tf2 library. It provides:
* Coordinate Frames: Each sensor or part of a robot has its own coordinate frame (e.g., the robot's base, sensors, end effectors).
* Transformations: These include translations (movement along axes) and rotations (changes in orientation) between frames.

By using transforms, robots can effectively understand their position in the world and how their sensors and motors are located in relation to their body.

The relationship between these coordinate frames is determined with tf-tree.
It essentially tells with a tree-like structure what is the child-frame's position in relation to the parent frame.

### Commonly used coordinate frames

<img src="images/ros2_tf_frames.png" alt="ROS 2 common tf frames" width="500">

_Image source: [wiki.ros.org](https://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot)_

**map**

The map frame provides a global reference point for the robot's environment, allowing it to understand its position within a larger context. 
Typically, the coordinates in the map frame present the robot's coordinates on a 2D map.

**odom**

The odom frame represents the robot's position based on its odometry data. 
It tracks the robot's movement from its starting point, being subject to drift and inaccuracies.

**base_footprint**

The base_footprint frame is a 2D representation of the robot's footprint on the ground, typically used for planning and movement calculations without considering the robot's height.

**base_link**

The base_link frame represents the robot's main body and is used as a reference for other components, such as sensors and arms.

**laser_link**

The laser_link frame denotes the position of a laser sensor on the robot. 
It is essential for interpreting the data collected by the laser for tasks like mapping and obstacle detection, providing a reference for where the sensor is located in relation to other frames.


### TF frames in RViz 

When working with RViz, you will need to use the "Fixed Frame" to determine from which frame's perspective you are 
visualizing the data. 
This is an important feature to know about, as sometimes the data you are looking to visualize might not be available if you are visualizing a wrong frame. 

#### Changing the Fixed Frame

1. On Andino, the default frame is set to "base_footprint". 
This means that RViz coordinate origin (0, 0), is set to the robot's footprint.
Move the robot around with teleoperation using Gazebo. You can see that the robot is always located in the center of the grid that RViz visualizes.
    
    <img src="images/rviz_coordinate_frame.png" alt="RViz base_footprint frame" width="400">

1. Change the "Fixed Frame" from "Global Options" to "odom" to use odometry as the coordinate frame instead of the robot base_footprint frame.

    <img src="images/rviz_fixed_frame.png" alt="RViz Fixed Frame" width="400">

1. Drive the robot around. You will see the robot moving in relation to "odom" frame.
    
    <img src="images/rviz_odom_frame.png" alt="RViz odom frame" width="400">

#### Visualizing the TF-tree

Sometimes it might be useful to check the robot's tf-tree for debugging purposes. 
You can do it by opening the "Tree" option under the TF menu.

**Tip:** You might need to press reset-button on bottom left, for the odom-frame to be correctly on top of the tree. 
This is needed because the `odom` frame is the one that tracks the movement of the robot in the environment, so it makes
sense for it to be the main parent frame, so that we easily keep track of the movement of all the other frames as well.

<img src="images/tf_tree.png" alt="ROS 2 tf tree" width="300">


## Summary

By the end of these exercises, you have now learned
- What are ROS 2, Gazebo, and Rviz
- How to launch Andino simulation and control the robot from Gazebo
- What ROS 2 topics are
- How to publish to a topic
- How to subscribe to a topic
- What tf-frames are