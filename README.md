# Robotics Essentials ROS 2

Welcome to an open-source repository that contains materials and exercises needed 
to learn the essential robotics skills using ROS 2. 

This material is implemented for the 2024 iteration of the Master's level "Robotics & XR" course taught by 
[Dr. Ilkka Jormanainen](https://www.linkedin.com/in/ilkka-jormanainen-5954441/) at the [University of Eastern Finland](https://www.uef.fi/en), in collaboration with [Henki Robotics](https://henkirobotics.com/).

[<img src="images/uef_logo.jpg" alt="UEF Logo" width="200" height="190"/>](https://www.uef.fi/en)
[<img src="images/henki_robotics_logo.png" alt="Henki Robotics Logo" width="200" height="190"/>](https://henkirobotics.com/)

## Course overview

This course is designed to give you hands-on experience with the basics of robotics using ROS 2 and Gazebo simulation. 
The exercises focus on the [Andino robot](https://github.com/Ekumen-OS/andino_gz/tree/humble) from Ekumen and are structured to gradually introduce you to ROS 2 and Docker.


No prior experience in ROS 2 or Docker is needed, and since everything runs through Docker, you won’t need to install ROS 2 beforehand. 
Along the way, you’ll learn important concepts like autonomous navigation and mapping for mobile robots.
All coding exercises are done in Python.

<img src="images/autonomous_navigation.gif" alt="drawing" width="1200"/>

## Exercises

This repository contains a set of exercises to learn the concepts through practical demos and coding exercises.

0. [Setup](0-setup)
    - Setup the exercises
    - Run the Gazebo simulation 
    - Learn to use Docker
1. [ROS 2 Introduction](1-ros_2_introduction)
    - ROS 2 introduction
    - Gazebo and Rviz
    - ROS 2 topics; publish and subscribe
    - Transformations and tf-frames
2. [SLAM and Navigation Demo](2-slam_and_navigation_demo)
    - Create a map using slam-toolbox
    - Navigate autonomously using Nav2
    - ROS 2 services
3. [Create your first ROS 2 package](3-create_ros_2_package)
    - ROS 2 packages, how to create your own
    - Building and sourcing
    - ROS 2 Nodes
4. [Robot Odometry](4-robot_odometry)
    - Calculate and publish your robot's odometry using wheel velocities
    - Robot odometry and how to calculate it
    - Publish and subscribe to topics from Python code
5. [Path Planning](5-path_planning)
    - Basic navigation concepts
    - Modify Nav2 parameters
    - Custom path planning using Nav2

## Lecture slides

These slides accompany the practical exercises included in this repo, and are meant to be covered at the same time.

1. [Theme 1: Robotics in society](lecture_slides/theme_1_robotics_in_society)
2. [Theme 2: Robotics applications](lecture_slides/theme_2_robotics_applications)
3. [Theme 3: Robot control theory](lecture_slides/theme_3_robot_control_theory)
4. [Theme 4: Navigation](lecture_slides/theme_4_navigation)
5. [Theme 5: Robotics & AI](lecture_slides/theme_5_robotics_ai)
6. [Theme 6: XR applications in Robotics](lecture_slides/theme_6_xr_applications_robotics)


## Contribute
Did you encounter a problem during your exercises, or did you find a mistake?
Report issues in the "Issues" section in GitHub, and we will help you.

If you've found a fix to an existing issue, you can open a new Pull Request to fix it and contribute to this course.
We highly appreciate all the contributions!