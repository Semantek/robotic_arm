# Pick and place robotic arm project

## Introdution

This is the test project to try features of ROS, MoveIt! and OpenCV. The main objectives of the project are simulating the process of collection from the table surface of several randomly located objects using a robotic arm and place objects to the bin. 

The project is used a u10 robot from [here](https://github.com/ros-industrial/universal_robot), and the model of a three-finger grabber from [here](https://github.com/shadow-robot/smart_grasping_sandbox). 

A [Kinect](https://wiki.ros.org/openni_camera) camera model was used to recognize objects and determine their position in the world.

## Getting started

To get started you mast using Docker. So, if you haven't done it already, head over to the [Docker](https://www.docker.com/) website and follow the instructions over there.

### First run
To start this project run the following command to download Docker image.

    docker run -it \
	    --name ra\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        semantek/robotic_arm

### Start next time
Do not use "**run**" next time, but use "**start**" command. To do this, follow the next steps.
To make access from container to graphical tools run the following command on your host:

    xhost +local:root

Then start the container

    docker start ra

And come in the container

    docker exec -it ra bash


## Gazebo simulation

To start simulation with recognition of the objects on the table run next command in container:

Default objects on the table are randomly placed void coke cans. To change objects to the balls use the argument  "**ball**".

    roslaunch object_recognition main.launch object := ball



If you don't need a camera view window, use the argument  "**no_view**".

    roslaunch object_recognition main.launch no_view:=true


Camera in the simulation have "*in_static*" property, so you can move it in place as you like. 

The process of object recognition published number and places in the "**recognized_object_array**" topic.

## Start pick and place

To start pick and place run next command

    roslaunch roboarn_pap pap.launch

Robotic arm mast starts moving, grasping objects on the table and throw them to the bin one-by-one. Between loops arm will be stand in a neutral position.
If all will be OK, process shut down with "**No objects**" message.
