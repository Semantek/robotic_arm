# ROS melodic
FROM osrf/ros:melodic-desktop-full-bionic

# using bash instead of sh to be able to source
ENV TERM xterm
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update ROS packages
RUN rosdep update && \
    apt-get update && \
    apt-get dist-upgrade && \
    source /opt/ros/melodic/setup.bash && \
    echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc

# Install Catkin and Moveit!
RUN apt-get install ros-melodic-catkin python-catkin-tools
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/ && \
    catkin_make

RUN mkdir -p ~/ws_moveit/src && \
    cd ~/ws_moveit/src && \
    wstool init . && \
    wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && \
    wstool update -t . && \
    export ROS_PACKAGE_PATH=~/catkin_ws/src:~/ws_moveit/src:$ROS_PACKAGE_PATH

# Create project foulders
RUN cd /opt/ros/melodic/share/ && \
    mkdir simulations && \
    #mkdir my_workspace
    # cd /opt/ros/melodic/share/my_workspace/
COPY robotic_arm_moveit_config ~/catkin_ws/src
COPY object_recognition ~/catkin_ws/src
COPY roboarm_pap ~/catkin_ws/src
RUN cd ~/catkin_ws/ && \
    catkin_make

# Clone robot model
RUN cd /opt/ros/melodic/share/simulations/ && \
    git clone https://github.com/ros-industrial/universal_robot.git
COPY robotic_arm /opt/ros/melodic/share/simulations/
COPY robotic_arm_gazebo /opt/ros/melodic/share/simulations/

# Install libery openni kinect to use Gazebo camera in ROS 
RUN apt-get install ros-fuerte-openni-camera ros-fuerte-openni-launch

RUN apt-get install python-scipy && \
    apt-get install libcanberra-gtk3-module && \
    apt-get install ros*controller*

# Setup entrypoint
#COPY ./entrypoint.sh /

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]