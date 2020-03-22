# ROS melodic
FROM osrf/ros:melodic-desktop-full-bionic

# Using bash instead of sh to be able to source
ENV TERM xterm

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update ROS packages Install Catkin and Moveit!

RUN apt-get update && \

    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-melodic-catkin python-catkin-tools ros-melodic-moveit wget

RUN mkdir -p ~/catkin_ws/src  && \
    cd ~/catkin_ws/  && \
    catkin init  

# Create project foulders
RUN cd /opt/ros/melodic/share/ && \
    mkdir simulations  && \
    cd /opt/ros/melodic/share/simulations

#RUN cd /opt/ros/melodic/share/my_workspace/scr

COPY ./robotic_arm_moveit_config ~/catkin_ws/src/
COPY ./object_recognition ~/catkin_ws/src/
COPY ./roboarm_pap ~/catkin_ws/src/

RUN cd ~/catkin_ws/src && \
    source /opt/ros/melodic/setup.bash && \
    catkin build && \
    echo "source /~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Clone robot model and coty sim files
RUN cd /opt/ros/melodic/share/simulations/ && \
    git clone https://github.com/ros-industrial/universal_robot.git
    
COPY ./robotic_arm /opt/ros/melodic/share/simulations/
COPY ./robotic_arm_gazebo /opt/ros/melodic/share/simulations/

# Install libraries to use Gazebo camera in ROS 
RUN apt-get install  -y ros-melodic-ros-control ros-melodic-ros-controllers
RUN apt-get install  -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Install other libraries 
RUN apt-get install -y python-scipy 
RUN apt-get install -y libcanberra-gtk3-module

# Setup entrypoint
#COPY ./entrypoint.sh /

#ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
