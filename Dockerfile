# ROS melodic

FROM osrf/ros:melodic-desktop-full-bionic



# Using bash instead of sh to be able to source

ENV TERM xterm

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update ROS packages Install Catkin and Moveit!
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-melodic-catkin python-catkin-tools ros-melodic-moveit wget && \
    mkdir -p /opt/ros/melodic/share/my_workspace/scr && \

    cd /opt/ros/melodic/share/my_workspace/ && \
    source /opt/ros/melodic/setup.bash && \
    echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc

RUN cd /opt/ros/melodic/share/my_workspace && \
    catkin init
    

# Create project foulders

RUN cd /opt/ros/melodic/share/ && \

    mkdir simulations 

#RUN cd /opt/ros/melodic/share/my_workspace/scr

COPY robotic_arm_moveit_config /opt/ros/melodic/share/my_workspace/scr

COPY object_recognition /opt/ros/melodic/share/my_workspace/scr

COPY roboarm_pap /opt/ros/melodic/share/my_workspace/scr

RUN cd /opt/ros/melodic/share/my_workspace/ && \
    catkin build && \
    echo "source /opt/ros/melodic/share/my_workspace/devel/setup.bash" >> ~/.bashrc



# Clone robot model and coty sim files

RUN cd /opt/ros/melodic/share/simulations/ && \

    git clone https://github.com/ros-industrial/universal_robot.git

COPY robotic_arm /opt/ros/melodic/share/simulations/
COPY robotic_arm_gazebo /opt/ros/melodic/share/simulations/



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
