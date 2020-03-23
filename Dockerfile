# ROS melodic

FROM osrf/ros:melodic-desktop-full-bionic

# Using bash instead of sh to be able to source
ENV TERM xterm

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update ROS packages Install Catkin and Moveit!
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-melodic-catkin python-catkin-tools ros-melodic-moveit wget
    
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN mkdir -p ~/catkin_ws/src  && \
    cd ~/catkin_ws/  && \
    source /opt/ros/melodic/setup.bash  && \
    catkin init

# Create project foulders
RUN cd /opt/ros/melodic/share/ && \
    mkdir simulations  && \
    cd /opt/ros/melodic/share/simulations && \
    mkdir robotic_arm_gazebo  && \
    mkdir robotic_arm 

RUN cd ~/catkin_ws/src && \
    mkdir robotic_arm_moveit_config && \
    mkdir object_recognition && \
    mkdir roboarm_pap

COPY ./robotic_arm_moveit_config /catkin_ws/src/robotic_arm_moveit_config
COPY ./object_recognition/ /catkin_ws/src/object_recognition/
COPY ./roboarm_pap/ /catkin_ws/src/roboarm_pap/

RUN chmod +x /catkin_ws/src/object_recognition/src/object_recognition_start.py     
RUN chmod +x /catkin_ws/src/roboarm_pap/src/start_work.py

RUN cd ~/catkin_ws && \
    source /opt/ros/melodic/setup.bash && \
    catkin build && \
    echo 'source /catkin_ws/devel/setup.bash' >> /.bashrc    

# Clone robot model and coty sim files
RUN cd /opt/ros/melodic/share/simulations/ && \
    git clone https://github.com/ros-industrial/universal_robot.git
  
COPY ./robotic_arm /opt/ros/melodic/share/simulations/robotic_arm
COPY ./robotic_arm_gazebo /opt/ros/melodic/share/simulations/robotic_arm_gazebo

# Install libraries to use Gazebo camera in ROS 
RUN apt-get install  -y ros-melodic-ros-control ros-melodic-ros-controllers
RUN apt-get install  -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Install other libraries 
RUN apt-get install -y python-scipy 
RUN apt-get install -y libcanberra-gtk3-module

CMD ["bash"]
