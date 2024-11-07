FROM althack/ros2:humble-dev AS nav

###########################################
#  Navigation 2
###########################################

ENV DEBIAN_FRONTEND=noninteractive
# Install addition ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \ 
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-plotjuggler-ros \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc

RUN echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${WORKSPACE}/install/robot_utils/share/robot_utils/models" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${WORKSPACE}/src/hunav_gazebo_wrapper/media" >> /home/ros/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /home/ros/.bashrc

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros/.bashrc
RUN echo "export ROS_DOMAIN_ID=5" >> /home/ros/.bashrc
RUN echo "export CYCLONEDDS_URI=${WORKSPACE}/cyclonedds.xml" >> /home/ros/.bashrc