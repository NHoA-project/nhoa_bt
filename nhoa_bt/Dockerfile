FROM gitlab.local.eurecat.org:5050/robotics-automation/robotics-dockers/ros_dev:noetic

# Root permissions necessary.
USER root

# Prepare ROS workspace.
RUN mkdir -p /nhoa_bt_ws
WORKDIR /nhoa_bt_ws
   
RUN apt-get update && \
    apt-get install -y git && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libzmq3-dev \
    libboost-dev \
    ros-noetic-behaviortree-cpp-v3 \
    ros-noetic-control-msgs \
    ros-noetic-controller-manager \
    ros-noetic-controller-manager-msgs \
    ros-noetic-hri-msgs \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-move-base-msgs 
    # ros-noetic-hri \
    # ros-noetic-pyhri \
    # ros-noetic-human-description \
    # ros-noetic-hri-rviz \
    # ros-noetic-hri-face-detect \
    # ros-noetic-hri-full-body \
    # ros-noetic-hri-usb-cam \

# Clone necessary packages.
WORKDIR /nhoa_bt_ws/src

# CLone github repos.
RUN git clone https://github.com/NHoA-project/nhoa_bt.git ./nhoa
# RUN git clone https://github.com/nhoa-project/nhoa_approach_action.git ./nhoa_approach_action
# RUN git clone https://github.com/nhoa-project/hri_face_detect.git ./hri_face_detect
WORKDIR /nhoa_bt_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build  -DCATKIN_ENABLE_TESTING=0 -DCMAKE_CXX_STANDARD=11"


# Add source to bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /nhoa_bt_ws/devel/setup.bash" >> ~/.bashrc