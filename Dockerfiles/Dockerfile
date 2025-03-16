FROM osrf/ros:humble-desktop-full-jammy

# set the environment variable with the command ENV <key>=<value>, it can be replaced online
ENV DEBIAN_FRONTEND=noninteractive
# RUN is used to execute and add new layer on top of the base immage
RUN apt-get update
RUN apt-get update --fix-missing
RUN apt-get upgrade -y 
RUN apt-get install -y ros-humble-plotjuggler

RUN apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    chrony \
    tmux python3-pip\
    xterm \
    libeigen3-dev \
    nano \
    ros-humble-rviz2 \
    nautilus \ 
    iputils-ping \
    iproute2  \
    python3-rosdep \
    && apt-get clean
RUN apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    && apt-get clean
# # install realsense library 
RUN apt-get install -y ros-humble-realsense2-* \
    ros-humble-generate-parameter-library \
    && apt-get clean
RUN sudo apt-get install -y  rapidjson-dev
RUN apt-get update
RUN apt-get upgrade -y

 

ENV DEBIAN_FRONTEND=dialog

# Create a new user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi
RUN groupadd --gid ${USER_GID} ${USERNAME} 
RUN useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo git \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

#Change HOME environment variable
ENV HOME=/home/${USERNAME}
# Choose to run as user
ENV USER=${USERNAME}

USER ${USERNAME}

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Install the python packages cosi non vengono installati da root
RUN pip3 install numpy 
RUN pip3 install opencv-contrib-python transforms3d
# install gazebo ignition
# RUN sudo apt-get install -y  curl lsb-release gnupg
# RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpgecho "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN echo " CURL DONE"



# Set up auto-source of workspace for ros user
ARG WORKSPACE=THE_vision_ws
WORKDIR /home/ros/${WORKSPACE}
RUN sudo usermod -a -G render,video ${USERNAME}
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN echo "if [ -f ~/${WORKSPACE}/install/setup.bash ]; then source ~/${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/" >> ~/.bashrc
RUN sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

RUN sudo apt-get update
RUN sudo apt-get -y upgrade

# rosdep realsense library 
RUN sudo apt-get install python3-rosdep -y
RUN sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN sudo rosdep init
RUN rosdep update 
RUN echo "rosdep install -i --from-path /home/ros/${WORKSPACE}/src --rosdistro humble --skip-keys=librealsense2 -y" >> /home/ros/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
# RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# RUN echo "source /home/ros/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc