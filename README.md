# THE_vision_ros2

ROS2 Workspace to integrate the plant pose recognition vision module with the UR control module.
The module uses the realsense D455 or D435 stereo camera to identify the plant into the image plane and then use the camera intrinsic parameter to obtain the plant's cartesian pose in camera frame.
The relationship between the UR base frame and the camera frame is get using a calibration procedure using two arco marker placed on the manipulator gripper. 
The repository provide a docker compose to simplify the framework deploy.

## Prerequisites 

[Docker Engine instalation](https://docs.docker.com/engine/install/ubuntu/)

[Docker Compose  Installation](https://docs.docker.com/compose/install/linux/#install-the-plugin-manually)

In order to use the realsense has to be install a kernel module on the host. The installation step can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages).
If the user did not want to use the docker compose engine and you have installed ROS2 Humble on your host, then follow the [realsese ros2 wrapper](https://github.com/IntelRealSense/realsense-ros) installation procedure.

## Docker Compose Set up 

In order to start the docker compose you have to build the container and then start the container using the docker compose cli:

``` docker compose build```  can be used to build the container

``` docker compose up``` can be used to start the container 

```docker compose exec  ros2_humble_the_vision bash``` can be used to open a shell into the container

## Start the Aruco Tracker 

in order to start the aruco trakcer you have to start the camera node using ``` ros2 launch realsense_camera rs_launch.py```, then to start the tracker you have to use ```ros2 launch ros2_aruco aruco_recognition.launch.py ```

## TODO 
-> Set Up the UR control and planning framework

-> Develop the calibration Node 

-> pick and place of dummy plants