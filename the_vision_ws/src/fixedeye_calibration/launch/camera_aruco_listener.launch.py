from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def  generate_launch_description():

    ld = LaunchDescription()

    sample_num = LaunchConfiguration("listened_sample")
    sleep_time = LaunchConfiguration("sleep_duration")

    ld.add_action(
        DeclareLaunchArgument(
            "listened_sample",
            default_value="5",
            description="Tf2 Sample listened to get mean and covariance estimation.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "sleep_duration",
            default_value="40",
            description="Milliseconds wait between tow different sample record",
        )
    )


    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"]
        )
    )

    aruco_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros2_aruco"), "/launch", "/aruco_recognition.launch.py"]
        )
    )


    listener_node = Node(
        package="fixedeye_calibration",
        executable="listener_node",
        output="log",
        parameters=[
                {"listen_sleep": sleep_time},
                {"sample_number": sample_num},
            ]
    )

    ld.add_action(realsense_camera)
    ld.add_action(aruco_tf_node)
    ld.add_action(listener_node)

    return ld