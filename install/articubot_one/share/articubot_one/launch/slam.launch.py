import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME


    gazebo_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_sim.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam_node=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
                )]), launch_arguments={'slam_params_file':'./src/articubot_one/config/mapper_params_online_async.yaml','use_sim_time': 'true'}.items()
    )


    rviz_slam=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','rviz_launch.py'
                )])
    )

    return LaunchDescription([
        gazebo_sim,
        slam_node,
        rviz_slam
        

    ])