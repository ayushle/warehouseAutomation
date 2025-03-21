import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
        
    navigation_node=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'
                )]), launch_arguments={'params_file':'./src/articubot_one/config/nav2_params.yaml','use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        navigation_node
        

    ])    
