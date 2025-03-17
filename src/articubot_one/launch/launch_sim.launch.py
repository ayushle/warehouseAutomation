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

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    world_path = "/home/ayush/Documents/ros_ws/src/articubot_one/worlds/warehouse.world"
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
                    ,launch_arguments={'world': world_path}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

     # Start controller_manager
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','joint_state_broadcaster'],

        output='screen'
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','joint_trajectory_controller'],
        output='screen'
    )
    joint_publisher_launcher = ExecuteProcess(
        cmd=['ros2','run','articubot_one','joint_publisher'],
        output='screen'
    )
    
    
    # joint_publisher = Node(
    #     package='articubot_one',
    #     executable='joint_publisher',
    #     name='joint_publisher',
    #     output='screen'
    # )

    
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        joint_publisher_launcher

    ])