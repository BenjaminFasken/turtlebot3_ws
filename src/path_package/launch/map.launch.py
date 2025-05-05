import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directories
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_cartographer_pkg_dir = get_package_share_directory('turtlebot3_cartographer')
    path_package_pkg_dir = get_package_share_directory('path_package') # Assuming your package is named 'path_package'

    # Path to the launch files to include
    turtlebot3_world_launch_file = os.path.join(
        turtlebot3_gazebo_pkg_dir, 'launch', 'turtlebot3_world.launch.py'
    )
    cartographer_launch_file = os.path.join(
        path_package_pkg_dir, 'launch', 'cartographer.launch.py'
    )

    return LaunchDescription([
        # 1. Launch TurtleBot3 Gazebo World
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_world_launch_file),
            # You might need to add launch_arguments here if the world launch file requires them
        ),

        # 2. Launch Cartographer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_file),
            launch_arguments={'use_sim_time': 'True'}.items(),
        ),

        # 3. Run map_filter_node
        Node(
            package='path_package',
            executable='map_filter_node',
            name='map_filter_node',
            output='screen',
            # Add parameters if needed, e.g., parameters=[{'param_name': 'value'}]
        ),

        # 4. Run path_node
        Node(
            package='path_package',
            executable='path_node', # Assuming the executable is named 'path_node'
            name='path_node',
            output='screen',
            # Add parameters if needed
        ),
    ])