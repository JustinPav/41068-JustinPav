import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the turtlebot3 model environment variable
    turtlebot3_model = os.getenv('TURTLEBOT3_MODEL', 'waffle_pi')

    return LaunchDescription([
        # Set TURTLEBOT3_MODEL environment variable
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=turtlebot3_model),
        
        # Launch turtlebot3 in Gazebo with the warehouse environment
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_warehouse.launch.py'],
            output='screen'
        ),
        
        # Launch navigation2 with the specified map
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 
                 'map:=' + os.path.join(os.getenv('HOME'), 'Maps', 'map.yaml')],
            output='screen'
        ),
        
        # Run your custom ROS 2 node (Sprint4 circle_the_circle)
        ExecuteProcess(
            cmd=['ros2', 'run', 'Sprint4', 'circle_the_circle'],
            output='screen'
        ),
    ])
