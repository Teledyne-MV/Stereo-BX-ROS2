from launch import LaunchDescription
from launch_ros.actions import Node
   
def generate_launch_description():
    return LaunchDescription([
        # Launch RViz2 with a predefined config and debug log level
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',  # Output log messages to the screen
            arguments=[
                '-d', './src/stereo_image_publisher/launch/cfg.rviz',
                # '--ros-args', 
                # '--log-level', 'rviz2:=debug'  # Set log level to debug for RViz2
            ],
            remappings=[
                # Remap the topic directly in RViz2
                ('/image', '/Bumblebee_X/rectified_left_image')  # Remap the image topic
            ],
            parameters=[
                {
                    'use_sim_time': True,  # Optional: set if using simulation time
                }
            ],
        ),
    ])
    
    