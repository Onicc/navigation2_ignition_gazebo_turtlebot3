
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    sdf = os.path.join(
        get_package_share_directory('turtlebot3'),
        'models', 'turtlebot3', 'model.sdf')

    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]),
    ])
