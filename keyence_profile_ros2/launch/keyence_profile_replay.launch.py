from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fixture_file'),
        DeclareLaunchArgument('profiles_topic', default_value='/profiles'),
        DeclareLaunchArgument('raw_topic', default_value='/profiles_float'),
        DeclareLaunchArgument('pitch_topic', default_value='/profiles_pitch_m'),
        DeclareLaunchArgument('loop', default_value='false'),
        Node(
            package='keyence_profile_ros2', executable='keyence_profile_replay',
            parameters=[{
                'fixture_file': LaunchConfiguration('fixture_file'),
                'profiles_topic': LaunchConfiguration('profiles_topic'),
                'raw_topic': LaunchConfiguration('raw_topic'),
                'pitch_topic': LaunchConfiguration('pitch_topic'),
                'loop': LaunchConfiguration('loop'),
            }],
        ),
    ])
