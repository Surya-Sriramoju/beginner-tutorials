from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    msg_arg = DeclareLaunchArgument('message', default_value = TextSubstitution(text="Hai"))
    freq_arg = DeclareLaunchArgument('pub_freq', default_value = TextSubstitution(text="1000"))
    
    return LaunchDescription([
        msg_arg,
        freq_arg,
        Node(
            package='beginner_tutorials',
            executable='talker',
            parameters=[
                {"message" : LaunchConfiguration('message')},
                {"pub_freq" : LaunchConfiguration('pub_freq')}
            ]
        ),
        Node(
            package='beginner_tutorials',
            executable='listener'
        )
    ])