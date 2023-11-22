from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess

def generate_launch_description():
    msg_arg = DeclareLaunchArgument('message', default_value = TextSubstitution(text="Hai"))
    freq_arg = DeclareLaunchArgument('pub_freq', default_value = TextSubstitution(text="1000"))
    ros_bag_arg =  DeclareLaunchArgument('rosbag', default_value = TextSubstitution(text = "True"), choices = ['True', 'False'], description = "Enter True or False to record rosbag")
    
    publisher = Node(
            package='beginner_tutorials',
            executable='talker',
            parameters=[
                {"message" : LaunchConfiguration('message')},
                {"pub_freq" : LaunchConfiguration('pub_freq')}
            ]
    )

    subscriber = Node(
            package='beginner_tutorials',
            executable='listener'
    )
    
    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('rosbag')),
            cmd=['ros2', 'bag', 'record', '-a'],
        shell=True
    )
    
    return LaunchDescription([
        msg_arg,
        freq_arg,
        ros_bag_arg,
        publisher,
        subscriber,
        recorder
    ])