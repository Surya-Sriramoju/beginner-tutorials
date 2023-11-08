# beginner-tutorials

## Dependencies
- Ros 2 Humble
- Basic understanding of ROS 2


## Instructions to run
1) Copy the package "beginner_tutorials" to the ros2 workspace in src
2) Source the ros workspace "source /opt/ros/<distro>/setup.bash", "source ~/ros2_ws/install/setup.bash" 
3) Inside ros2 workspace, build the package using colcon build
4) In the other terminal execute "ros2 run beginner_tutorials talker" which starts publishing my custom message
5) In the other terminal execute "ros2 run beginner_tutorials listener" which starts subscribing to my custom message