# beginner-tutorials

## Dependencies
- Ros 2 Humble
- Basic understanding of ROS 2


## Instructions to build and run
```bash
# clone the package inside source
  cd ros2_ws/src
  git clone https://github.com/Surya-Sriramoju/beginner-tutorials.git
# fix dependency issues
  rosdep install -i --from-path src --rosdistro humble -y
# build the code
  cd ~/ros2_ws
  colcon build --packages-select beginner_tutorials
# Terminal 1
  ros2 run beginner_tutorials listener
# Terminal 2
  ros2 run beginner_tutorials talker

```

## Sending a service request
```bash
# Terminal 3
  ros2 service call /change_string beginner_tutorials/srv/ChangeString "{after: <add a string here>}"

```

## Launching the launch file
```bash
  ros2 launch beginner_tutorials launch.py
# with params
  ros2 launch beginner_tutorials launch.py message:=<your message> pub_freq:=<desired_message_frequency>
```
## TF Frames
```bash
# To verify the broadcasted tf frames from world to talk
  ros2 run tf2_ros tf2_echo world talk
# To visualise the TF frames
  ros2 run tf2_tools view_frames
```
## For recording the ros2 bag
```bash
# For recording the ros2 bag
  ros2 launch beginner_tutorials launch_ros_bag.py record:=True
# to replay the rosbag, run listener in terminal 1
  ros2 run beginner_tutorials
# run the following after navigating to rosbag_output in terminal 2
  ros2 bag play rosbag2_2023_11_21-23_56_16_0.db3
```
## Integration Testing with gtest
```bash
# For performing the tests
  colcon test --packages-select beginner_tutorials
# For viewing the results
  cat log/latest_test/beginner_tutorials/stdout_stderr.log
```

## cppling and cppcheck test
```bash
# run cppcheck
   mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck.txt
# run cpplint
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name '*.cpp' -not -path './build/*') &> results/cpplint.txt
# google test
  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")

```
