cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)


add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

ament_target_dependencies(talker rclcpp std_msgs tf2_msgs)
ament_target_dependencies(listener rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_ts_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(talker "${cpp_ts_target}" "${tf2_ros_LIBRARIES}")
target_link_libraries(listener "${cpp_ts_target}")

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TESTFILES
  test/talker_test.cpp)
  ament_add_gtest(talker_test ${TESTFILES})
  ament_target_dependencies(talker_test rclcpp std_msgs rosidl_default_runtime)
endif()

ament_package()
