cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
# find_package(beginner_tutorials REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# set(srv_files
#   "srv/ChangeString.srv"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  # ${srv_files}
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

#include_directories(include/beginner_tutorials)

add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_ts_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(talker "${cpp_ts_target}")
target_link_libraries(listener "${cpp_ts_target}")
# target_link_libraries(change_string_client "${cpp_ts_target}")

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
