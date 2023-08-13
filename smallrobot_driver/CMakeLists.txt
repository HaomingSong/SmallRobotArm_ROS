cmake_minimum_required(VERSION 3.1.3)
project(smallrobot_driver)

## 需要生成消息，即自定义数据结构
find_package(catkin REQUIRED COMPONENTS
  message_generation            
  actionlib
  control_msgs
  joint_state_publisher
  robot_state_publisher
  roscpp
  sensor_msgs
  std_msgs
)

## 将msg文件夹中的文件进行转换生成ROS C++头文件
add_message_files(FILES
  ExtraFeatures.msg
)


## 生成ROS C++头文件的消息，将其以来的其他消息结构放置在此处
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


# 头文件引入
include_directories(
  include include/smallrobot_driver
  ${catkin_INCLUDE_DIRS}
)


# ROS
add_executable(smallrobot_main
  include/smallrobot_driver/common.h
  include/smallrobot_driver/Smallrobot.h
  include/smallrobot_driver/SmallrobotRos.h
  src/common.cpp
  src/Smallrobot.cpp
  src/SmallrobotRos.cpp
  src/smallrobot_main.cpp
)
add_dependencies(smallrobot_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smallrobot_main
  ${catkin_LIBRARIES} -lpthread
)
