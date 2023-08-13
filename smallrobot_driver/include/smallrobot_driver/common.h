#ifndef COMMON_H
#define COMMON_H


//常用头文件
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <typeinfo>
#include <inttypes.h>
#include <cstdlib>
#include <sys/time.h>
#include <sys/select.h>
#include <ctime>

#include <cerrno>
#include <cmath>
#include <bits/stdc++.h>

#include <cstring>
#include <string.h>
#include <signal.h>
#include <vector>

//C++ 11
#include <thread>
#include <chrono>
#include <mutex>

//tcp
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

//ros头文件
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "robot_state_publisher/robot_state_publisher.h"

//动作编程
#include "actionlib/server/action_server.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/server_goal_handle.h" 
#include "control_msgs/FollowJointTrajectoryAction.h"


//是否显示接收到的数据
//#define RECV_DISPLAY 1
#define USE_ROS 1


using namespace std;

//常量定义
const double PI = 3.1415926;

namespace MyFunctions
{
    extern bool condition;      //循环判断条件   false为不符合条件   true为符合条件
    extern void stop(int sign); //捕获信号量
    extern const bool ok();     //反馈condition变量

} // namespace MyFunctions





#endif // COMMON_H
