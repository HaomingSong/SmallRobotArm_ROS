#include "SmallrobotRos.h"


int main(int argc, char *argv[])
{
    //节点命名
    ros::init(argc, argv, "smallrobot_node");

    //私有参数获取句柄
    ros::NodeHandle nh("~");

    //IP和端口
    string smallrobot_ip;
    int smallrobot_port;

    //参数获取
    nh.param<string>("smallrobot_ip", smallrobot_ip, "127.0.0.1");
    nh.param<int>("smallrobot_port", smallrobot_port, 8080);

    //准备连接
    smallrobotRobotPtr->setServerIP(smallrobot_ip);
    smallrobotRobotPtr->setServerPort(smallrobot_port);


    //ros机械臂实例
    SmallrobotRobotRos * smallrobotRobotRos = new SmallrobotRobotRos();

    return 0;
}