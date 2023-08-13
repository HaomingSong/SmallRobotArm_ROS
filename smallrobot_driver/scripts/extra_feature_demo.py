#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
from smallrobot_driver.msg import ExtraFeatures
from time import sleep


def ExtraFeaturesDemo():
    rospy.init_node('extraFeaturesCommander', anonymous = False )
    pub = rospy.Publisher("/smallrobot/extraFeatures", ExtraFeatures, queue_size=10)


    msg = ExtraFeatures()
    #根据功能编号进行调用，参考 SmallrobotRobotRos中 extraFeaturesCB回调函数的内容。

    # PIN0_ON = 8,            //GPIO_0高电平
    # PIN0_OFF = 9,           //GPIO_0低电平
    # PIN1_ON = 10,            //GPIO_1高电平
    # PIN1_OFF = 11,           //GPIO_1低电平
    # msg.Tag = 8
    # print ("GPIO_0 高电平")

    # 需要时，发送一次即可
    # pub.publish(msg)

    # 或者循环控制IO口的高低点平 周期1s
    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        
        print("夹取")
        msg.Tag = 10
        pub.publish(msg)
        sleep(0.5)
        msg.Tag = 9
        pub.publish(msg)
        sleep(4.5)

        print("释放")
        msg.Tag = 8
        pub.publish(msg)
        sleep(0.5)
        msg.Tag = 11
        pub.publish(msg)
        sleep(4.5)
        



if __name__ == '__main__':
    try:
        ExtraFeaturesDemo()
    except rospy.ROSInterruptException:
        pass