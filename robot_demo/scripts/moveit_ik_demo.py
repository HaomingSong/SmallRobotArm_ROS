#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        print("末端%s"%end_effector_link)
                        
        # 设置目标位置所使用的参考坐标系
        #reference_frame = 'base_link'
        reference_frame = 'link1'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.1)
        arm.set_max_velocity_scaling_factor(1.0)



        # 控制机械臂先回到初始化位置
        arm.set_named_target('zero')
        print ("zero...")
        arm.go()
        print ("Arrive zero")
        rospy.sleep(1)

               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame


        # # 设置点位
        # target_pose.header.stamp = rospy.Time.now()
        # target_pose.pose.position.x = 0.2
        # target_pose.pose.position.y = 0.0
        # target_pose.pose.position.z = 0.2
        # target_pose.pose.orientation.x = 0.0
        # target_pose.pose.orientation.y = 0.707
        # target_pose.pose.orientation.z = 0.0
        # target_pose.pose.orientation.w = 0.70729
        # # 设置机器臂当前的状态作为运动初始状态
        # arm.set_start_state_to_current_state()
        # # 设置机械臂终端运动的目标位姿
        # arm.set_pose_target(target_pose, end_effector_link)
        # # 规划运动路径
        # plan_success,traj,planning_time,error_code = arm.plan()
        # print(plan_success)
        # # 按照规划的运动路径控制机械臂运动
        # print("执行开始")
        # arm.execute(traj)
        # print("执行完毕")
        # rospy.sleep(1)


        # # 设置点位
        # target_pose.header.stamp = rospy.Time.now()
        # target_pose.pose.position.x = 0.0
        # target_pose.pose.position.y = 0.2
        # target_pose.pose.position.z = 0.2
        # target_pose.pose.orientation.x = 0.0
        # target_pose.pose.orientation.y = 0.707
        # target_pose.pose.orientation.z = 0.0
        # target_pose.pose.orientation.w = 0.70729
        # # 设置机器臂当前的状态作为运动初始状态
        # arm.set_start_state_to_current_state()
        # # 设置机械臂终端运动的目标位姿
        # arm.set_pose_target(target_pose, end_effector_link)
        # # 规划运动路径
        # plan_success,traj,planning_time,error_code = arm.plan()
        # print(plan_success)
        # # 按照规划的运动路径控制机械臂运动
        # print("执行开始")
        # arm.execute(traj)
        # print("执行完毕")
        # rospy.sleep(1)


        # # 设置点位
        # target_pose.header.stamp = rospy.Time.now()
        # target_pose.pose.position.x = 0.0
        # target_pose.pose.position.y = -0.2
        # target_pose.pose.position.z = 0.2
        # target_pose.pose.orientation.x = 0.0
        # target_pose.pose.orientation.y = 0.707
        # target_pose.pose.orientation.z = 0.0
        # target_pose.pose.orientation.w = 0.70729
        # # 设置机器臂当前的状态作为运动初始状态
        # arm.set_start_state_to_current_state()
        # # 设置机械臂终端运动的目标位姿
        # arm.set_pose_target(target_pose, end_effector_link)
        # # 规划运动路径
        # plan_success,traj,planning_time,error_code = arm.plan()
        # print(plan_success)
        # # 按照规划的运动路径控制机械臂运动
        # print("执行开始")
        # arm.execute(traj)
        # print("执行完毕")
        # rospy.sleep(1)


        # # 设置点位
        # target_pose.header.stamp = rospy.Time.now()
        # target_pose.pose.position.x = 0.3
        # target_pose.pose.position.y = 0.0
        # target_pose.pose.position.z = 0.20
        # target_pose.pose.orientation.x = 0.0
        # target_pose.pose.orientation.y = 0.0
        # target_pose.pose.orientation.z = 0.0
        # target_pose.pose.orientation.w = 1.0
        # # 设置机器臂当前的状态作为运动初始状态
        # arm.set_start_state_to_current_state()
        # # 设置机械臂终端运动的目标位姿
        # arm.set_pose_target(target_pose, end_effector_link)
        # # 规划运动路径
        # plan_success,traj,planning_time,error_code = arm.plan()
        # print(plan_success)
        # # 按照规划的运动路径控制机械臂运动
        # print("执行开始")
        # arm.execute(traj)
        # print("执行完毕")
        # rospy.sleep(1)

        # 设置点位
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.707
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.70729
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        # 规划运动路径
        plan_success,traj,planning_time,error_code = arm.plan()
        if (plan_success):
            numberOfpoints = len(traj.joint_trajectory.points)
            print("轨迹点数量%d" % numberOfpoints)
            print(traj.joint_trajectory.points[numberOfpoints-1].positions)
            # 按照规划的运动路径控制机械臂运动
            print("执行开始")
            execute_success = arm.execute(traj, wait = True)
            print(execute_success)
            print("执行完毕")
        else : 
            print("路径规划失败")
        rospy.sleep(1)

        # # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
