#include "SmallrobotRos.h"

// 机械臂实例
// SmallrobotRobot * smallrobotRobotPtr = new SmallrobotRobot("192.168.31.215", 8080);
SmallrobotRobot *smallrobotRobotPtr = new SmallrobotRobot();

// SmallrobotRobot 监听友元函数
void listening(SmallrobotRobot *p)
{
    std::cout << "void listening \n";
    p->listening();
}

SmallrobotRobotRos::SmallrobotRobotRos()
{
    // 句柄实例
    ros::NodeHandle nh;

    // 动作名称
    action_name = "/smallrobot/smallrobot_controller/follow_joint_trajectory";

    // 初始化关节变量
    joint_msg.name.resize(6);
    joint_msg.position.resize(6);
    joint_msg.header.frame_id = "/smallrobot";

    // 初始化ros_feedback
    ros_feedback.header.frame_id = "/smallrobot";
    ros_feedback.desired.positions.resize(6);
    ros_feedback.actual.positions.resize(6);

    // 关节命名
    joint_msg.name[0] = "joint1";
    joint_msg.name[1] = "joint2";
    joint_msg.name[2] = "joint3";
    joint_msg.name[3] = "joint4";
    joint_msg.name[4] = "joint5";
    joint_msg.name[5] = "joint6";

    // 角度差
    memset(delta_postions, 0, sizeof(delta_postions));
    // 各个关节执行完毕所需的时间
    memset(durations, 0, sizeof(durations));
    // 时间记录
    duration_sum = 0;
    // 有效时间数据个数
    numberOfValidDuration = 0;

    // 功能
    extra_features_msg.Tag = 0;
    extra_features_msg.Position.resize(6);
    moveMode = 0; // 0:轨迹方式,按照Moveit规划路径进行运动  1:单点方式,即直接运行

    // 启动
    smallrobotRobotPtr->startConstruction();

    // 监听机械臂
    std::thread t_listening = std::thread(listening, smallrobotRobotPtr);

    // 关节发布者初始化
    joint_pub = nh.advertise<sensor_msgs::JointState>("/smallrobot/joint_states", 1);

    // 服务器初始化
    as = new Server(nh, action_name, boost::bind(&SmallrobotRobotRos::executeCB, this, _1), false);

    // 服务器开启
    as->start();

    // 功能订阅者初始化
    extra_features_sub = nh.subscribe("/smallrobot/extraFeatures", 1, &SmallrobotRobotRos::extraFeaturesCB, this);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 更新关节数据
    jointStateUpdate();

    if (t_listening.joinable())
    {
        t_listening.join();
    }
    else
    {
        std::cout << "thread cannot join" << std::endl;
    }
    std::cout << "退出\n";
    ros::shutdown();
}

SmallrobotRobotRos::~SmallrobotRobotRos()
{
}

// extraFeatures功能回调函数
void SmallrobotRobotRos::extraFeaturesCB(const smallrobot_driver::ExtraFeaturesConstPtr &msg)
{
    // 获取消息标签
    extra_features_msg.Tag = msg->Tag;

    switch (msg->Tag)
    {
    case STOPPED:
        // 紧急制动
        smallrobotRobotPtr->robot_stop();
        break;

    case PWM_START:
        // 获取数据
        smallrobotRobotPtr->pwm_handle.PSC = msg->PSC;
        smallrobotRobotPtr->pwm_handle.ARR = msg->ARR;
        smallrobotRobotPtr->pwm_handle.CCR1 = msg->CCR1;
        smallrobotRobotPtr->pwm_handle.PluseCount = msg->PluseCount;
        // 发送pwm数据
        smallrobotRobotPtr->pwm_start();
        break;

    case PWM_STOP:
        // 关闭pwm
        smallrobotRobotPtr->pwm_stop();
        break;

    case PIN0_ON:
        smallrobotRobotPtr->pin0_on();
        break;

    case PIN0_OFF:
        smallrobotRobotPtr->pin0_off();
        break;

    case PIN1_ON:
        smallrobotRobotPtr->pin1_on();
        break;

    case PIN1_OFF:
        smallrobotRobotPtr->pin1_off();
        break;

    case TOGGLE_ENABLE_PINS: // 12
        /*
         * 关节序号  0   1   2   3   4   5   6   7     8pwm    全部
         * 输入数值  1   2   4   8   16  32  64  128   256     (511)0x1ff
         *
         * 可以进行各种使能组合,不过为了方便,建议在终端操作时,采用上述几个数值
         */

        smallrobotRobotPtr->enable_pins = msg->PSC & 0x01ff;
        cout << "smallrobotRobotPtr->enable_pins " << smallrobotRobotPtr->enable_pins << endl;
        smallrobotRobotPtr->toggle_enable_pins();
        break;

    case USART_START: // 13
        // 开启串口通信
        smallrobotRobotPtr->usart_start();
        break;

    case USART_STOP: // 14
        // 关闭USART通信中断
        smallrobotRobotPtr->usart_stop();
        break;

    case RS485_ENABLE: // 17
        smallrobotRobotPtr->rs485_enable();
        break;

    case RS485_DISABLE: // 18
        smallrobotRobotPtr->rs485_disable();
        break;

    case LOCATION_SETTING: // 20
        // 设置当前角度脉冲数值,必须要在机械臂完全停止运动的时候使用这个功能
        smallrobotRobotPtr->location_setting_handle.state = LOCATION_SETTING;

        // 6轴
        smallrobotRobotPtr->location_setting_handle.position[0] = msg->Position[0];
        smallrobotRobotPtr->location_setting_handle.position[1] = msg->Position[1];
        smallrobotRobotPtr->location_setting_handle.position[2] = msg->Position[2];
        smallrobotRobotPtr->location_setting_handle.position[3] = msg->Position[3];
        smallrobotRobotPtr->location_setting_handle.position[4] = msg->Position[4];
        smallrobotRobotPtr->location_setting_handle.position[5] = msg->Position[5];

        // 调用API
        smallrobotRobotPtr->location_setting();
        break;

#ifdef USE_FAST_MOVE
    case MoveMode_SETTING:
        // 0 单点运动模式,直接运动到最终位姿，从而达到高速响应的效果
        // 1 轨迹运动模式
        moveMode = msg->PSC;
        std::cout << "MoveMode:" << moveMode << std::endl;
        break;
#endif

    default:
        std::cout << "错误的功能代码，请检查后在进行输入" << std::endl;
        break;
    }
}

// goal回调函数
void SmallrobotRobotRos::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::cout << "\033[1m\033[32m SmallrobotRobotRos::executeCB start \033[0m 当前机械臂状态为"
              << (int)smallrobotRobotPtr->location.state << std::endl;

    if (smallrobotRobotPtr->location.state != STOPPED)
    { // 如果上一个任务没有执行完毕

        std::cout << "上一个任务没有完成" << std::endl;
        ros_result.error_code = ros_result.OLD_HEADER_TIMESTAMP;
        ros_result.error_string = "Action server is busy now, ignore this requestion\n";
        as->setAborted(ros_result);
        // as->setSucceeded(ros_result);
        return;
    }

#ifdef USE_FAST_MOVE
    if (moveMode == 0) // 轨迹模式
    {
#endif
        duration_total = 0;

        // 有时可能会需要路径点重排列，当如果是固定赋值，则不需要
        if (ros::ok())
        {
            smallrobotRobotPtr->NumberOfPoints = goal->trajectory.points.size(); // 获取路径点数量

            // 将路点的终点写入ros_feedback中
            ros_feedback.desired.positions[0] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[0];
            ros_feedback.desired.positions[1] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[1];
            ros_feedback.desired.positions[2] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[2];
            ros_feedback.desired.positions[3] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[3];
            ros_feedback.desired.positions[4] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[4];
            ros_feedback.desired.positions[5] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[5];

            /*******************************************************
             *        将路径点赋值 角度获取 单点执行时间获取开始
             *        1 计算出各个路径点的位置信息
             *        2 计算出到达各个路径点需要的运行角度信息          *
             *        3 计算出各个点所花费的时间
             *        4 计算出各个路径点的各个轴运动速度
             *******************************************************/
            // 路径点赋值
            for (int index = 0; index < smallrobotRobotPtr->NumberOfPoints; index++)
            {
                // 1 计算出各个路径点的位置信息
                for (int i = 0; i < 6; i++)
                {
                    // 获取位置信息，也是脉冲信息， 位置 = (物理角度 / PI) * 单位脉冲 + 零点偏移
                    smallrobotRobotPtr->trajectory[index].position[i] =
                        (goal->trajectory.points[index].positions[i] * smallrobotRobotPtr->plu2angel[i]) / PI + smallrobotRobotPtr->zeroPlu[i];
                }

                // 2 计算出到达各个路径点需要的运行角度信息
                if (index != 0)
                {

                    for (int i = 0; i < 6; i++)
                    {
                        delta_postions[i] =
                            smallrobotRobotPtr->trajectory[index].position[i] -
                            smallrobotRobotPtr->trajectory[index - 1].position[i];
                    }
                }

                /*******************************************************
                 *         3 计算出各个点所花费的时间                      *
                 *******************************************************/
                // 第一个轨迹点，各个关节速度为0
                if (index == 0)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        durations[i] = 0;
                    }

                    /***************************************************
                     * 第一个点是机械臂当前位置点,因此机械臂到达此点花费的时间0  *
                     ****************************************************/
                    // 获得此点运行的平均时间
                    duration_mean = 0;
                    // 时间记录
                    duration_sum = 0;
                    // 有效时间数据个数
                    numberOfValidDuration = 0;
                }

                // 最后一个点
                else if (index == smallrobotRobotPtr->NumberOfPoints - 1)
                {
                    // 最后一个点 各个轴的情况
                    for (int i = 0; i < 6; i++)
                    {
                        // 最后一个轨迹点速度为0，所以要执行这个点，则必须和前一个点的速度保持一致
                        if (goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 2].velocities[i] == 0)
                        { // 如果倒数第二个点已经速度为0的话，那么最后一个点和前面保持一致
                            durations[i] = 0;
                        }
                        else
                        {
                            // 如果此关节的位置未发生变化
                            if (delta_postions[i] == 0)
                            {
                                durations[i] = 0;
                            }

                            // 如果位置发生变化
                            else
                            {
                                durations[i] =
                                    (goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[i] -
                                     goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 2].positions[i]) *
                                    200000000 / goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 2].velocities[i];

                                // 和前一个点的速度保持一致
                                smallrobotRobotPtr->trajectory[index].numberOfFullPeriod[i] = smallrobotRobotPtr->trajectory[index - 1].numberOfFullPeriod[i];
                                smallrobotRobotPtr->trajectory[index].restPeriod[i] = smallrobotRobotPtr->trajectory[index - 1].restPeriod[i];
                                smallrobotRobotPtr->trajectory[index].numberOfPeriod[i] = smallrobotRobotPtr->trajectory[index - 1].numberOfPeriod[i];
                                smallrobotRobotPtr->trajectory[index].period[i] = smallrobotRobotPtr->trajectory[index - 1].period[i];

                                // 累加出6个轴的总时间
                                duration_sum += abs(durations[i]);
                                // 记录有效关节数，防止一些关节不进行运动，而导致执行所需要的平均时间偏低
                                numberOfValidDuration++;
                            }
                        }
                    }

                    // 获得此点运行的平均时间
                    duration_mean = numberOfValidDuration == 0 ? 0 : round(duration_sum / numberOfValidDuration);
                    // 时间记录
                    duration_sum = 0;
                    // 有效时间数据个数
                    numberOfValidDuration = 0;
                }

                // 路径中间的点
                else
                {
                    // 获得各个轴的
                    for (int i = 0; i < 6; i++)
                    {
                        // 其余的点，运行时间 = 角度差/速度，  周期 = 运行时间 / 位置差
                        if (goal->trajectory.points[index].velocities[i] == 0)
                        {
                            durations[i] = 0;
                        }
                        else
                        {
                            // 如果位置没有发生变化
                            if (delta_postions[i] == 0)
                            {
                                durations[i] = 0;
                            }
                            else
                            {
                                // 如果位置有变化 算出当前轴执行完当前点，所需要的时间(单位为us)
                                durations[i] =
                                    (goal->trajectory.points[index].positions[i] - goal->trajectory.points[index - 1].positions[i]) * 200000000 / goal->trajectory.points[index].velocities[i];

                                // smallrobotRobotPtr->trajectory[index].period[i] =
                                //     abs((1000000 * PI) / goal->trajectory.points[index].velocities[i] / smallrobotRobotPtr->plu2angel[i]);

                                // 累加出总时间
                                duration_sum += abs(durations[i]);
                                // 记录有效关节数，防止一些关节不进行运动，而导致执行所需要的平均时间偏低
                                numberOfValidDuration++;
                            }
                        }

                        // 分段获取速度信息
                        // std::cout << "positions[" << i <<"] " << this->goal.points[index].positions[i] <<
                        //      "  Goal : " << goal->trajectory.points[index].positions[i] << endl;
                        // cout << goal->trajectory.points[index].velocities[i] << " ";
                    }

                    // 获得此点运行的平均时间
                    duration_mean = numberOfValidDuration == 0 ? 0 : round(duration_sum / numberOfValidDuration);
                    // 时间记录
                    duration_sum = 0;
                    // 有效时间数据个数
                    numberOfValidDuration = 0;
                }

                // 保存这个点运行需要的时间
                smallrobotRobotPtr->trajectory[index].duration = duration_mean;
                duration_total += duration_mean;

                std::cout << "第 " << index << "个点"
                          << " duration " << duration_mean / 200 << "us, "
                          << (double)duration_mean / 200000000 << "s" << endl;
                /*******************************************************
                 *        将路径点赋值 角度获取 单点执行时间获取结束          *
                 *******************************************************/

                /*******************************************************
                 *   当所有点执行需要的时间都获取之后，开始获取速度            *
                 *******************************************************/

                // 第一个点,没有运行速度, 在前面 if (index == 0) 中已经设定完毕
                if (index != 0)
                {

                    for (int i = 0; i < 6; i++)
                    {
                        // 只有速度不为0，且发生了位置偏移，才进行脉冲周期计算
                        if (goal->trajectory.points[index].velocities[i] != 0 && delta_postions[i] != 0)
                        {
                            // 周期 = 运行时间/脉冲差
                            smallrobotRobotPtr->trajectory[index].period[i] = abs(duration_mean / delta_postions[i]);

                            subVelocityModule(index, i);
                        }
                        // 如果此关节在此点运行过程中 没有任何位置移动，则在此周期内，不允许此关节发生移动
                        else if (delta_postions[i] == 0)
                        {
                            smallrobotRobotPtr->trajectory[index].period[i] = duration_mean;
                            subVelocityModule(index, i);
                        }
                    }
                }
            }

            // 打印
            // smallrobotRobotPtr->printTrajectory();

            std::cout << "预计使用" << (double)duration_total / 200000000 << "s" << std::endl;
        }
#ifdef USE_FAST_MOVE
    }
    else if (moveMode == 1) // 快速响应模式
    {
        smallrobotRobotPtr->NumberOfPoints = goal->trajectory.points.size(); // 获取路径点数量

        // 将路点的终点写入ros_feedback中
        ros_feedback.desired.positions[0] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[0];
        ros_feedback.desired.positions[1] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[1];
        ros_feedback.desired.positions[2] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[2];
        ros_feedback.desired.positions[3] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[3];
        ros_feedback.desired.positions[4] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[4];
        ros_feedback.desired.positions[5] = goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[5];

        // 路径终点写入通讯协议对象
        for (int i = 0; i < 6; i++)
        {
            // 获取位置信息，也是脉冲信息， 位置 = (物理角度 / PI) * 单位脉冲 + 零点偏移
            smallrobotRobotPtr->trajectory[0].position[i] =
                (goal->trajectory.points[smallrobotRobotPtr->NumberOfPoints - 1].positions[i] *
                 smallrobotRobotPtr->plu2angel[i]) /
                    PI +
                smallrobotRobotPtr->zeroPlu[i];

            // 最高速度
            smallrobotRobotPtr->trajectory[0].period[i] = smallrobotRobotPtr->fastMovePeriod[i];
        }

        smallrobotRobotPtr->NumberOfPoints = 1;
    }
#endif

    // 调用smallrobotRobot中的sendTrajectory进行发送数据的操作
    // smallrobotRobotPtr->printTrajectory();
    smallrobotRobotPtr->sendTrajectory();

    gettimeofday(&tStart, 0);
    // 等待状态变化 无需特别高的实时性
    usleep(500000); // 至少等待0.5s
    while (!smallrobotRobotPtr->isArrived())
    {
        usleep(10000); // 等待0.1s
    }

    gettimeofday(&tEnd, 0);
    // 动作完成，反馈结果，设置完成状态
    ros_result.error_code = ros_result.SUCCESSFUL;
    as->setSucceeded(ros_result);

    duration_total_actual = ((tEnd.tv_sec - tStart.tv_sec) * 1000000 + tEnd.tv_usec - tStart.tv_usec);
    // cout << ", 实际使用" << ltime << "us，"

    // std::cout << "\033[32mSmallrobotRobotRos::executeCB finished\033[0m" << endl;

    // 检测是否到达最终位置，后期通过一个话题实现紧急取消的功能，使用标志位来通知此处是否被取消
    /*
    for (int i = 0; i < axies; i ++)
    {
        if (feedback.point.positions[i] != this->goal.points[smallrobotRobotPtr->NumberOfPoints-1].positions[i])
        {
            //动作未完成，反馈抢占性取消
            as->setPreempted();
            return;
        }
    }*/

    // sleep(10);

    std::cout << "路径执行完成" << std::endl;
    std::cout << "预计使用" << (double)duration_total / 200000000 << "s, 实际使用"
              << (double)duration_total_actual / 1000000 << "s" << std::endl;
}

// 向ros系统中更新关节状态
void SmallrobotRobotRos::jointStateUpdate()
{
    while (ros::ok())
    {
        for (int i = 0; i < 6; i++)
        {
            joint_msg.position[i] = (smallrobotRobotPtr->location.position[i] - smallrobotRobotPtr->zeroPlu[i]) * PI / smallrobotRobotPtr->plu2angel[i];
            ros_feedback.actual.positions[i] = joint_msg.position[i];
        }
        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);

        if (smallrobotRobotPtr->location.state == RUNING)
        {
            ros_feedback.header.stamp = ros::Time::now();
            as->publishFeedback(ros_feedback);
        }

        usleep(100000);
    }
}

// 重排序，urdf设计的顺序比较好，可以不用这个
void SmallrobotRobotRos::reorder(trajectory_msgs::JointTrajectory trajectory)
{
}

// 机械臂根据编码器数据归零，由于有不同编码器，此部分通常由用户完成
void SmallrobotRobotRos::return_to_zero()
{
    // 先关闭Location上传
    smallrobotRobotPtr->upload_stop();

    // sleep(1);

    // 获取编码器数据
    smallrobotRobotPtr->usart_start();

    // sleep(1);

    // 根据编码器工厂进行通信设计，如果编码器是被动方式，数据放入到缓存中
    char s[18] = "Hello world ros!\n";
    smallrobotRobotPtr->usart_tx_len = 17;
    memcpy(smallrobotRobotPtr->usartTXBuffer, s, smallrobotRobotPtr->usart_tx_len);

    // 调用发送
    smallrobotRobotPtr->usart_send();

    sleep(1);

    // 等待获得编码器数据，然后进行执行，慢慢调整数据到零点位置
    //
    //
    //
    //

    /*
    //设置一个执行点
    smallrobotRobotPtr->NumberOfPoints = 1;
    //先进行清零
    memset(&smallrobotRobotPtr->trajectory[0], 0, PointSize);
    //设置执行位置
    // smallrobotRobotPtr->trajectory[0].position[0] = 0;
    // smallrobotRobotPtr->trajectory[0].position[1] = 0;
    // smallrobotRobotPtr->trajectory[0].position[2] = 0;
    // smallrobotRobotPtr->trajectory[0].position[3] = 0;
    // smallrobotRobotPtr->trajectory[0].position[4] = 0;
    // smallrobotRobotPtr->trajectory[0].position[5] = 0;
    // smallrobotRobotPtr->trajectory[0].position[6] = 0;
    // smallrobotRobotPtr->trajectory[0].position[7] = 0;
    //无需运动的关节，将于10ms关闭
    smallrobotRobotPtr->trajectory[0].period = 10000;
    //设置执行速度
    smallrobotRobotPtr->trajectory[0].duration[0] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[1] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[2] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[3] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[4] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[5] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[6] = 5000;
    smallrobotRobotPtr->trajectory[0].duration[7] = 5000;
    //调用smallrobotRobot中的sendTrajectory进行发送数据的操作
    smallrobotRobotPtr->printTrajectory();
    //发送数据
    smallrobotRobotPtr->sendTrajectory();
    */

    // 获取编码器数据

    // 关闭串口
    // smallrobotRobotPtr->usart_stop();

    // sleep(1);

    // 开启Location上传
    // smallrobotRobotPtr->upload_start();
}

// 辅助运算模块
void SmallrobotRobotRos::subVelocityModule(int pointIndex, int jointIndex)
{
    if (smallrobotRobotPtr->trajectory[pointIndex].period[jointIndex] <= 1200000)
    {
        smallrobotRobotPtr->trajectory[pointIndex].restPeriod[jointIndex] = 99;

        smallrobotRobotPtr->trajectory[pointIndex].numberOfFullPeriod[jointIndex] =
            smallrobotRobotPtr->trajectory[pointIndex].period[jointIndex] / 200 - 1;

        smallrobotRobotPtr->trajectory[pointIndex].numberOfPeriod[jointIndex] = duration_mean / 200 -
            delta_postions[jointIndex] * smallrobotRobotPtr->trajectory[pointIndex].numberOfFullPeriod[jointIndex];

        std::cout << "小数算法" << std::endl;
    }
    else if (smallrobotRobotPtr->trajectory[pointIndex].period[jointIndex] > 1200000 && smallrobotRobotPtr->trajectory[pointIndex].period[jointIndex] <= 3600000000)
    {
        smallrobotRobotPtr->trajectory[pointIndex].restPeriod[jointIndex] =
            (int)__builtin_sqrt(smallrobotRobotPtr->trajectory[pointIndex].period[jointIndex] / 2);

        smallrobotRobotPtr->trajectory[pointIndex].numberOfFullPeriod[jointIndex] =
            smallrobotRobotPtr->trajectory[pointIndex].restPeriod[jointIndex];

        smallrobotRobotPtr->trajectory[pointIndex].numberOfPeriod[jointIndex] = 0;

        std::cout << "大数算法" << std::endl;
    }
    else
    {
        // 无法作出更慢的动作了
        smallrobotRobotPtr->trajectory[pointIndex].restPeriod[jointIndex] = 60000;
        smallrobotRobotPtr->trajectory[pointIndex].numberOfFullPeriod[jointIndex] = 60000;
        smallrobotRobotPtr->trajectory[pointIndex].numberOfPeriod[jointIndex] = 0;
    }
}