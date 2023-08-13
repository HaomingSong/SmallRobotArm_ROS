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
             *        将路径点赋值 角度获取 单点执行时间获取开始          *
             *******************************************************/
            // 路径点赋值
            for (int index = 0; index < smallrobotRobotPtr->NumberOfPoints; index++)
            {
                // 获得各个轴的   位置数据数据
                for (int i = 0; i < 6; i++)
                {
                    // 获取位置信息，也是脉冲信息， 位置 = (物理角度 / PI) * 单位脉冲 + 零点偏移
                    smallrobotRobotPtr->trajectory[index].position[i] =
                        (goal->trajectory.points[index].positions[i] * smallrobotRobotPtr->plu2angel[i]) / PI + smallrobotRobotPtr->zeroPlu[i];
                }

                /*******************************************************
                 *                     第一个点位                       *
                 *******************************************************/
                // 第一个轨迹点，各个关节速度为0
                if (index == 0)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        durations[i] = 0;
                        smallrobotRobotPtr->trajectory[index].period[i] = 0;
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
                            smallrobotRobotPtr->trajectory[index].period[i] = 0;
                        }
                        else
                        {
                            // 如果位置未发生变化
                            if (smallrobotRobotPtr->trajectory[index].position[i] == smallrobotRobotPtr->trajectory[index - 1].position[i])
                            {
                                durations[i] = 0;
                                smallrobotRobotPtr->trajectory[index].period[i] = 0;
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
                            smallrobotRobotPtr->trajectory[index].period[i] = 0;
                        }
                        else
                        {
                            // 如果位置没有发生变化
                            if (smallrobotRobotPtr->trajectory[index].position[i] == smallrobotRobotPtr->trajectory[index - 1].position[i])
                            {
                                durations[i] = 0;
                                smallrobotRobotPtr->trajectory[index].period[i] = 0;
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

                        // 获取位置信息，也是脉冲信息， 位置 = (物理角度 / PI) * 单位脉冲 + 零点偏移
                        smallrobotRobotPtr->trajectory[index].position[i] =
                            (goal->trajectory.points[index].positions[i] * smallrobotRobotPtr->plu2angel[i]) / PI + smallrobotRobotPtr->zeroPlu[i];

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
                          << " duration " << duration_mean/20 << "us, "
                          << (double)duration_mean / 20000000 << "s" << endl;
                /*******************************************************
                 *        将路径点赋值 角度获取 单点执行时间获取结束          *
                 *******************************************************/

                /*******************************************************
                 *   当所有点执行需要的时间都获取之后，开始获取速度            *
                 *******************************************************/
                // 第一个点,没有运行速度, 在前面 if (index == 0) 中已经设定完毕
                if (index != 0)
                {
                    // 获取角度差值
                    for (int i = 0; i < 6; i ++)
                    {
                        delta_postions[i] = 
                            smallrobotRobotPtr->trajectory[index].position[i] - 
                            smallrobotRobotPtr->trajectory[index - 1].position[i]

                    }

                    for (int i = 0; i < 6; i++)
                    {
                        // 只有速度不为0，且发生了位置偏移，才进行脉冲周期计算
                        if (goal->trajectory.points[index].velocities[i] != 0 && delta_postions[i] != 0)
                        {
                            // 周期 = 运行时间/脉冲差
                            smallrobotRobotPtr->trajectory[index].period[i] = abs(duration_mean /delta_postions[i]);

                            
                            if (smallrobotRobotPtr->trajectory[index].period[i] <= 120000)
                            {
                                smallrobotRobotPtr->trajectory[index].restPeriod[i] = 9;
                                smallrobotRobotPtr->trajectory[index].numberOfFullPeriod[i] = 
                                    (duration_mean / delta_postions[i]) / 20;
                                smallrobotRobotPtr->trajectory[index].numberOfPeriod[i] = (duration_mean - 
                                    delta_postions[i] * 20 * smallrobotRobotPtr->trajectory[index].numberOfFullPeriod[i];
                                std::cout << "小数算法" << std::endl;
                            }
                            else if(smallrobotRobotPtr->trajectory[index].period[i] > 120000 && smallrobotRobotPtr->trajectory[index].period[i] <= 3600000000)
                            {
                                smallrobotRobotPtr->trajectory[index].restPeriod[i] = 
                                    (int)__builtin_sqrt(smallrobotRobotPtr->trajectory[index].period[i]);
                                smallrobotRobotPtr->trajectory[index].numberOfFullPeriod[i] = 
                                    smallrobotRobotPtr->trajectory[index].restPeriod[i] + 1;
                                smallrobotRobotPtr->trajectory[index].numberOfPeriod[i] = 0;

                                std::cout << "大数算法" << std::endl;
                            }
                            else
                            {
                                //无法作出更慢的动作了
                                smallrobotRobotPtr->trajectory[index].restPeriod[i] = 60000;
                                smallrobotRobotPtr->trajectory[index].numberOfFullPeriod[i] = 60000;
                                smallrobotRobotPtr->trajectory[index].numberOfPeriod[i] = 0;
                            }
                        }
                    }
                }
            }

            // 打印
            // smallrobotRobotPtr->printTrajectory();

        std::cout << "预计使用" << (double)duration_total / 20000000 << "s" << std::endl;
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
    // smallrobotRobotPtr->sendTrajectory();

    // gettimeofday(&tStart, 0);
    // // 等待状态变化 无需特别高的实时性
    // usleep(500000); // 至少等待0.5s
    // while (!smallrobotRobotPtr->isArrived())
    // {
    //     usleep(10000); // 等待0.1s
    // }

    // gettimeofday(&tEnd, 0);
    // // 动作完成，反馈结果，设置完成状态
    // ros_result.error_code = ros_result.SUCCESSFUL;
    // as->setSucceeded(ros_result);

    // duration_total_actual = ((tEnd.tv_sec - tStart.tv_sec) * 1000000 + tEnd.tv_usec - tStart.tv_usec);
    // // cout << ", 实际使用" << ltime << "us，"

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
    std::cout << "预计使用" << (double)duration_total / 20000000 << "s, 实际使用"
              << (double)duration_total_actual / 1000000 << "s" << std::endl;
}