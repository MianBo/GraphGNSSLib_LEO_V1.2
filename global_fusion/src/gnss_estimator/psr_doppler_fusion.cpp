/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 *
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 *
 * Main fucntions: pseudorange/Doppler velocity fusion using factor graph optimization
 * input: pseudorange, Doppler velocity from GPS/BeiDou.
 * output: position of the GNSS receiver
 * Date: 2020/11/27
 *******************************************************/

// std inputs and outputs, fstream
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
// time
#include <time.h>
// algorithm
#include <algorithm>

// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
// ros
#include <ros/ros.h>
/* Reference from NovAtel GNSS/INS */
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"
#include <nlosexclusion/GNSS_Raw_Array.h>

#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../tic_toc.h"

// allign
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/NavSatFix.h>

// rtklib
#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"

// /* factor graph related head file */
#include "../../include/gnss_estimator/psr_doppler_fusion.h"

class psr_doppler_fusion
{
    ros::NodeHandle nh; // ROS节点句柄
    std::string frame_id = "map"; // ROS消息中使用的坐标系名称
    /* ros subscriber */
    ros::Publisher pub_WLSENU, pub_FGOENU, pub_global_path, pub_fgo_llh; // 发布WLS的ENU,FGO的ENU,全局轨迹路径，FGO优化结果的LLH坐标
    std::map<double, nlosexclusion::GNSS_Raw_Array> gnss_raw_map; // 存储GNSS原始数据的map，以时间戳为键
    std::map<double, nav_msgs::Odometry> doppler_map; // 存储多普勒数据的map

    GNSS_Tools m_GNSS_Tools; // utilities

    /* subscriber */
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> gnss_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw;

    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux;

    /* thread for data processing */
    std::thread optimizationThread;

    int gnss_frame = 0;
    Eigen::Matrix<double, 3, 1> ENU_ref;
    int slidingWindowSize = 10000000; // ? epoch measurements 150 100000 // 滑动窗口大小（wc有点抽象）
    bool hasNewData = false;

    /* latest state in ENU */
    Eigen::Matrix<double, 3, 1> FGOENULatest;

    /* path in ENU */
    nav_msgs::Path fgo_path; // FGO优化后的轨迹路径，用于可视化

    /* log path */
    std::string gnss_fgo_path; // 存储结果日志文件的路径，保存优化结果

private:
    // std::unique_ptr<factor_graph> factor_graph_ptr_; // factor graph ptr
    FactorGraph factor_graph; // 声明并创建一个FactorGraph类的实例来处理因子图优化相关操作

    bool initializeFactorGraph(ceres::Solver::Options &options) // 用于初始化因子图的相关参数和配置
    {
        /* initialize the factor graph size */
        factor_graph.setWindowSize(slidingWindowSize); // 调用FactorGraph对象的setWindowSize方法，将滑动窗口大小设置为之前定义的slidingWindowSize变量值

        /* set up ceres-solver options */
        factor_graph.setupSolverOptions(options); // 调用FactorGraph对象的setupSolverOptions方法，将传入的Ceres求解器选项配置传递给因子图，用于设置优化器的各种参数

        /* set up loss functions (Huber, Cauchy, NULL)*/
        factor_graph.setupLossFunction("Huber"); // 调用FactorGraph对象的setupLossFunction方法，配置损失函数类型为"Huber"，可以减小异常值对优化结果的影响，提高鲁棒性。
    }

public:
    psr_doppler_fusion()
    {
        /* setup gnss_fgo_path */
        // 获取ros参数服务器里面的gnss_fgo_path参数，这玩意儿指定优化结果的输出文件路径，应该是对应的data_Whampoa_20210521_GNSSLEO.launch
        if (!ros::param::get("~gnss_fgo_path", gnss_fgo_path))
        {
            std::cerr << "ROS parameter 'gnss_fgo_path' of psr_doppler_fusion is not set!" << std::endl;
            gnss_fgo_path = "./GNSS_only_FGO_trajectoryllh_psr_dop_fusion.csv"; // 默认路径是当前目录下的GNSS_only_FGO_trajectoryllh_psr_dop_fusion.csv文件，这里的路径可能是要改的
        }
        std::cout << "gnss_fgo_path-> " << gnss_fgo_path << std::endl;
        /* thread for factor graph optimization */
        optimizationThread = std::thread(&psr_doppler_fusion::solveOptimization, this); // 独立线程执行因子图优化，避免阻塞主线程

        pub_WLSENU = nh.advertise<nav_msgs::Odometry>("WLSGoGPS", 100); // 发布WLS_ENU
        pub_FGOENU = nh.advertise<nav_msgs::Odometry>("FGO", 100);      // 发布FGP_ENU
        pub_fgo_llh = nh.advertise<sensor_msgs::NavSatFix>("fgo_llh", 100); // 发布FGO_LLH
        // 订阅GNSS伪距和多普勒速度数据
        gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000)); // GNSS only
        doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));                   // GNSS only

        // gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/leo_raw_publisher_node/LEOPsrCarRov1", 10000));// LEO only
        // doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/psr_spp_gnssleo_node/LEODopVelRov", 10000));// LEO only
        // gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov", 10000));// GNSS+LEO
        // doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/psr_spp_gnssleo_node/GNSS_LEO_DopVelRov", 10000));// GNSS+LEO
        syncdoppler2GNSSRaw.reset(new message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub, *doppler_sub, 10000));
        syncdoppler2GNSSRaw->registerCallback(boost::bind(&psr_doppler_fusion::gnssraw_doppler_msg_callback, this, _1, _2));//时间同步器，同步两个订阅者的数据

        pub_global_path = nh.advertise<nav_msgs::Path>("/FGOGlobalPath", 100); // 创建用于发布全局轨迹路径的ROS发布者

        /* reference point for ENU calculation */
        ENU_ref << ref_lon, ref_lat, ref_alt; // ENU坐标系的参考点
    }

    /**
     * @brief perform factor graph optimization
     * @param none
     * @return none
     */
    void solveOptimization()
    {
        /* clear data stream */
        factor_graph.clearDataStream(); // 清空因子图数据流，确保没有残留数据
        while (1)
        {
            m_gnss_raw_mux.lock();

            if (factor_graph.getDataStreamSize() > 3 && hasNewData) // 如果因子图中数据流至少有4个才能进行有效优化
            {
                /* define the problem */
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary;
                /* start clock for factor graph optimization */
                TicToc OptTime;
                /* initialize factor graph */
                initializeFactorGraph(options);
                /* clear variables */
                factor_graph.clearVariables();
                /* get data stream size */
                factor_graph.getDataStreamSize(); // 获取数据流大小，更新因子图内部的数据计数。
                /* setup memory for state */
                factor_graph.setupStateMemory(); // 为状态变量分配内存空间
                /* initialize factor graph parameters */
                factor_graph.initializeFactorGraphParas(); // 初始化因子图参数，如因子数量、状态数量等计数器
                /* initialize the previous optimzied states */
                factor_graph.initializeOldGraph(); // 初始化之前优化过的状态，利用历史优化结果作为当前优化的初值
                /* initialize the newly added states */
                factor_graph.initializeNewlyAddedGraph(); // 初始化新添加的状态，使用加权最小二乘解作为初值
                /* add parameter blocks */
                factor_graph.addParameterBlocksToGraph(problem);// 将参数块添加到优化问题中，这些参数块代表需要优化的状态变量
                /* fix the first parameter block */
                factor_graph.fixFirstState(false, problem); // 设置是否固定第一个状态参数块，这里传入false表示不固定第一个状态
                /* add Doppler FACTORS to factor graph */
                factor_graph.addDopplerFactors(problem); // 向因子图中添加多普勒因子
                /* add pseudorange FACTORS to factor graph */
                factor_graph.addPseudorangeFactors(problem); // 向因子图中添加伪距因子
                /* solve the factor graph */
                factor_graph.solveFactorGraph(problem, options, summary); // 执行因子图优化，使用Ceres求解器求解优化问题
                /* save graph state to variables */
                factor_graph.saveGraphStateToVector(); // 将优化后的图状态保存到向量中，供后续计算使用
                /* set reference point for ENU calculation */
                factor_graph.setupReferencePoint(); // 设置ENU坐标计算的参考点
                /* get the path in factor graph */
                FGOENULatest = factor_graph.getLatestStateENU(); // 获取因子图中最新的ENU坐标状态
                /* publish the lastest state in factor graph */
                factor_graph.printLatestStateENU(); // 打印最新的ENU状态，用于调试和监控
                /* publish the path from FGO */
                fgo_path = factor_graph.getPathENU(fgo_path);
                pub_global_path.publish(fgo_path); // 获取优化后的轨迹路径并发布到ROS中，供RVIZ等工具可视化
                /* remove the data outside sliding window */
                factor_graph.removeStatesOutsideSlidingWindow(); // 移除滑动窗口外的数据，维持滑动窗口大小限制
                /* log result */
                factor_graph.logResults(gnss_fgo_path); // 将优化结果记录到文件中，文件路径由gnss_fgo_path指定
                /* free memory (done when finish all the data) */
                // factor_graph.freeStateMemory();
                hasNewData = false; // 重置新数据标志，表示当前数据已处理完毕
                /** */
                std::cout << "OptTime-> " << OptTime.toc() << std::endl;
            }
            m_gnss_raw_mux.unlock();
            std::chrono::milliseconds dura(10); // this thread sleep for 10 ms
            std::this_thread::sleep_for(dura);
        }
    }

    /**
   * @brief gnss raw msg and doppler msg callback
   * @param gnss raw msg and doppler msg
   * @return void
   @
   */
    void gnssraw_doppler_msg_callback(const nlosexclusion::GNSS_Raw_ArrayConstPtr &gnss_msg, const nav_msgs::OdometryConstPtr &doppler_msg)
    {
        m_gnss_raw_mux.lock();
        hasNewData = true;
        gnss_frame++;
        double time_frame = doppler_msg->pose.pose.position.x;
        /* save the  */
        if (checkValidEpoch(time_frame) && m_GNSS_Tools.checkRepeating(*gnss_msg)) // 检查时间帧是否在有效范围内且GNSS消息是否为重复数据
        {
            if (gnss_msg->GNSS_Raws.size())
            {
                doppler_map[time_frame] = *doppler_msg;
                gnss_raw_map[time_frame] = *gnss_msg; // 将多普勒和GNSS原始数据分别存储到对应的映射表中，以时间戳为键

                factor_graph.input_gnss_raw_data(*gnss_msg, time_frame);
                factor_graph.input_doppler_data(*doppler_msg, time_frame); // 将GNSS原始数据和多普勒数据输入到因子图中，供后续优化使用

                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                    m_GNSS_Tools.getAllPositions(*gnss_msg),
                    m_GNSS_Tools.getAllMeasurements(*gnss_msg),
                    *gnss_msg, "WLS"); // 调用WLS计算SPP
                Eigen::Matrix<double, 3, 1> WLSENU;
                WLSENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
                LOG(INFO) << "WLSENU -> " << std::endl
                          << WLSENU;
                LOG(INFO) << "doppler_map.size() -> " << std::endl
                          << doppler_map.size(); // 使用Google日志库输出WLS解算得到的ENU坐标和多普勒数据映射表的大小

                nav_msgs::Odometry odometry; // 创建Odometry消息并设置帧ID，用于发布WLS解算结果
                odometry.header.frame_id = frame_id; //"map";
                odometry.child_frame_id = frame_id;  //"map";
                odometry.pose.pose.position.x = WLSENU(0);
                odometry.pose.pose.position.y = WLSENU(1);
                odometry.pose.pose.position.z = WLSENU(2);
                pub_WLSENU.publish(odometry);
            }
        }

        /* release the lock */
        m_gnss_raw_mux.unlock();
    }

    ~psr_doppler_fusion()
    {
    }
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;                             // output to console
    google::InitGoogleLogging(argv[0]);                // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags
    ros::init(argc, argv, "psr_doppler_fusion_node");
    ROS_INFO("\033[1;32m----> psr_doppler_fusion_node Started.\033[0m");
    // ...
    psr_doppler_fusion psr_doppler_fusion;
    ros::spin();
    return 0;
}
