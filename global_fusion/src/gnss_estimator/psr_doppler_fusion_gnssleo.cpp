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
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>

// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<Eigen/Core>

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
    ros::NodeHandle nh; // ROS句柄

    /* ros subscriber */
    ros::Publisher pub_WLSENU, pub_FGOENU, pub_global_path, pub_fgo_llh,pub_FGOECEF; // 发布WLSENU、FGOENU、全局路径、FGO经纬高和FGOECEF坐标
    std::map<double, nlosexclusion::GNSS_Raw_Array> gnss_raw_map;
    std::map<double, nav_msgs::Odometry> doppler_map; // 使用map存储GNSS原始数据和多普勒测速数据，按时间戳索引
    GNSS_Tools m_GNSS_Tools; // utilities
    /* subscriber */
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> gnss_raw_array_sub; // 订阅GNSS原始数据数组消息
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub; // 订阅多普勒测速消息
    std::unique_ptr<message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw; // 时间同步器，用于同步GNSS原始数据和多普勒数据
    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux;
    /* thread for data processing */
    std::thread optimizationThread;
    int gnss_frame = 0; // 计数器，用于跟踪处理的GNSS数据帧数。
    Eigen::Matrix<double, 3,1> ENU_ref; // 存储ENU坐标系的参考点（经度、纬度、高度）
    int slidingWindowSize = 10000000; // ? epoch measurements 150 100000 // 滑动窗口大小，用于因子图优化中的批量处理。
    bool hasNewData = false;
    /* latest state in ENU */
    Eigen::Matrix<double ,3,1> FGOENULatest;  // 存储最新的因子图优化结果在ENU坐标系中的位置。
    /* latest state in ECEF */
    Eigen::Matrix<double ,3,1> FGOECEFLatest; // 存储最新的因子图优化结果在ECEF坐标系中的位置。
    /* path in ENU */
    nav_msgs::Path fgo_path; // 存储因子图优化结果的轨迹路径
    /* log path */
    std::string gnssleo_fgo_path; // 存储日志文件路径，用于保存处理结果

private:
    // std::unique_ptr<factor_graph> factor_graph_ptr_; // factor graph ptr
    FactorGraph factor_graph;

    
    bool initializeFactorGraph(ceres::Solver::Options& options) 
    {
        /* initialize the factor graph size */
        factor_graph.setWindowSize(slidingWindowSize);

        /* set up ceres-solver options */
        factor_graph.setupSolverOptions(options);

        /* set up loss functions (Huber, Cauchy, NULL)*/
        factor_graph.setupLossFunction("Huber");
    }


public:
    psr_doppler_fusion()
    {
        /* setup gnssleo_fgo_path */
        ros::param::get("~gnssleo_fgo_path", gnssleo_fgo_path); // 尝试从ROS参数服务器获取~gnssleo_fgo_path参数值
        if (!ros::param::get("~gnssleo_fgo_path", gnssleo_fgo_path))
        {
            LOG(ERROR) << "gnssleo_fgo_path not set, using default path";
            gnssleo_fgo_path = "./GNSSLEO_FGO_trajectoryllh_psr_dop_fusion.csv"; // change to your path // 如果参数未设置，则使用默认路径，不过这里要改吗？
        }
        std::cout << "gnssleo_fgo_path-> "<< gnssleo_fgo_path<< std::endl; // 通过日志和标准输出显示最终使用的路径
        /* thread for factor graph optimization */
        optimizationThread = std::thread(&psr_doppler_fusion::solveOptimization, this);
        // 创建四个ROS发布者，用于发布不同格式的定位结果
        pub_WLSENU = nh.advertise<nav_msgs::Odometry>("WLSGoGPS_GNSSLEO", 100); // 发布加权最小二乘(WLS)解算的ENU坐标系结果
        pub_FGOENU = nh.advertise<nav_msgs::Odometry>("FGO_GNSSLEO", 100); //  发布因子图优化(FGO)的ENU坐标系结果
        pub_FGOECEF = nh.advertise<nav_msgs::Odometry>("PsrDopp_FGO_GNSSLEO_ECEF", 100); // 发布因子图优化的ECEF坐标系结果
        pub_fgo_llh = nh.advertise<sensor_msgs::NavSatFix>("fgo_llh_GNSSLEO", 100); // 发布因子图优化的经纬高(LLH)坐标结果

        //gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));// GNSS only
        //doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));// GNSS only
        // 订阅GNSS+LEO组合的伪距观测数据，主题为/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov
        gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov", 10000));// GNSS+LEO
        // 订阅多普勒速度数据，主题为/psr_spp_gnssleo_node/GNSS_LEO_DopVelRov
        doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/psr_spp_gnssleo_node/GNSS_LEO_DopVelRov", 10000));// GNSS+LEO
        // 使用ROS的TimeSynchronizer实现时间同步，确保GNSS和多普勒数据在时间上对齐
        syncdoppler2GNSSRaw.reset(new message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub, *doppler_sub, 10000));
        // 注册回调函数gnssraw_doppler_msg_callback，当同步数据到达时调用该函数处理
        syncdoppler2GNSSRaw->registerCallback(boost::bind(&psr_doppler_fusion::gnssraw_doppler_msg_callback,this, _1, _2));
        // 创建一个用于发布完整轨迹路径的发布者，主题为/GNSS_LEO_FGOGlobalPath。
        pub_global_path = nh.advertise<nav_msgs::Path>("/GNSS_LEO_FGOGlobalPath", 100); // 
        /* reference point for ENU calculation */ // 设置ENU(东-北-天)坐标系的参考点，使用全局变量ref_lon(经度)、ref_lat(纬度)和ref_alt(高度)作为参考坐标。
        ENU_ref<< ref_lon, ref_lat, ref_alt;

    }


    /**
	 * @brief perform factor graph optimization
	 * @param none
	 * @return none
	 */
    void solveOptimization()
    {
        /* clear data stream */
        factor_graph.clearDataStream();

        
        
        while(1)
        {
            m_gnss_raw_mux.lock();

            if(factor_graph.getDataStreamSize()>3 && hasNewData)
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
                factor_graph.getDataStreamSize();

                /* setup memory for state */
                factor_graph.setupStateMemory();

                /* initialize factor graph parameters */
                factor_graph.initializeFactorGraphParas();


                /* initialize the previous optimzied states */
                factor_graph.initializeOldGraph();

                /* initialize the newly added states */
                factor_graph.initializeNewlyAddedGraph();

                /* add parameter blocks */
                factor_graph.addParameterBlocksToGraph(problem);

                /* fix the first parameter block */
                factor_graph.fixFirstState(false, problem);

                /* add Doppler FACTORS to factor graph */
                factor_graph.addDopplerFactors(problem);

                /* add pseudorange FACTORS to factor graph */
                factor_graph.addPseudorangeFactors(problem);

                /* solve the factor graph */
                factor_graph.solveFactorGraph(problem, options, summary);

                /* save graph state to variables */
                factor_graph.saveGraphStateToVector();

                /* set reference point for ENU calculation */
                factor_graph.setupReferencePoint();

                /* get the latest state in ENU*/
                FGOENULatest = factor_graph.getLatestStateENU();

                /* get the latest state in ENU*/
                FGOECEFLatest = factor_graph.getLatestStateECEF();
                /* publish the latest state in ECEF */
                nav_msgs::Odometry odometry_ecef; // 创建了一个nav_msgs::Odometry类型的消息，用于发布接收机在ECEF（地心地固坐标系）坐标系下的位置。
                odometry_ecef.header.frame_id = "glio_leo";
                odometry_ecef.child_frame_id = "glio_leo"; // frame_id和child_frame_id都设置为"glio_leo"，表示这是GNSS/LEO融合定位结果
                odometry_ecef.pose.pose.position.x = FGOECEFLatest(0);
                odometry_ecef.pose.pose.position.y = FGOECEFLatest(1);
                odometry_ecef.pose.pose.position.z = FGOECEFLatest(2); // 将最新的ECEF坐标位置（存储在FGOECEFLatest向量中）赋值给Odometry消息的position字段。
                std::cout<< "FGOECEFLatest-> "<< std::endl<< FGOECEFLatest<< std::endl;
                pub_FGOECEF.publish(odometry_ecef); // 后通过pub_FGOECEF发布这个Odometry消息。这使得其他ROS节点可以订阅并使用这个定位结果

                /* publish the lastest state in factor graph */
                factor_graph.printLatestStateENU();

                /* publish the path from FGO */
                fgo_path = factor_graph.getPathENU(fgo_path);
                pub_global_path.publish(fgo_path);

                /* remove the data outside sliding window */
                factor_graph.removeStatesOutsideSlidingWindow();

                /* log result */
                factor_graph.logResults(gnssleo_fgo_path);

                /* free memory (done when finish all the data) */
                // factor_graph.freeStateMemory();

                hasNewData = false;

                /** */
                std::cout << "OptTime-> "<< OptTime.toc()<< std::endl;
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
    void gnssraw_doppler_msg_callback(const nlosexclusion::GNSS_Raw_ArrayConstPtr& gnss_msg, const nav_msgs::OdometryConstPtr& doppler_msg)
    {
        m_gnss_raw_mux.lock();
        hasNewData = true;
        gnss_frame++;
        double time_frame = doppler_msg->pose.pose.position.x;
        /* save the  */
        if(checkValidEpoch(time_frame) && m_GNSS_Tools.checkRepeating(*gnss_msg))
        {
            if(gnss_msg->GNSS_Raws.size())
            {
                doppler_map[time_frame] = *doppler_msg;
                gnss_raw_map[time_frame] = *gnss_msg;

                factor_graph.input_gnss_raw_data(*gnss_msg, time_frame);
                factor_graph.input_doppler_data(*doppler_msg, time_frame);

                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                        m_GNSS_Tools.getAllPositions(*gnss_msg),
                                        m_GNSS_Tools.getAllMeasurements(*gnss_msg),
                                        *gnss_msg, "WLS");
                Eigen::Matrix<double ,3,1> WLSENU;
                WLSENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
                LOG(INFO) << "WLSENU -> "<< std::endl << WLSENU;
                LOG(INFO) << "doppler_map.size() -> "<< std::endl << doppler_map.size();

                nav_msgs::Odometry odometry;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "map";
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
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "psr_doppler_fusion_gnssleo_node"); 
    ROS_INFO("\033[1;32m----> psr_doppler_fusion_gnssleo_node Started.\033[0m"); 
    // ...
    psr_doppler_fusion psr_doppler_fusion;
    ros::spin();
    return 0;
}