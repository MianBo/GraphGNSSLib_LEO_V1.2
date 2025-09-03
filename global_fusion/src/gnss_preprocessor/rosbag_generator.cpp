/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 *
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 *
 * Main fucntions: add the raw GNSS data into the rosbag file by alligning the rostopic from SPAN-CPT.
 * input: rosbag, rinex file gnss raw measurements.
 * output: rosbag with raw gnss data
 * Date: 2020/11/28
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

#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX

// rtklib
#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"

class rosbag_generator
{
    // ros::NodeHandle nh;
    /* ros subscriber */
    // 加权最小二乘估计的ENU坐标，因子图优化的ENU坐标，全局路径，原始GNSS数据，基站GNSS原始数据，多普勒速度数据
    ros::Publisher pub_WLSENU, pub_FGOENU, pub_global_path, pubGNSSRaw, pubStationGNSSRaw, pubDopplerVel;

    std::map<int, nlosexclusion::GNSS_Raw_Array> gnss_raw_map;         // 存储GNSS原始数据
    std::map<int, nav_msgs::Odometry> doppler_map;                     // 值是里程计数据，用于存储多普勒测量数据
    std::map<int, nlosexclusion::GNSS_Raw_Array> station_gnss_raw_map; // 存储基站GNSS原始数据

    GNSS_Tools m_GNSS_Tools; // GNSS工具类实例，在/include/gnss_tools.h中被定义

    /* subscriber */
    // 指向GNSS_Raw_Array和Odometry类型的订阅者们。用于订阅移动站/基站的GNSS原始数据、多普勒速度数据和时间同步指针
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> gnss_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub;
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> station_gnss_raw_array_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nlosexclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw;

    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux; // 互斥锁，保护多线程环境下的GNSS原始数据访问安全

    int gnss_frame = 0; // 待处理的GNSS数据帧数
    int curGPSSec = 0;  // 当前的GPS秒数

    bool hasNewData = false; // 标记是否有新数据到达

    /* thread for data processing */
    std::thread publishGNSSTopicThread; // 处理数据发布任务的线程

    /* parameters to be get */
    int startGPSSec = 0; // 存储开始处理的GPS秒数
    int endGPSSec = 0;   // 存储结束处理的GPS秒数，二者应该都是从参数服务器中获得的

    Eigen::Matrix<double, 3, 1> ENU_ref; // 存储ENU坐标系的参考点

    ros::Subscriber span_BP_sub; // 声明ROS订阅者，用于定于SPAN-PNT设备的BESTPOS位置消息

    bool finishGNSSReader = false;

public:
    rosbag_generator(ros::NodeHandle &nh)
    {
        /* thread for factor graph optimization */
        // publishGNSSTopicThread = std::thread(&rosbag_generator::GNSSDataPublisherFun, this);

        /* publisher */
        // pub_WLSENU = nh.advertise<nav_msgs::Odometry>("WLSGoGPS", 100); //

        /* subscriber of three topics  */
        // 定于移动站/基站的GNSS的原始数据和多普勒的速度数据，主题见代码。订阅者队列大小都设置微100000
        gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));
        /* GNSS measurements from station (reference end) */
        station_gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarStation1", 10000)); // measurements from station
        // 创建时间同步器，对三个订阅者数据进行时间同步，同步队列大小为32，并注册回调函数，当三个数据都到达时会调用回调函数
        syncdoppler2GNSSRaw.reset(new message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nlosexclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub, *station_gnss_raw_array_sub, *doppler_sub, 32));
        syncdoppler2GNSSRaw->registerCallback(boost::bind(&rosbag_generator::gnssraw_doppler_msg_callback, this, _1, _2, _3));
        /* subscribe to span-cpt message */
        span_BP_sub = nh.subscribe("/novatel_data/inspvax", 500, &rosbag_generator::span_bp_callback, this); // 订阅SPAN-CPT设备的位置信息，并注册回调函数
        /* publish the path from factor graph optimization */
        pub_global_path = nh.advertise<nav_msgs::Path>("/FGOGlobalPath", 100); // 创建路径发布者，用于发布FGO得到的全局路径
        /* publish the raw gnss data*/
        pubGNSSRaw = nh.advertise<nlosexclusion::GNSS_Raw_Array>("/rosbag_generator_node/GNSSPsrCarRov1", 100); // 发布移动站的GNSS数据
        /* publish the raw gnss data from station*/
        pubStationGNSSRaw = nh.advertise<nlosexclusion::GNSS_Raw_Array>("/rosbag_generator_node/GNSSPsrCarStation1", 100); // 发布基站的GNSS数据
        /* publish the Doppler gnss data from station*/
        pubDopplerVel = nh.advertise<nav_msgs::Odometry>("/rosbag_generator_node/GNSSDopVelRov1", 100); // 发布多普勒速度数据
        /* reference point for ENU calculation */
        ENU_ref << ref_lon, ref_lat, ref_alt; // 设置ENU坐标系参考点，使用RTKLib中定义的参考经纬度和高度
        /* get parameters */
        nh.param("startGPSSec", startGPSSec, 2); // 从参数服务器中获取相关参数
        nh.param("endGPSSec", endGPSSec, 2);
        // nh.param("soltype",soltype, 2); // 原本这个代码是用来获取解算类型参数的
    }

    /* check the valid epoch based on gps time span*/
    bool checkValidEpoch(double gps_sec)
    {
        if ((gps_sec >= start_gps_sec) && (gps_sec <= end_gps_sec))
        {
            return true;
        }
        else
            return false;
    }

    /**
    * @brief span_cpt callback
    * @param span_cpt bestpos msg
    * @return void
    @
    */
    void span_bp_callback(const novatel_msgs::INSPVAXConstPtr &fix_msg) // SPAN-CPT设备位置信息的回调函数，接收到/novatel_data/inspvax主题的消息时会被调用
    {
        int gpsSec = fix_msg->header.gps_week_seconds;
        gpsSec = gpsSec / 1000; // 从消息头中提取GPS周秒数并转换为秒，除以1000是将毫秒转为秒
        std::cout << "GPS seconds from Rosbag" << gpsSec << "\n";
        if (finishGNSSReader) // GNSS数据读取完毕后finishGNSSReader会设置为true，表示可以开始发布数据
        {
            std::map<int, nlosexclusion::GNSS_Raw_Array>::iterator GNSSiter_pr;
            GNSSiter_pr = gnss_raw_map.begin();
            int length = gnss_raw_map.size(); // 声明迭代器用于遍历GNSS_raw_map，不过这部分好像没用到
            /* find rover gnss data and publish */
            nlosexclusion::GNSS_Raw_Array roverGNSSRawData; // 存储找到的数据
            GNSSiter_pr = gnss_raw_map.find(gpsSec);        // 使用gpsSec作为键在gnss_raw_map中查找数据
            if (GNSSiter_pr != gnss_raw_map.end())
            {
                roverGNSSRawData = GNSSiter_pr->second;
                pubGNSSRaw.publish(roverGNSSRawData);
            } // 如果找到了对应的数据就获取数据并发布
            /* find station gnss data and publish */
            nlosexclusion::GNSS_Raw_Array stationGNSSRawData; // 存储基站数据
            GNSSiter_pr = station_gnss_raw_map.find(gpsSec);
            if (GNSSiter_pr != station_gnss_raw_map.end())
            {
                stationGNSSRawData = GNSSiter_pr->second;
                pubStationGNSSRaw.publish(stationGNSSRawData);
            } // 同上
            /* find rover doppler vel data and publish */
            std::map<int, nav_msgs::Odometry>::iterator gnssDopplerVelIter; // 声明迭代器用于遍历doppler_map
            nav_msgs::Odometry roverDopplerVelData;                         // 存储多普勒数据
            gnssDopplerVelIter = doppler_map.find(gpsSec);
            if (gnssDopplerVelIter != doppler_map.end())
            {
                roverDopplerVelData = gnssDopplerVelIter->second;
                pubDopplerVel.publish(roverDopplerVelData);
            } // 同上
        }
    }

    /**
   * @brief gnss raw msg and doppler msg callback
   * @param gnss raw msg and doppler msg
   * @return void
   @
   */
    void gnssraw_doppler_msg_callback(const nlosexclusion::GNSS_Raw_ArrayConstPtr &gnss_msg, const nlosexclusion::GNSS_Raw_ArrayConstPtr &station_gnss_msg, const nav_msgs::OdometryConstPtr &doppler_msg)
    {
        m_gnss_raw_mux.lock(); // 加锁保护，防止多线程冲突
        if (finishGNSSReader)  // 如果GNSS数据已完成，则解锁，不再处理新数据
        {
            m_gnss_raw_mux.unlock();
            return;
        }
        hasNewData = true; // 有新数据到达
        gnss_frame++;      // 则GNSS数据帧数递增（帧计数器）
        double time0 = gnss_msg->GNSS_Raws[0].GNSS_time;
        double time1 = station_gnss_msg->GNSS_Raws[0].GNSS_time;
        double time_frame = doppler_msg->pose.pose.position.x; // GNSS数据时间/基站GNSS数据时间/多普勒消息（存在position.x字段）的时间提取
        curGPSSec = gnss_msg->GNSS_Raws[0].GNSS_time;          // 更新当前的GPS秒数
        if (checkValidEpoch(time_frame) && m_GNSS_Tools.checkRepeating(*gnss_msg)) // 如果时间帧有效且GNSS数据不重复
        {
            if (gnss_msg->GNSS_Raws.size()) // 如果GNSS数据不为空
            {   // 将三种数据按时间帧存入对应的map中
                doppler_map[int(time_frame)] = *doppler_msg;
                gnss_raw_map[int(time_frame)] = *gnss_msg;
                station_gnss_raw_map[int(time_frame)] = *station_gnss_msg;
                // 使用加权最小二乘法WLS计算GNSS位置解
                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                    m_GNSS_Tools.getAllPositions(*gnss_msg), // 所有卫星位置
                    m_GNSS_Tools.getAllMeasurements(*gnss_msg), // 所有观测值
                    *gnss_msg, "WLS");
                Eigen::Matrix<double, 3, 1> WLSENU;
                WLSENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF); // ECEF->ENU
                LOG(INFO) << "WLSENU -> " << std::endl
                          << WLSENU;
            }
        }
        if (curGPSSec > end_gps_sec) // 如果已超过预设的结束时间
        {
            finishGNSSReader = true; // 完成读取
            std::cout << " you can play the bag file now!  " << doppler_map.size() << std::endl;
        }
        /* release the lock */
        m_gnss_raw_mux.unlock(); // 解锁互斥量，释放资源
    }

    ~rosbag_generator() // 析构函数
    {
    }
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;                             // output to console
    google::InitGoogleLogging(argv[0]);                // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags
    ros::init(argc, argv, "rosbag_generator_node");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> rosbag_generator_node Started.\033[0m");
    // ...
    rosbag_generator rosbag_generator_(nh);
    ros::spin();
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    // }

    return 0;
}
