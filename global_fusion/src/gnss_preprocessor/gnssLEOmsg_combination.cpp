/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Yixin GAO (yixin.gao@connect.polyu.hk)
 * 
 * Main fucntions: Combine GNSS and LEO Doppler element and Pseudorange, carrier phase raw msg into one topic
 * input: pseudorange, Doppler element from GPS/BeiDou/LEO.
 * output: GNSS_LEO_PsrCarRov1 topic and GNSS_LEO_Dopp_Array 
 * Date: 2025/01/19
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
#include <nlosExclusion/GNSS_Raw.h>
#include <nlosExclusion/GNSS_Raw_Array.h>
#include <nlosExclusion/LEO_dopp_Array.h>
#include <nlosExclusion/LEO_dopp.h>
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
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/NavSatFix.h>

#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX

// rtklib
#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"

class gnssLEOmsg_combination
{
    ros::NodeHandle nh;

    // ros::Publisher pub_WLSENU, pub_FGOENU, pub_global_path, pubStationGNSSRaw;
    ros::Publisher pubGNSSRaw, pubDopplerRaw;
    /* ros subscriber */
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map;
    std::map<double, nlosExclusion::LEO_dopp_Array> doppler_element_map;

    GNSS_Tools m_GNSS_Tools; // utilities

    /* subscriber */
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> gnss_raw_array_sub1;
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::LEO_dopp_Array>> doppler_element_sub1;
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> gnss_raw_array_sub2;
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::LEO_dopp_Array>> doppler_element_sub2;
    // std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> station_gnss_raw_array_sub;

    typedef message_filters::sync_policies::ApproximateTime<nlosExclusion::GNSS_Raw_Array, nlosExclusion::LEO_dopp_Array, nlosExclusion::GNSS_Raw_Array, nlosExclusion::LEO_dopp_Array> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> syncdopplerElement2GNSSRaw;
    // std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw;

    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux;

    int gnss_frame = 0;
    int curGPSSec = 0;

    bool hasNewData = false;

    /* thread for data processing */
    // std::thread publishGNSSTopicThread;

    /* parameters to be get */
    int startGPSSec = 0;
    int endGPSSec   = 456900; 

    Eigen::Matrix<double, 3,1> ENU_ref;


    bool finishGNSSReader = false;

public:
    gnssLEOmsg_combination(ros::NodeHandle& nh)
    {      
        
        /* subscriber of three topics  */
        gnss_raw_array_sub1.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        gnss_raw_array_sub2.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/LEOPsrCarRov1", 10000));
        doppler_element_sub1.reset(new message_filters::Subscriber<nlosExclusion::LEO_dopp_Array>(nh, "/gnss_preprocessor_node/GNSS_Dopp_Array", 10000));
        doppler_element_sub2.reset(new message_filters::Subscriber<nlosExclusion::LEO_dopp_Array>(nh, "/gnss_preprocessor_node/LEO_Dopp_Array", 10000));


        syncdopplerElement2GNSSRaw.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10000), *gnss_raw_array_sub1, *doppler_element_sub1, *gnss_raw_array_sub2, *doppler_element_sub2));
        syncdopplerElement2GNSSRaw->registerCallback(boost::bind(&gnssLEOmsg_combination::gnssraw_doppler_msg_callback,this, _1, _2,_3,_4));
        /* subscribe to span-cpt message */
        //span_BP_sub =nh.subscribe("/novatel_data/inspvax", 500, &rosbag_generator::span_bp_callback, this);

        /* publish the path from factor graph optimization */
       //pub_global_path = nh.advertise<nav_msgs::Path>("/FGOGlobalPath", 100); // 

        /* publish the raw gnss data*/
        pubGNSSRaw = nh.advertise<nlosExclusion::GNSS_Raw_Array>("/gnssLEOmsg_combination_node/GNSS_LEO_PsrCarRov", 100); //

        /* publish the raw gnss data from station*/ 
        //pubStationGNSSRaw = nh.advertise<nlosExclusion::GNSS_Raw_Array>("/rosbag_generator_node/GNSSPsrCarStation1", 100); // 

        /* publish the Doppler gnss data*/ 
        pubDopplerRaw = nh.advertise<nlosExclusion::LEO_dopp_Array>("/gnssLEOmsg_combination_node/GNSS_LEO_Dopp_Array", 100); //    
        
        /* reference point for ENU calculation */
        ENU_ref<< ref_lon, ref_lat, ref_alt;

        /* get parameters */
        nh.param("startGPSSec",   startGPSSec, 2);
        nh.param("endGPSSec",     endGPSSec, 2);
        // nh.param("soltype",soltype, 2);

    }

    /* check the valid epoch based on gps time span*/
    bool checkValidEpoch(double gps_sec)
    {
        if((gps_sec >= start_gps_sec) && (gps_sec <=end_gps_sec))
        {
            return true;
        }
        else return false;
    }

   
    /**
   * @brief gnss raw msg and doppler_element msg callback
   * @param gnss raw msg and doppler_element msg
   * @return void
   @ 
   */
   void gnssraw_doppler_msg_callback(const boost::shared_ptr<const nlosExclusion::GNSS_Raw_Array>& gnss_msg1, const boost::shared_ptr<const nlosExclusion::LEO_dopp_Array>& doppler_element_msg1,
                                      const boost::shared_ptr<const nlosExclusion::GNSS_Raw_Array>& gnss_msg2, const boost::shared_ptr<const nlosExclusion::LEO_dopp_Array>& doppler_element_msg2)
    {
        std::lock_guard<std::mutex> lock(m_gnss_raw_mux);

        
        if(finishGNSSReader) 
        {
            //m_gnss_raw_mux.unlock();
            return;
        }

        hasNewData = true;
        gnss_frame++;
        double time0 = gnss_msg1->GNSS_Raws[0].GNSS_time;
        // double time1 = station_gnss_msg->GNSS_Raws[0].GNSS_time;
        double time_frame = doppler_element_msg1->LEO_Dopps[0].GNSS_time;
        double time1 = gnss_msg2->GNSS_Raws[0].GNSS_time;
        double time3 = doppler_element_msg2->LEO_Dopps[0].GNSS_time;

        curGPSSec = gnss_msg1->GNSS_Raws[0].GNSS_time;
        
        std::cout<<"gnss time1: " <<time0 <<"; dopp time1: "<<time_frame<<"gnss time 2: "<<time1<<"; dopp time2: "<<time3<<std::endl; 
        std::cout<<"gnss_msg size:"<<gnss_msg1->GNSS_Raws.size()<<", "<<gnss_msg2->GNSS_Raws.size()<<std::endl;
        std::cout<<"dopp_element msg size:"<<doppler_element_msg1->LEO_Dopps.size()<<", "<<doppler_element_msg2->LEO_Dopps.size()<<std::endl;
        // std::cout<<"doppler time_frame " <<time_frame <<std::endl;
        // std::cout<<"station time1 " <<time1 <<std::endl;

        /* Combine LEO and GNSS doppler msg and Psr msg to one topic*/
        nlosExclusion::GNSS_Raw_Array gnss_msg = *gnss_msg1;
        gnss_msg.GNSS_Raws.insert(gnss_msg.GNSS_Raws.end(), gnss_msg2->GNSS_Raws.begin(), gnss_msg2->GNSS_Raws.end());
        for (int i=0; i<gnss_msg.GNSS_Raws.size();i++)
        {
            gnss_msg.GNSS_Raws[i].total_sv = gnss_msg.GNSS_Raws.size();
        }
        std::cout<<" GNSS_LEO Raw msg size is" << gnss_msg.GNSS_Raws.size() << std::endl;
        gnss_msg.header.stamp = gnss_msg1->header.stamp;
        nlosExclusion::LEO_dopp_Array doppler_element_msg = *doppler_element_msg1;
        doppler_element_msg.LEO_Dopps.insert(doppler_element_msg.LEO_Dopps.end(),doppler_element_msg2->LEO_Dopps.begin(),doppler_element_msg2->LEO_Dopps.end());
        for (int i=0; i<doppler_element_msg.LEO_Dopps.size();i++)
        {
            doppler_element_msg.LEO_Dopps[i].n = doppler_element_msg.LEO_Dopps.size();
        }
        std::cout<<" GNSS_LEO Doppler Element msg size is" << doppler_element_msg.LEO_Dopps.size() << std::endl;
        doppler_element_msg.header.stamp = doppler_element_msg1->header.stamp;
        /* save the data */
        gnss_raw_map[curGPSSec] = gnss_msg;
        doppler_element_map[curGPSSec] = doppler_element_msg;
        pubGNSSRaw.publish(gnss_msg);
        pubDopplerRaw.publish(doppler_element_msg);
        if(curGPSSec>end_gps_sec)
        {
            finishGNSSReader = true;
            std::cout<< " you can play the bag file now!  " << doppler_element_map.size()<<std::endl;
        }
        
        /* release the lock */
        m_gnss_raw_mux.unlock();
    }

    ~gnssLEOmsg_combination()
    {
    }
   
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "gnssLEOmsg_combination_node"); 
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> gnssLEOmsg_combination_node Started.\033[0m"); 
    // ...
    system("rosbag record -O gnss_leo_data.bag /gnssLEOmsg_combination_node/GNSS_LEO_PsrCarRov1 /gnssLEOmsg_combination_node/GNSS_LEO_Dopp_Array &");
    gnssLEOmsg_combination gnssLEOmsg_combination_(nh);
    ros::spin();
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    // }
    
    return 0;
}
