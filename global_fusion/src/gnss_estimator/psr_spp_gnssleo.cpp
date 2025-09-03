/*******************************************************
 * Copyright (C) 2025,Trustworthy Autonomous Systems (TAS) Laboratory , Hong Kong Polytechnic University
 *
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Yixin GAO (yixing.gao@connect.polyu.hk)
 * Function: subscribe the raw measurements, perform snap shot GNSS single point positioning based pseudorange measurements. The theoretical output should be similar with the conventional weighted least squares (WLS).
 * Date: 2025/01/13
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
/* Reference from NovAtel GNSS/INS */
#include <novatel_msgs/INSPVAX.h>
// #include <novatel_oem7_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h>
// #include <novatel_oem7_msgs/BESTPOS.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"
#include <nlosexclusion/GNSS_Raw_Array.h>
#include <nlosexclusion/GNSS_Raw.h>
#include <nlosexclusion/LEO_dopp.h>
#include <nlosexclusion/LEO_dopp_Array.h>

#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <iostream>

#include <nav_msgs/Path.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "ceres/dynamic_autodiff_cost_function.h"

#include "../tic_toc.h"

// rtklib /
#include <stdarg.h>
#include "../RTKLIB/src/rtklib.h"

DEFINE_double(corridor_length, 30.0, "Length of the corridor that the robot is "
                                     "travelling down.");
DEFINE_double(pose_separation, 0.5, "The distance that the robot traverses "
                                    "between successive odometry updates.");
DEFINE_double(odometry_stddev, 0.1, "The standard deviation of "
                                    "odometry error of the robot.");
DEFINE_double(range_stddev, 0.01, "The standard deviation of range readings of "
                                  "the robot.");
// The stride length of the dynamic_autodiff_cost_function evaluator.
static const int kStride = 10;

static const char rcsid[] = "$Id:$";

/* constants -----------------------------------------------------------------*/
#define NFREQ 3 /* number of carrier frequencies */
#define SQR(x) ((x) * (x))

#define NX (4 + 3) /* # of estimated parameters */

#define MAXITR 10          /* max number of iteration for point pos */
#define ERR_ION 5.0        /* ionospheric delay std (m) */
#define ERR_TROP 3.0       /* tropspheric delay std (m) */
#define ERR_SAAS 0.3       /* saastamoinen model error std (m) */
#define ERR_BRDCI 0.5      /* broadcast iono model error factor */
#define ERR_CBIAS 0.3      /* code bias error std (m) */
#define REL_HUMI 0.7       /* relative humidity for saastamoinen model */
#define D2R (M_PI / 180.0) /* deg to rad */
#define R2D (180.0 / M_PI) /* rad to deg */

GNSS_Tools m_GNSS_Tools; // utilities

class gnssSinglePointPositioning
{
    ros::NodeHandle nh;
    std::string frame_id = "map";
    // ros subscriber
    // std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> gnss_leo_raw_array_sub;
    ros::Subscriber gnss_leo_raw_array_sub;
    // std::unique_ptr<message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nlosexclusion::LEO_dopp_Array>> syncdoppler2LEOPsrRaw;

    ros::Publisher pub_WLS = nh.advertise<nav_msgs::Odometry>("/psr_spp_gnssleo_node/WLS_spp_GNSS_LEO", 100);
    ros::Publisher pub_WLS_ECEF = nh.advertise<nav_msgs::Odometry>("/psr_spp_gnssleo_node/WLS_spp_GNSS_LEO_ECEF", 100);             //
    ros::Publisher pub_velocity_from_doppler = nh.advertise<nav_msgs::Odometry>("/psr_spp_gnssleo_node/GNSS_LEO_DopVelRov", 20000); // r;
    // ros::Publisher pub_velocity_from_doppler = nh.advertise<nav_msgs::Odometry>("/psr_spp_gnssleo_node/LEODopVelRov", 20000); //r;

    // std::queue<nlosexclusion::GNSS_Raw_ArrayConstPtr> gnss_leo_raw_buf;
    std::map<double, nlosexclusion::GNSS_Raw_Array> gnss_leo_raw_buf;
    std::map<double, nlosexclusion::GNSS_Raw_Array> gnss_leo_raw_map;
    // std::queue<nlosexclusion::LEO_dopp_ArrayConstPtr> gnss_leo_doppler_buf;
    // std::map<double, nlosexclusion::LEO_dopp_Array> gnss_leo_doppler_map;

    std::mutex m_gnss_raw_mux;
    std::mutex optimize_mux;
    std::thread optimizationThread;

    GNSS_Tools m_GNSS_Tools; // utilities

    nav_msgs::Path wls_path;

    int gnss_raw_frame = 0;
    std::vector<double> rover_x; // obtained from solveppmization
    std::vector<double> rover_y;
    std::vector<double> rover_z;
    int gnss_frame = 0;

    Eigen::Matrix<double, 3, 1> ENULlhRef;

    bool hasNewData = false;
    /* 新增部分 */
    ros::Time GPSTimeToROSTime(double gps_time)
    {
        int gps_week = static_cast<int>(gps_time / 604800);
        double gps_seconds = gps_time - gps_week * 604800; // 提取GPS周数和周内秒数
        const ros::Time gps_epoch(315964800, 0);           // 315964800s 是从Unix时间起点1970/1/1到GPS时间起点1980/1/6的秒数

        // Calculate the total seconds in ros to the given GPS time
        double total_seconds = gps_week * 604800 + gps_seconds + gps_epoch.toSec(); // 计算总的Unix秒数

        // Transform the total seconds to seconds and nanoseconds
        int64_t sec = static_cast<int64_t>(total_seconds);
        int64_t nsec = static_cast<int64_t>((total_seconds - sec) * 1e9); // 将总秒数分为秒和纳秒，用来构造ROS时间对象

        // Create a ROS time object
        ros::Time ros_time(sec, nsec);

        return ros_time;
    }
    /* 新增结束 */
public:
    // from Weisong
    // var _ variance 1/weight_matrix
    struct pseudorangeFactor
    {
        pseudorangeFactor(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var)
            : sat_sys(sat_sys), s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange), var(var) {}

        template <typename T>
        bool operator()(const T *state, T *residuals) const
        {
            T est_pseudorange;
            T delta_x = pow((state[0] - s_g_x), 2);
            T delta_y = pow((state[1] - s_g_y), 2);
            T delta_z = pow((state[2] - s_g_z), 2);
            est_pseudorange = sqrt(delta_x + delta_y + delta_z);
            double OMGE_ = 7.2921151467E-5;
            double CLIGHT_ = 299792458.0;
            est_pseudorange = est_pseudorange + OMGE_ * (s_g_x * state[1] - s_g_y * state[0]) / CLIGHT_;
            if (sat_sys == "GPS")
            {
                est_pseudorange = est_pseudorange - state[3]; // GPS修正
            }
            // add by Yixin
            else if (sat_sys == "LEO") // 新增部分，LEO钟差修正（不修）
            {
                est_pseudorange = est_pseudorange;
            }
            else
            {
                est_pseudorange = est_pseudorange - state[4]; // 北斗修正
            }
            residuals[0] = (est_pseudorange - T(pseudorange)) / T(var); // 输出残差值
            return true;
        }
        double s_g_x, s_g_y, s_g_z, pseudorange, var;
        std::string sat_sys; // satellite system
    };

    // from Weisong

    struct pseudorangeConstraint
    {
        typedef ceres::DynamicAutoDiffCostFunction<pseudorangeConstraint, kStride>
            PseudorangeCostFunction;
        pseudorangeConstraint(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var, int keyIndex)
            : sat_sys(sat_sys), s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange), var(var), keyIndex(keyIndex) {}

        template <typename T>
        bool operator()(T const *const *state, T *residuals) const
        {

            T est_pseudorange;
            T delta_x = pow((state[keyIndex][0] - s_g_x), 2);
            T delta_y = pow((state[keyIndex][1] - s_g_y), 2);
            T delta_z = pow((state[keyIndex][2] - s_g_z), 2);
            est_pseudorange = sqrt(delta_x + delta_y + delta_z);

            double OMGE_ = 7.2921151467E-5;
            double CLIGHT_ = 299792458.0;
            est_pseudorange = est_pseudorange + OMGE_ * (s_g_x * state[keyIndex][1] - s_g_y * state[keyIndex][0]) / CLIGHT_;

            if (sat_sys == "GPS")
            {
                est_pseudorange = est_pseudorange - state[keyIndex][3];
            }
            // add by Yixin
            else if (sat_sys == "LEO")
            {
                est_pseudorange = est_pseudorange;
            }
            else
            {
                est_pseudorange = est_pseudorange - state[keyIndex][4];
            }
            residuals[0] = (est_pseudorange - T(pseudorange)) / T(var);
            return true;
        }

        // Factory method to create a CostFunction from a pseudorangeConstraint to
        // conveniently add to a ceres problem.
        static PseudorangeCostFunction *Create(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var, int keyIndex)
        {

            pseudorangeConstraint *constraint = new pseudorangeConstraint(
                sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, var, keyIndex);

            PseudorangeCostFunction *cost_function = new PseudorangeCostFunction(constraint);
            std::cout << "keyIndex-> " << keyIndex << std::endl;
            for (int i = 0; i < (keyIndex + 1); i++)
            {
                cost_function->AddParameterBlock(5);
            }

            cost_function->SetNumResiduals(1);
            return (cost_function);
        }

        double s_g_x, s_g_y, s_g_z, pseudorange, var;
        int keyIndex;
        std::string sat_sys; // satellite system
    };

public:
    gnssSinglePointPositioning()
    {

        // gnss_leo_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov", 10000));
        // gnss_leo_doppler_sub.reset(new message_filters::Subscriber<nlosexclusion::LEO_dopp_Array>(nh, "/gnss_leo_msg_combination_node/GNSS_LEO_Dopp_Array", 10000));
        // syncdoppler2LEOPsrRaw.reset(new message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nlosexclusion::LEO_dopp_Array>(*gnss_leo_raw_array_sub, *gnss_leo_doppler_sub, 10000));
        // syncdoppler2LEOPsrRaw->registerCallback(boost::bind(&gnssSinglePointPositioning::psr_doppler_msg_callback,this, _1, _2));
        gnss_leo_raw_array_sub = nh.subscribe("/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov", 500, &gnssSinglePointPositioning::psr_doppler_msg_callback, this);
        // gnss_leo_raw_array_sub = nh.subscribe("/leo_raw_publisher_node/LEOPsrCarRov1", 500, &gnssSinglePointPositioning::psr_doppler_msg_callback, this);
        // gnss_leo_raw_array_sub = nh.subscribe("/gnss_preprocessor_node/GNSSPsrCarRov1", 500, &gnssSinglePointPositioning::psr_doppler_msg_callback, this);
        // gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        optimizationThread = std::thread(&gnssSinglePointPositioning::solvePptimization, this);

        // ref position
        ENULlhRef.resize(3, 1);
        ENULlhRef << ref_lon, ref_lat, ref_alt;
    }

    ~gnssSinglePointPositioning()
    {
        optimizationThread.detach();
    }

    /**
    * @brief pseudorange and doppler msg callback
    * @param gnss_leo_psr_msg and gnss_leo_dopp_msg
    * @return void
    @
    */

    void psr_doppler_msg_callback(const nlosexclusion::GNSS_Raw_ArrayConstPtr &gnss_leo_psr_msg)
    {
        std::lock_guard<std::mutex> lock(m_gnss_raw_mux);
        gnss_frame++;
        if (gnss_leo_psr_msg->GNSS_Raws.size())
        {
            hasNewData = true;
            gnss_leo_raw_buf[gnss_frame] = *gnss_leo_psr_msg;
            // gnss_leo_raw_map[gnss_frame] = *gnss_leo_psr_msg;
        }
    }

    /**
     * @brief Compute receiver velocity using LEO satellites doppler shift and publish the result.
     *
     * This function computes the receiver's position and velocity based on the provided Doppler shifts,
     * satellite positions, velocities, and other parameters, and publishes the result as a nav_msgs::Odometry message.
     *
     * @param doppler_shifts An array of Doppler shifts for each satellite, in Hz.
     * @param lambdas An array of wavelengths for each satellite, in meters.
     * @param n The number of observation data points.
     * @param rs An array of satellite positions and velocities in ECEF coordinates, in meters and meters per second.
     *           The array length should be 6 * n, with each satellite's data in the format [x, y, z, vx, vy, vz].
     * @param dts An array of satellite clock biases and drifts, in seconds.
     *            The array length should be 2 * n, with each satellite's data in the format [dt, dtr].
     * @param rr The receiver's position in ECEF coordinates, in meters. The array length should be 3.
     * @param azel An array of azimuth and elevation angles for each satellite, in radians.
     *             The array length should be 2 * n, with each satellite's data in the format [azimuth, elevation].
     * @param vsat An array of flags indicating whether each satellite is valid. The array length should be n.
     * @return int Status of the computation (1: success, 0: failure).
     */
    int pntpos_GNSSLEO(const double *prn, double current_week, double current_tow, const double *doppler_shifts, const double *lambdas,
                       int n, const double *rs, double *dts, double *rr,
                       double *azel, int *vsat) // PRN，当前GPS周/周内秒，多普勒频移，波长，观测数据点数，卫星位置和速度，卫星钟差和漂移，接收机位置，卫星方位角和仰角，卫星有效标志
    {
        double dop_res[n] = {0};
        double velocity[3] = {0};
        double ENU[3] = {0};
        // Estimate receiver velocity using Doppler shifts for GNSS-LEO satellites
        estvel_GNSSLEO(prn, doppler_shifts, lambdas, n, rs, dts, rr, azel, vsat, velocity, dop_res); // 调用函数基于多普勒频移计算接收机速度（最小二乘）
                                                                                                     // Convert ECEF position to ENU position
        // Eigen::Matrix<double, 3, 1> ENU_ref;
        // ENU_ref << ref_lon, ref_lat, ref_alt;
        // Eigen::Matrix<double, 3, 1> ENU;
        // Eigen::Matrix<double, 3, 1> ECEF;
        // ECEF << rr[0], rr[1], rr[2];
        // ENU = m_GNSS_Tools.ecef2enu(ENU_ref, ECEF);
        // Publish the result as nav_msgs::Odometry // 构造ROS消息，将header设置为GPS时间转换的ROs时间，并将fram_id都设置为map
        nav_msgs::Odometry odometry;
        odometry.header.frame_id = frame_id; //"map";
        odometry.child_frame_id = frame_id;  //"map";
        // add by Yixin
        odometry.header.stamp = GPSTimeToROSTime((current_week * 604800 + current_tow)); // need to change according to your week
        // temporary state of receiver position in ECEF
        Eigen::Matrix<double, 3, 1> state_matrix;
        state_matrix << rr[0], rr[1], rr[2];
        // std::cout << "state_matrix: " << state_matrix << std::endl;
        // Eigen::Matrix<double, 3, 1> temp_ENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state_matrix);
        // ENU[0] = temp_ENU(0);
        // ENU[1] = temp_ENU(1);
        // ENU[2] = temp_ENU(2);
        odometry.pose.pose.position.x = current_tow;
        odometry.pose.pose.position.y = 0;
        odometry.pose.pose.position.z = 0; // x坐标是GPS时间（周内秒），y和z坐标设置微0
        odometry.twist.twist.linear.x = velocity[0];
        odometry.twist.twist.linear.y = velocity[1];
        odometry.twist.twist.linear.z = velocity[2]; // 将estvel_GNSSLEO计算得到的接收机速度填入twist部分
        // Optionally, set the covariance of the twist (velocity) based on the Doppler residuals
        odometry.twist.covariance[0] = norm(dop_res, n); // 将多普勒残差的范数作为速度协方差的第一项，表示速度估计的不确定性
        pub_velocity_from_doppler.publish(odometry);     // 通过发布器发布构造好的里程计信息
        // saveOdometryToFile("${workspaceRoot}/src/global_fusion/dataset/odometry_data.csv", current_week, current_tow, rr, velocity);
        return 1; // Return success status
    }

    // add by Yixin
    /**
     * @brief Estimate receiver velocity using Doppler shifts for GNSS_LEO satellites.
     *
     * This function estimates the receiver's velocity based on the provided Doppler shifts,
     * satellite positions, velocities, and other parameters.
     *
     * @param doppler_shifts A vector of Doppler shifts for each satellite, in Hz.
     * @param lambdas A vector of wavelengths for each satellite, in meters.
     * @param n The number of observation data points.
     * @param rs An array of satellite positions and velocities in ECEF coordinates, in meters and meters per second.
     *           The array length should be 6 * n, with each satellite's data in the format [x, y, z, vx, vy, vz].
     * @param dts An array of satellite clock biases and drifts, in seconds.
     *            The array length should be 2 * n, with each satellite's data in the format [dt, ddt].
     * @param rr The receiver's position in ECEF coordinates, in meters. The array length should be 3.
     * @param azel An array of azimuth and elevation angles for each satellite, in radians.
     *             The array length should be 2 * n, with each satellite's data in the format [azimuth, elevation].
     * @param vsat An array of flags indicating whether each satellite is valid. The array length should be n.
     * @param dop_res An array to store the computed Doppler residuals. The array length should be n.
     * @param velocity An array to store the estimated receiver velocity. The array length should be 3.
     */
    int estvel_GNSSLEO(const double *prn, const double *doppler_shifts, const double *lambdas, int n, const double *rs, const double *dts,
                       double *rr, double *azel, int *vsat, double *velocity, double *dop_res) // 和上个函数差不多，最后一个dop_res是多普勒参差
    {
        double x[6] = {0}, dx[6], Q[36], *v, *H; // 状态向量，接收机速度和3个系统的钟漂；状态向量更新量；协方差矩阵；观测残差向量；设计矩阵
        int i, j, k, nv; // 循环变量和有效观测数量
        trace(3, "estvel_GNSSLEO  : n=%d\n", n); // RTKlib的日志函数，级别3
        v = mat(n, 1);
        H = mat(6, n); // 分配矩阵内存给残差向量v和设计矩阵H

        for (i = 0; i < MAXITR; i++) // 迭代10次
        {

            /* doppler residuals */
            // std::cout << "---------------------"<< i+1 <<" Iteration---------------------" << std::endl;
            nv = resdop_GNSSLEO(prn, doppler_shifts, lambdas, n, rs, dts, rr, x, azel, vsat, v, H); // 调用后面的函数计算当前状态下的多普勒残差
            if (nv < 4) // 观测值数量不足的时候返回错误
            {
                std::cerr << "Error: not enough valid satellites" << std::endl;
                return 0;
            }

            // if ((nv = resdop_GNSSLEO(doppler_shifts, lambdas, n, rs, dts, rr, x, azel, vsat, v, H)) < 4) {
            //     std::cerr << "Error: not enough valid satellites" << std::endl;
            //     return 0;
            // }
            // added by Yixin for matrix check
            if (0) // if(0)和注释没啥两样
            {
                printf("The Coefieient Matrix H is :\n");
                for (j = 0; j < 6; j++)
                {
                    for (k = 0; k < nv; k++)
                    {
                        printf("%.2f,", H[j + k * 6]);
                    }
                    printf("\n");
                }
            }
            /* least square estimation */
            if (lsq(H, v, 6, nv, dx, Q)) // 最小二乘估计，lsq函数在rtklib/src的rtklib.h里面，具体实现是rtkcmn.c代码中
                break;

            for (j = 0; j < 6; j++) // 更新状态向量
                x[j] += dx[j];

            if (norm(dx, 6) < 1E-6) // 计算更新量dx的范数，小于阈值即收敛，此时将接收机速度复制到输出参数中
            {
                for (i = 0; i < 3; i++)
                    velocity[i] = x[i];
                break;
            }
        }
        dop_res = v; // 函数结束，将残差向量v的值输出给dop_res

        // free(v);
        // free(H);
    }

    // add by Yixin
    /* doppler residuals for LEO obtained from csv ---------------------------------------------------
     * compute doppler residuals for LEO satellites
     * args   : double *doppler_shifts I   doppler shifts for each satellite (Hz)
     *          double *lambdas        I   wavelengths for each satellite (m)
     *          int    n                            I   number of observation data
     *          double *rs                          I   satellite positions and velocities (ECEF) (m|m/s)
     *          double *dts                         I   satellite clock biases and drifts (s)
     *          double *rr                          I   receiver position (ECEF) (m)
     *          double *x                           I   state vector (receiver velocity and clock bias)
     *          double *azel                        I   azimuth/elevation angles (rad)
     *          int    *vsat                        I   valid satellite flags
     *          double *v                           O   doppler residuals
     *          double *H                           O   design matrix
     * return : int                                 number of valid observations used in the computation
     * notes  : this function does not use obs and nav structures, but directly uses doppler shifts and lambdas
     */
    int resdop_GNSSLEO(const double *prn, const double *doppler_shifts, const double *lambdas, int n, const double *rs, const double *dts,
                       double *rr, double *x, double *azel, int *vsat, double *v, double *H) // 和上个函数区别不大，最后一个H是设计矩阵，它和多普勒残差v一起输出
    {
        double rate, pos[3], E[9], a[3], e[3], vs[3], cosel; // 距离变化率；接收机位置；ECEF->ENU坐标转换矩阵；视线向量(ENU/ECEF);卫星相对于接收机的速度；仰角余弦值
        int i, j, nv = 0;

        trace(3, "resdop_GNSSLEO  : n=%d\n", n);

        ecef2pos(rr, pos); // 将接收机ECEF->LLH
        xyz2enu(pos, E);   // ECEF -> NEU的坐标转换矩阵

        for (i = 0; i < n && i < MAXOBS; i++) // 最多处理MAXOBS颗卫星，这里没规定？查了查rtklib.h是64
        {
            double lam = lambdas[i]; // 获取当前卫星的波长
            // std::cout << "norm:" << norm(rs + 3 + i * 6, 3) << std::endl;
            if (doppler_shifts[i] == 0.0 || lam == 0.0 || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0) // 如果多普勒/波长为0，或卫星无效，卫星速度向量范数小于等于0则跳过
            {
                continue;
            }
            /* line-of-sight vector in ecef */
            cosel = cos(azel[1 + i * 2]); // 计算视线向量(ENU)
            a[0] = sin(azel[i * 2]) * cosel; // E
            a[1] = cos(azel[i * 2]) * cosel; // N
            a[2] = sin(azel[1 + i * 2]);     // U

            matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e); // 调用RTKLIB的函数将ENU转到ECEF上。TN表示第一个矩阵转置，第二个不转置。将3*3的矩阵E和3*1的矩阵a相乘，结果存在e中

            /* satellite velocity relative to receiver in ecef */
            for (int j = 0; j < 3; j++) // 计算卫星相对于接收机的速度
            {
                vs[j] = rs[j + 3 + i * 6] - x[j]; // 相对速度分量 = 第i颗卫星在ECEF中的速度分量 - 接收机速度分量
            }

            /* range rate with earth rotation correction */ // 计算考虑地球自转修正的距离变化率
            rate = dot(vs, e, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] - rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);
            // std::cout << "Receiver Velocity and clock drift = " << x[0] << ',' << x[1] << ',' << x[2] <<  ',' << x[3] << std::endl;
            /* doppler residual */
            // std::cout<< "doppler:" << doppler_shifts[i]<< std::endl;
            // std::cout<< " dts:"<<dts[1+i*2]<< std::endl;
            //  std::cout<< "lambda:" << lam << ", doppler:" << doppler_shifts[i] << ", rate:" << rate<<", x:"<<x<<", dts:"<<dts[1+i*2]<<std::endl;
            for (j = 0; j < 6; j++) 
                H[j + nv * 6] = j < 3 ? -e[j] : 0.0; // 构建设计矩阵 H 的前三列（对应接收机速度）
            if (m_GNSS_Tools.PRNisGPS(prn[i])) // 如果是GPS
            {
                // std::cout<< " GPS doppler residue ("<< nv <<"):" << v[nv] << std::endl;
                v[nv] = -lam * doppler_shifts[i] - (rate + x[3] - CLIGHT * dts[1 + i * 2]); // 多普勒残差 = 观测值 - ( 预测距离变化率 + 钟漂 - 卫星钟漂)
                H[3 + nv * 6] = 1.0; // 设计矩阵第四列设置为1（GPS钟漂
            }
            else if (m_GNSS_Tools.PRNisLEO(prn[i])) // 如果是LEO
            {
                // std::cout<< " LEO doppler residue ("<< nv <<"):" << v[nv] << std::endl;
                v[nv] = -lam * doppler_shifts[i] - (rate + x[4] - CLIGHT * dts[1 + i * 2]); // 原理同上
                H[4 + nv * 6] = 1.0; // 设计矩阵第五列设置为1（LEO钟漂
            }
            else // 对于其他系统（其实也就是北斗）
            {
                // std::cout<< " Unknown doppler residue ("<< nv <<"):" << v[nv] << std::endl;
                v[nv] = -lam * doppler_shifts[i] - (rate + x[5] - CLIGHT * dts[1 + i * 2]);
                H[5 + nv * 6] = 1.0; // 第六列是给其他系统的钟漂留的
            }
            // std::cout<< " doppler residue ("<< nv <<"):" << v[nv] << std::endl;
            /* design matrix */

            nv++; // 增加有效观测计数
        }
        return nv; // 返回有效观测数量
    }

    void solvePptimization()
    {
        while (1)
        {
            // process gnss raw measurements
            optimize_mux.lock();
            {
                std::lock_guard<std::mutex> lock(m_gnss_raw_mux);
                if (!gnss_leo_raw_buf.empty())
                {
                    gnss_leo_raw_map.insert(gnss_leo_raw_buf.begin(), gnss_leo_raw_buf.end());
                    gnss_leo_raw_buf.clear(); // 清空缓冲区
                    gnss_frame = 0;
                    hasNewData = true; // 标记有新数据
                }
            }
            if (gnss_leo_raw_map.size() && hasNewData)
            {
                TicToc optimization_time;
                ceres::Problem problem;
                ceres::Solver::Options options;
                options.use_nonmonotonic_steps = true;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
                options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
                options.num_threads = 8;
                options.max_num_iterations = 100;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                // loss_function = new ceres::HuberLoss(1.0);
                loss_function = NULL;

                int length = gnss_leo_raw_map.size();
                double state_array[length][5]; // ECEF_x, ECEF_y, ECEF_z, ECEF_gps_clock_bias, ECEF_beidou_clock_bias
                // std::vector<double*> parameter_blocks;
                // std::vector<double*> state_array;
                // state_array.reserve(length);

                // for(int i = 0; i < length; i++)
                // {
                //     double a[5] = {1,2,3,4,5};
                //     state_array.push_back(a);
                // }

                std::map<double, nlosexclusion::GNSS_Raw_Array>::iterator iter;
                iter = gnss_leo_raw_map.begin();
                for (int i = 0; i < length; i++, iter++) // initialize
                {
                    nlosexclusion::GNSS_Raw_Array gnss_leo_data = (iter->second);
                    Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                        m_GNSS_Tools.getAllPositions(gnss_leo_data),
                        m_GNSS_Tools.getAllMeasurements(gnss_leo_data),
                        gnss_leo_data, "WLS");

                    state_array[i][0] = -2419233.0952641154; // 0
                    state_array[i][1] = 5385474.751636437;   // 0
                    state_array[i][2] = 2405344.414697726;   // 0 ECEF Origin changed by Yixin
                    state_array[i][3] = 0;
                    state_array[i][4] = 0;

                    problem.AddParameterBlock(state_array[i], 5);
                }

                std::map<double, nlosexclusion::GNSS_Raw_Array>::iterator iter_pr;
                iter_pr = gnss_leo_raw_map.begin();
                for (int m = 0; m < length; m++, iter_pr++) //
                {
                    nlosexclusion::GNSS_Raw_Array gnss_leo_data = (iter_pr->second);
                    MatrixXd weight_matrix;                                                   // goGPS weighting
                    weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_leo_data, "WLS"); // goGPS
                    // std::cout << "weight_matrix: rows = " << weight_matrix.rows() << ", cols = " << weight_matrix.cols() << std::endl;//add by Yixin
                    int sv_cnt = gnss_leo_data.GNSS_Raws.size();
                    // state_array[m] = new double[5]; //
                    // double a[5] = {1,2,3,4,5};
                    // state_array[m] = a;
                    for (int i = 0; i < sv_cnt; i++)
                    {
                        std::string sat_sys;
                        double s_g_x = 0, s_g_y = 0, s_g_z = 0, var = 1;
                        double pseudorange = 0;
                        if (m_GNSS_Tools.PRNisGPS(gnss_leo_data.GNSS_Raws[i].prn_satellites_index))
                            sat_sys = "GPS";
                        else if (m_GNSS_Tools.PRNisBeidou(gnss_leo_data.GNSS_Raws[i].prn_satellites_index))
                            sat_sys = "BeiDou";
                        else if (m_GNSS_Tools.PRNisLEO(gnss_leo_data.GNSS_Raws[i].prn_satellites_index))
                            sat_sys = "LEO";
                        else
                            sat_sys = "Unknown";

                        s_g_x = gnss_leo_data.GNSS_Raws[i].sat_pos_x;
                        s_g_y = gnss_leo_data.GNSS_Raws[i].sat_pos_y;
                        s_g_z = gnss_leo_data.GNSS_Raws[i].sat_pos_z;

                        pseudorange = gnss_leo_data.GNSS_Raws[i].pseudorange;

                        ceres::CostFunction *ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1, 5>(new pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1 / weight_matrix(i, i))));
                        problem.AddResidualBlock(ps_function, loss_function, state_array[m]);

                        //     pseudorangeConstraint::PseudorangeCostFunction* cost_function =
                        //     pseudorangeConstraint::Create(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i)), m);

                        //     // pseudorange_costFunction->AddParameterBlock(5);
                        //     // pseudorange_costFunction->SetNumResiduals(1);
                        //     std::cout << "state_array size" <<state_array.size()<< "\n";
                        //     problem.AddResidualBlock(cost_function, loss_function, state_array);
                    }
                }

                ceres::Solve(options, &problem, &summary);
                std::cout << summary.BriefReport() << "\n";
                Eigen::Matrix<double, 3, 1> ENU;
                Eigen::Matrix<double, 3, 1> state;
                Eigen::Matrix<double, 3, 1> LLH;
                state << state_array[length - 1][0], state_array[length - 1][1], state_array[length - 1][2];

                ENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);
                LLH = m_GNSS_Tools.ecef2llh(state);
                LOG(INFO) << "ENU- WLS-> " << std::endl
                          << ENU;
                LOG(INFO) << "ENU- LLH-> " << std::endl
                          << LLH;
                nav_msgs::Odometry odometry;
                // check if the key 'length' exists in the map // 检查gnss_leo_map中是否存在键为length的元素，若存在就用GPSTimeToROSTime函数转换为ROS时间，并设置为里程计消息时间戳
                if (gnss_leo_raw_map.find(length) != gnss_leo_raw_map.end())
                {
                    odometry.header.stamp = GPSTimeToROSTime(
                        (gnss_leo_raw_map[length].GNSS_Raws[0].GNSS_week * 604800 +
                         gnss_leo_raw_map[length].GNSS_Raws[0].GNSS_time));
                }
                else
                {
                    std::cerr << "Error: Key 'length' not found in gnss_leo_raw_map" << std::endl;
                    std::cerr << "length: " << length << std::endl;

                    return;
                }
                // 再次设置里程计消息的时间戳，并设计消息的参考坐标系和子坐标系为frame_id('map')
                odometry.header.stamp = GPSTimeToROSTime((gnss_leo_raw_map[length].GNSS_Raws[0].GNSS_week * 604800 + gnss_leo_raw_map[length].GNSS_Raws[0].GNSS_time)); // need to change according to your week
                odometry.header.frame_id = frame_id;                                                                                                                    //"map";
                odometry.child_frame_id = frame_id;                                                                                                                     //"map";
                odometry.pose.pose.position.x = ENU(0);
                odometry.pose.pose.position.y = ENU(1);
                odometry.pose.pose.position.z = ENU(2); // 东北天的位置设置为里程计消息的位置
                rover_x.push_back(state(0)); // rover position in ECEF
                rover_y.push_back(state(1));
                rover_z.push_back(state(2)); // 接收机在ECEF的位置添加到对应向量中，用于记录轨迹
                // publish the result
                // ROS_INFO_STREAM("Publishing odometry: " << odometry);
                pub_WLS.publish(odometry); // 发布里程计消息到pub_WLS话题

                nav_msgs::Odometry odometry_ecef; // 创建第二个里程计消息，这次用ECEF坐标系下的位置，发布到pub_WLS_ECEF话题
                odometry_ecef.header.stamp = GPSTimeToROSTime((gnss_leo_raw_map[length].GNSS_Raws[0].GNSS_week * 604800 + gnss_leo_raw_map[length].GNSS_Raws[0].GNSS_time)); // need to change according to your week
                odometry_ecef.header.frame_id = frame_id;                                                                                                                    //"glio_leo";
                odometry_ecef.child_frame_id = frame_id;                                                                                                                     //"glio_leo";
                odometry_ecef.pose.pose.position.x = state(0);
                odometry_ecef.pose.pose.position.y = state(1);
                odometry_ecef.pose.pose.position.z = state(2);
                // publish the result
                // ROS_INFO_STREAM("Publishing odometry: " << odometry);
                pub_WLS_ECEF.publish(odometry_ecef);

                // 获取ROS参数服务器中的WLS_trajectory参数（WLS的轨迹文件路径）
                double current_tow = 0;
                std::string WLS_trajectory;
                ros::param::get("~WLS_trajectory", WLS_trajectory);
                if (!ros::param::get("~WLS_trajectory", WLS_trajectory))
                {
                    std::cerr << "ROS parameter 'WLS_trajectory' is not set!" << std::endl;
                    return;
                }
                FILE *gnss_leo_wls = fopen(WLS_trajectory.c_str(), "a+"); // 打开轨迹文件gnss_leo_wls
                if (gnss_leo_wls == NULL)
                {
                    perror("Error opening file");
                    return;
                }
                // 遍历所有数据点，将位置信息写入轨迹文件
                wls_path.poses.clear();
                for (int m = 0; m < length; m++) //
                {
                    nlosexclusion::GNSS_Raw_Array gnss_leo_data = gnss_leo_raw_map[m + 1]; // 获取GNSS原始数据
                    state << state_array[m][0], state_array[m][1], state_array[m][2]; // 获取接收机位置？
                    current_tow = gnss_leo_data.GNSS_Raws[0].GNSS_time; // 获取周内秒数
                    ENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state); 
                    LLH = m_GNSS_Tools.ecef2llh(state); // ECEF坐标转为ECEF/LLH
                    fprintf(gnss_leo_wls, "%d,%d,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f  \n", int(gnss_leo_data.GNSS_Raws[0].GNSS_week), int(current_tow), LLH(1), LLH(0), LLH(2), ENU(0), ENU(1), ENU(2));
                    fflush(gnss_leo_wls);
                }

                std::cout << "Time used for Ceres-solver-> " << optimization_time.toc() << std::endl;
                // 声明用于多普勒速度估计的变量向量
                std::vector<double> doppler_shifts;
                std::vector<double> lambdas;
                std::vector<double> rs;
                std::vector<double> rs_tmp;
                std::vector<double> dts;
                std::vector<double> dts_tmp;
                std::vector<double> azel;
                std::vector<double> prn;
                std::vector<int> vsat;
                // 初始化接收机位置数组，并遍历所有数据点
                double rr[6] = {0};
                for (int m = 0; m < length; m++)
                {
                    // add by Yixin
                    //  double *doppler_shifts,*lambdas,*rs,*dts,*var,*azel,*vsat;
                    //  int n = gnss_leo_data.GNSS_Raws.size();
                    //  rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel=zeros(2,n);
                    //  doppler_shifts=mat(n,1); lambdas=mat(n,1); vsat=mat(n,1);
                    nlosexclusion::GNSS_Raw_Array gnss_leo_data = gnss_leo_raw_map[m + 1]; // Changed by Yixin
                    rr[0] = state_array[m][0];
                    rr[1] = state_array[m][1];
                    rr[2] = state_array[m][2];
                    // 遍历当前数据点的所有卫星观测数据，将多普勒频移、波长、卫星位置速度、卫星钟差、方位角仰角等信息存储到对应向量中。
                    for (int i = 0; i < gnss_leo_data.GNSS_Raws.size(); i++)
                    {
                        doppler_shifts.push_back(gnss_leo_data.GNSS_Raws[i].doppler);
                        lambdas.push_back(gnss_leo_data.GNSS_Raws[i].lambda);
                        rs_tmp = {gnss_leo_data.GNSS_Raws[i].sat_pos_x, gnss_leo_data.GNSS_Raws[i].sat_pos_y, gnss_leo_data.GNSS_Raws[i].sat_pos_z, gnss_leo_data.GNSS_Raws[i].vel_x, gnss_leo_data.GNSS_Raws[i].vel_y, gnss_leo_data.GNSS_Raws[i].vel_z};
                        dts_tmp = {gnss_leo_data.GNSS_Raws[i].dt, gnss_leo_data.GNSS_Raws[i].ddt};
                        rs.insert(rs.end(), rs_tmp.begin(), rs_tmp.end());
                        dts.insert(dts.end(), dts_tmp.begin(), dts_tmp.end());
                        azel.push_back(gnss_leo_data.GNSS_Raws[i].azimuth * D2R);
                        azel.push_back(gnss_leo_data.GNSS_Raws[i].elevation * D2R);
                        // azel.insert(azel.end(), gnss_leo_data.GNSS_Raws[i].azimuth*D2R, gnss_leo_data.GNSS_Raws[i].elevation*D2R);
                        vsat.push_back(1);
                        prn.push_back(gnss_leo_data.GNSS_Raws[i].prn_satellites_index);
                    }
                    // 调用pntpos_GNSSLEO函数进行基于多普勒的定位解算，然后清空所有向量为下一次迭代做准备
                    pntpos_GNSSLEO(prn.data(), gnss_leo_data.GNSS_Raws[0].GNSS_week, gnss_leo_data.GNSS_Raws[0].GNSS_time, doppler_shifts.data(), lambdas.data(),
                                   gnss_leo_data.GNSS_Raws.size(), rs.data(), dts.data(), rr,
                                   azel.data(), vsat.data());
                    doppler_shifts.clear();
                    lambdas.clear();
                    rs.clear();
                    dts.clear();
                    azel.clear();
                    vsat.clear();
                    prn.clear();
                }
                hasNewData = false;
                gnss_leo_raw_map.clear();
            }

            std::chrono::milliseconds dura(10); // this thread sleep for 100 ms
            std::this_thread::sleep_for(dura);

            optimize_mux.unlock();
        }
    }

    //     void publishLEOOdometry(const ros::TimerEvent&)
    //     {
    //         if (!odometry_queue.empty())
    //         {
    //             pub_velocity_from_doppler.publish(odometry_queue.front());
    //             odometry_queue.pop();
    //         }
    //     }
    // 这个函数没用上，之前保存那个文件被注释掉了
    void saveOdometryToFile(const std::string &filename, const double current_week, const double current_tow, const double *rr, const double *velocity)
    {
        std::ofstream file(filename, std::ios::app);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        //  set the precision for floating-point numbers
        file << std::fixed << std::setprecision(6);

        //  Write data to the file
        file << current_week << ", " << current_tow << ", "
             << rr[0] << ", " << rr[1] << ", " << rr[2] << ", "
             << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << "\n";
        std::cout << "Odometry data saved to file: " << filename << std::endl;
    }
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;              // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    // google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags
    ros::init(argc, argv, "psr_spp_gnssleo_node");
    ROS_INFO("\033[1;32m----> psr_spp_gnssleo_node (solve WLS using Ceres-solver) Started.\033[0m");
    /* --- 新增 ---*/
    std::string WLS_trajectory;
    if (!ros::param::get("~WLS_trajectory", WLS_trajectory))
    {
        std::cerr << "ROS parameter 'WLS_trajectory' is not set!" << std::endl;
        return 0;
    }
    FILE *gnss_leo_spp = fopen(WLS_trajectory.c_str(), "w+"); // 从ROS参数服务器获取轨迹文件路径参数，并以写入模式打开
    if (gnss_leo_spp == NULL)
    {
        perror("Error opening gnss-leo output file");
        return 0;
    }
    fclose(gnss_leo_spp);
    /* --- 新增结束 --- */
    gnssSinglePointPositioning gnssSinglePointPositioning;
    ros::spin();
    return 0;
}