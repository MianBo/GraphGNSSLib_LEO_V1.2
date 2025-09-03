/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * Function: subscribe the raw measurements, perform snap shot GNSS single point positioning based pseudorange measurements. The theoretical output should be similar with the conventional weighted least squares (WLS).
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
#include "ceres/dynamic_autodiff_cost_function.h"


#include "../tic_toc.h"

// rtklib /
#include <stdarg.h>
#include "../RTKLIB/src/rtklib.h"

DEFINE_double(corridor_length, 30.0, "Length of the corridor that the robot is "
              "travelling down."); // 走廊长度
DEFINE_double(pose_separation, 0.5, "The distance that the robot traverses "
              "between successive odometry updates."); // 接收机位姿间隔距离
DEFINE_double(odometry_stddev, 0.1, "The standard deviation of "
              "odometry error of the robot."); // 里程计标准差
DEFINE_double(range_stddev, 0.01, "The standard deviation of range readings of "
              "the robot."); // 测距标准差
// The stride length of the dynamic_autodiff_cost_function evaluator.
static const int kStride = 10; // 动态自动微分代价函数评估器的步长常量


class gnssSinglePointPositioning
{
    ros::NodeHandle nh; // 创建ROS节点句柄

    // ros subscriber
    ros::Subscriber gnss_raw_sub; // 订阅者，订阅GNSS原始观测数据
    ros::Subscriber leo_raw_sub;//add by Yixin，从这里开始就添加LEO？
    ros::Publisher pub_WLS, pub_FGO; // 发布者，发布WLS和FGO的定位结果
    std::queue<nlosexclusion::GNSS_Raw_ArrayConstPtr> gnss_raw_buf; // GNSS原始数据的队列缓冲区
    std::map<double, nlosexclusion::GNSS_Raw_Array> gnss_raw_map; // 按时间戳映射的GNSS原始数据容器

    std::mutex m_gnss_raw_mux; 
    std::mutex optimize_mux;  // 两个互斥锁 
    std::thread optimizationThread; // 用于执行优化计算的独立线程

    GNSS_Tools m_GNSS_Tools; // utilities

    nav_msgs::Path fgo_path; // 存储FGO结果的路径信息，便于在ROS中可视化接收机的运行轨迹

    int gnss_frame = 0; // GNSS数据帧

    Eigen::Matrix<double, 3,1> ENULlhRef; // ENU->LLH参考点坐标？

    bool hasNewData =false; // 

    
public: 
    // from Weisong
    struct pseudorangeFactor
    {
        // 构造函数初始化列表，包括卫星系统类型，卫星在ECEF中的位置，伪距观测值和观测值的方差
        pseudorangeFactor(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var)
                    :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange),var(var){} 
        template <typename T> 
        bool operator()(const T* state, T* residuals) const // 重载，是Ceres Solver要求的函数形式，用于计算残差
        {
            T est_pseudorange; 
            T delta_x = pow((state[0] - s_g_x),2); // state是状态向量，包括接收机xyz坐标和gps/bds系统钟差
            T delta_y = pow((state[1] - s_g_y),2);
            T delta_z = pow((state[2] - s_g_z),2); 
            est_pseudorange = sqrt(delta_x+ delta_y + delta_z);// 计算接收机和卫星之间的欧几里的距离

            double OMGE_ = 7.2921151467E-5;
            double CLIGHT_ = 299792458.0;
            est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state[1]-s_g_y*state[0])/CLIGHT_; // 地球自转校正值
            
            if(sat_sys == "GPS") // 钟差修正（LEO不修）
            {
                est_pseudorange = est_pseudorange - state[3];
            }
            else 
            {
                est_pseudorange = est_pseudorange - state[4]; // 北斗
            }
            residuals[0] = (est_pseudorange - T(pseudorange)) / T(var); // 残差值计算并归一化

            return true;
        }
        double s_g_x, s_g_y, s_g_z, pseudorange, var; // 
        std::string sat_sys; // satellite system
    };

    // from Weisong
    
    struct pseudorangeConstraint // 实现动态自动微分的代价函数
    {
        typedef ceres::DynamicAutoDiffCostFunction<pseudorangeConstraint, kStride>
      PseudorangeCostFunction; // ceres的dynamicAutoDiffCostFuntion模板类的实例化
      // 多了一个keyindex，状态向量中的索引，用来在动态参数数组中定位特定的状态向量  
      pseudorangeConstraint(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var, int keyIndex)
                    :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange),var(var), keyIndex(keyIndex){} 
        template <typename T> // 这里的state是双重指针T const* const*
        bool operator()(T const* const* state, T* residuals) const
        {

            T est_pseudorange; 
            T delta_x = pow((state[keyIndex][0] - s_g_x),2);
            T delta_y = pow((state[keyIndex][1] - s_g_y),2);
            T delta_z = pow((state[keyIndex][2] - s_g_z),2);
            est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

            double OMGE_ = 7.2921151467E-5;
            double CLIGHT_ = 299792458.0;
            est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state[keyIndex][1]-s_g_y*state[keyIndex][0])/CLIGHT_;
            if(sat_sys == "GPS") 
            {
                est_pseudorange = est_pseudorange - state[keyIndex][3];
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
        static PseudorangeCostFunction* Create(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var, int keyIndex) {
            // 定义静态工厂方法 Creat 创建pseudorangeConstraint的代价函数实例来初始化约束条件
            pseudorangeConstraint* constraint = new pseudorangeConstraint(
                sat_sys, s_g_x, s_g_y, s_g_z,pseudorange,var, keyIndex); // 利用传入的约束创建对象
            // 创建一个PseudorangeCostFunction对象，将之前创建的约束对象作为参数传入。这将把约束条件包装成Ceres优化器可以使用的代价函数。
            PseudorangeCostFunction* cost_function = new PseudorangeCostFunction(constraint); 
            std::cout << "keyIndex-> " << keyIndex << std::endl;
            for(int i = 0; i <(keyIndex+1); i++) // 循环添加参数块到代价函数中，这里添加了keyIndex+1个参数块，每个参数块大小为5，即状态向量的个数。
            {
                cost_function->AddParameterBlock(5);
            }
            
            cost_function->SetNumResiduals(1);// 设置残差数量，1是因为每个伪距观测值只产生一个残差
            return (cost_function); // 返回代价函数实例
        }

        double s_g_x, s_g_y, s_g_z, pseudorange, var;
        int keyIndex;
        std::string sat_sys; // satellite system
    };



public:
  gnssSinglePointPositioning() // 构造函数
  {
      // 利用nh这个ROS节点句柄来订阅主题，当有新消息到达时，调用gnss_raw_msg_callback回调函数处理消息。
      gnss_raw_sub = nh.subscribe("/gnss_preprocessor_node/GNSSPsrCarRov1", 500, &gnssSinglePointPositioning::gnss_raw_msg_callback, this); // call callback for gnss raw msg
      //leo_raw_sub = nh.subscribe("/leo_raw_publisher_node/LEOPsrCarRov1", 500, &gnssSinglePointPositioning::gnss_raw_msg_callback, this); // call callback for leo raw msg add by Yixin
      optimizationThread = std::thread(&gnssSinglePointPositioning::solvePptimization, this);

      pub_WLS = nh.advertise<nav_msgs::Odometry>("WLS_spp", 100); // 发布WLS结果
      pub_FGO = nh.advertise<nav_msgs::Odometry>("FGO_spp", 100); // 发布FGO结果

      ENULlhRef.resize(3,1);
      ENULlhRef<< ref_lon, ref_lat, ref_alt; // 初始化参考点的ENU坐标
  }

  ~gnssSinglePointPositioning()
  {
      optimizationThread.detach();
  }

   /**
   * @brief gnss raw msg callback
   * @param gnss raw msg
   * @return void
   @ 
   */
    void gnss_raw_msg_callback(const nlosexclusion::GNSS_Raw_ArrayConstPtr& msg)
    {
        m_gnss_raw_mux.lock(); // 加锁，帧递增，处理方式和其他回调函数差不多
        gnss_frame++;
        if(msg->GNSS_Raws.size())
        {
            hasNewData = true;
            gnss_raw_buf.push(msg); 
            gnss_raw_map[gnss_frame] = *msg; // 将新接收到的数据分别存放到队列缓冲区和映射表中，后者以帧号为键存储数据
            // Eigen::MatrixXd positions = m_GNSS_Tools.getAllPositions(*msg);
            // Eigen::MatrixXd measurements = m_GNSS_Tools.getAllMeasurements(*msg);

            // print positions & measurements' cols and rows no. add by Yixin
            // std::cout << "positions: rows = " << positions.rows() << ", cols = " << positions.cols() << std::endl;
            // std::cout << "measurements: rows = " << measurements.rows() << ", cols = " << measurements.cols() << std::endl;
            
            Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                            m_GNSS_Tools.getAllPositions(*msg),
                                            m_GNSS_Tools.getAllMeasurements(*msg),
                                            *msg, "WLS"); // 调用gpss_tool中的xxx方法执行加权最小二乘定位解算
            Eigen::Matrix<double ,3,1> ENU;
            ENU = m_GNSS_Tools.ecef2enu(ENULlhRef, eWLSSolutionECEF); // ecef -> enu
            LOG(INFO) << "ENU WLS -> "<< std::endl << ENU;

            nav_msgs::Odometry odometry; // 创建消息对象并设置其帧ID
            odometry.header.frame_id = "map";
            odometry.child_frame_id = "map"; // 位置信息参考地图坐标系
            odometry.pose.pose.position.x = ENU(0);
            odometry.pose.pose.position.y = ENU(1);
            odometry.pose.pose.position.z = ENU(2); // 将ENU坐标分别填入Odemetry消息的位置字段
            pub_WLS.publish(odometry); // 发布WLS定位结果
        }
        m_gnss_raw_mux.unlock();
    }

    void solvePptimization()
    {
        while(1)
        {
            // process gnss raw measurements
            // optimize_mux.lock();
            if(gnss_raw_map.size() && hasNewData) // 检查gnss_raw_map是否有新数据需要处理
            {
                TicToc optimization_time;
                /* 配置ceres参数 */
                ceres::Problem problem; 
                ceres::Solver::Options options; // 创建优化问题对象 
                options.use_nonmonotonic_steps = true; // 设置使用非单调步骤
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // 稀疏法线乔列斯基线性求解器
                options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG; // 使用 Dogleg信任域策略
                options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG; // 使用子空间dogleg类型
                options.num_threads = 8;
                options.max_num_iterations = 100; // 8线程迭代100次
                ceres::Solver::Summary summary; // 创建求解器摘要对象
                ceres::LossFunction *loss_function; 
                // loss_function = new ceres::HuberLoss(1.0);
                loss_function = NULL; // 设置损失函数为空（即使用标准最小二乘）
                
                int length = gnss_raw_map.size(); // 获取GNSS数据数量
                double state_array[length][5]; // 定义状态变量 ECEF_x, ECEF_y, ECEF_z, ECEF_gps_clock_bias, ECEF_beidou_clock_bias
                // std::vector<double*> parameter_blocks;
                // std::vector<double*> state_array;
                // state_array.reserve(length);

                // for(int i = 0; i < length; i++)
                // {
                //     double a[5] = {1,2,3,4,5};
                //     state_array.push_back(a);
                // }

                // 遍历所有GNSS数据
                std::map<double, nlosexclusion::GNSS_Raw_Array>::iterator iter;
                iter = gnss_raw_map.begin();
                for(int i = 0;  i < length; i++,iter++) // initialize
                {
                    nlosexclusion::GNSS_Raw_Array gnss_data = (iter->second);
                    Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                                m_GNSS_Tools.getAllPositions(gnss_data),
                                                m_GNSS_Tools.getAllMeasurements(gnss_data),
                                                gnss_data, "WLS"); // 对当前数据进行WLS计算初始解

                    state_array[i][0] = 0; // 状态数组初始化为0
                    state_array[i][1] = 0;
                    state_array[i][2] = 0;
                    state_array[i][3] = 0;
                    state_array[i][4] = 0;

                    problem.AddParameterBlock(state_array[i],5); // 将每个状态数组添加为优化问题的参数块
                }
                // 再遍历当前时刻的GNSS数据
                std::map<double, nlosexclusion::GNSS_Raw_Array>::iterator iter_pr;
                iter_pr = gnss_raw_map.begin();
                for(int m = 0;  m < length; m++,iter_pr++) // 
                {
                    nlosexclusion::GNSS_Raw_Array gnss_data = (iter_pr->second); // 获取当前时刻的GNSS数据
                    MatrixXd weight_matrix; //goGPS weighting
                    weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS 计算权重矩阵
                    std::cout << "weight_matrix: rows = " << weight_matrix.rows() << ", cols = " << weight_matrix.cols() << std::endl;//add by Yixin
                    int sv_cnt = gnss_data.GNSS_Raws.size(); // 获取可见卫星数量
                    // state_array[m] = new double[5]; //
                    // double a[5] = {1,2,3,4,5};
                    // state_array[m] = a;
                    for(int i =0; i < sv_cnt; i++) // 遍历每颗卫星
                    {
                        std::string sat_sys;
                        double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                        double pseudorange = 0;
                        if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS"; // 判断卫星系统类型
                        else sat_sys = "BeiDou";

                        s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                        s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                        s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z; // 获取卫星位置

                        pseudorange = gnss_data.GNSS_Raws[i].pseudorange; // 获取伪距观测值
                        // 使用pseudorangeFactor作为残差计算模型，每个残差块有1个残差，参数块大小为5，并将残差块添加到优化问题中
                        ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1 
                                                                , 5>(new 
                                                                pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i))));
                        problem.AddResidualBlock(ps_function, loss_function, state_array[m]);
                    //     pseudorangeConstraint::PseudorangeCostFunction* cost_function =
                    //     pseudorangeConstraint::Create(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i)), m);

                    //     // pseudorange_costFunction->AddParameterBlock(5);
                    //     // pseudorange_costFunction->SetNumResiduals(1);
                    //     std::cout << "state_array size" <<state_array.size()<< "\n";
                    //     problem.AddResidualBlock(cost_function, loss_function, state_array);
                    }
                }

                ceres::Solve(options, &problem, &summary); // 利用配置的选项求解优化问题
                // std::cout << summary.BriefReport() << "\n";
                Eigen::Matrix<double ,3,1> ENU;
                Eigen::Matrix<double, 3,1> state;
                
                state<< state_array[length-1][0], state_array[length-1][1], state_array[length-1][2];
                ENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);
                LOG(INFO) << "ENU- FGO-> "<< std::endl<< ENU;
                nav_msgs::Odometry odometry; // 创建里程计消息
                // odometry.header = pose_msg->header;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "map";
                odometry.pose.pose.position.x = ENU(0);
                odometry.pose.pose.position.y = ENU(1);
                odometry.pose.pose.position.z = ENU(2);
                pub_FGO.publish(odometry); // 发布ENU坐标位置信息
                /* 将所有时刻的位置结果写入CSV文件 */
                FILE* FGO_trajectory = fopen("psr_spp_node_trajectory.csv", "w+");
                fgo_path.poses.clear();
                for(int m = 0;  m < length; m++) // 
                {
                    state<< state_array[m][0], state_array[m][1], state_array[m][2];
                    ENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);
                    fprintf(FGO_trajectory, "%d,%7.5f,%7.5f,%7.5f  \n", m, ENU(0),ENU(1),ENU(2));
                    fflush(FGO_trajectory);
                }

                std::cout << "Time used for Ceres-solver-> "<<optimization_time.toc()<<std::endl; // 输出优化时间

                hasNewData = false;
            }
            std::chrono::milliseconds dura(10); // this thread sleep for 100 ms
            std::this_thread::sleep_for(dura);
            // gnss_raw_map.clear();
            optimize_mux.unlock();
            
        }
    }

};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    // google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "psr_spp_node");  
    ROS_INFO("\033[1;32m----> psr_spp_node (solve WLS using Ceres-solver) Started.\033[0m");
    gnssSinglePointPositioning gnssSinglePointPositioning;
    ros::spin();
    return 0;
}
