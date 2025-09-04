#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>
#include <vector>
#include <ros/ros.h>
#include "../../RTKLIB/src/rtklib.h"
#include <std_msgs/Header.h>
#include <nlosexclusion/GNSS_Raw.h>
#include <nlosexclusion/GNSS_Raw_Array.h>

ros::Time GPSTimeToROSTime(double gps_time);

std::vector<std::vector<std::string>> readCSV(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return {};
    }
    std::string line;
    std::vector<std::vector<std::string>> data;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<std::string> row;
        std::string value;
        while (std::getline(iss, value, ','))
        {
            row.push_back(value);
        }
        data.push_back(row);
    }
    std::cout << "Read " << data.size() << " rows from file: " << filename << std::endl;
    file.close();
    return data;
}

void processRovLEOCSVData(const std::vector<std::vector<std::string>> &data, ros::Publisher &pub)
{
    nlosexclusion::GNSS_Raw_Array gnss_raw_array; // 初始化一组用于发布的GNSS观测值数组消息对象

    nlosexclusion::GNSS_Raw gnss_raw; // 单个GNSS观测数据的临时对象

    gnss_raw_array.header = std_msgs::Header();

    ros::Rate rate(2.5); // 2.5 Hz 发布频率
    std::cout << "Starting  Rover LEO Raw Publisher..." << std::endl;
    std::cout << "Total lines of data: " << data.size() << std::endl;
    size_t i = 0;             // 当前数据行索引
    size_t sv_count = 0;      // 历元已处理的卫星数量
    double total_sv_init = 0; // 当前历元的总卫星数

    while (i < data.size())
    { // 循环处理每个历元的数据
        // std::cout << "Processing line: " << i << std::endl;
        if (data[i].empty())
        {
            break;
        }
        total_sv_init = std::stoi(data[i + 2][0]); // 读卫星总数
        while (sv_count < total_sv_init)
        {
            gnss_raw.GNSS_week = std::stod(data[i][0]);
            gnss_raw.GNSS_time = std::stod(data[i + 1][0]);
            gnss_raw.total_sv = std::stod(data[i + 2][0]);
            gnss_raw.prn_satellites_index = std::stod(data[i + 3][0]);
            gnss_raw.pseudorange = std::stod(data[i + 4][0]);
            gnss_raw.raw_pseudorange = std::stod(data[i + 5][0]);
            gnss_raw.carrier_phase = std::stod(data[i + 6][0]);
            gnss_raw.doppler = std::stod(data[i + 7][0]);
            gnss_raw.lambda = std::stod(data[i + 8][0]);
            gnss_raw.snr = std::stod(data[i + 9][0]);
            gnss_raw.LLI = std::stoi(data[i + 10][0]);
            gnss_raw.slip = std::stoi(data[i + 11][0]);
            gnss_raw.elevation = std::stod(data[i + 12][0]);
            gnss_raw.azimuth = std::stod(data[i + 13][0]);
            gnss_raw.err_tropo = std::stod(data[i + 14][0]);
            gnss_raw.err_iono = std::stod(data[i + 15][0]);
            gnss_raw.sat_clk_err = std::stod(data[i + 16][0]);
            gnss_raw.sat_pos_x = std::stod(data[i + 17][0]);
            gnss_raw.sat_pos_y = std::stod(data[i + 18][0]);
            gnss_raw.sat_pos_z = std::stod(data[i + 19][0]);
            gnss_raw.ttx = std::stod(data[i + 20][0]);
            gnss_raw.vel_x = std::stod(data[i + 21][0]);
            gnss_raw.vel_y = std::stod(data[i + 22][0]);
            gnss_raw.vel_z = std::stod(data[i + 23][0]);
            gnss_raw.dt = std::stod(data[i + 24][0]);
            gnss_raw.ddt = std::stod(data[i + 25][0]);
            gnss_raw.tgd = std::stod(data[i + 26][0]);
            gnss_raw.visable = std::stoi(data[i + 27][0]);
            gnss_raw.sat_system = 'LEO'; // data[i+28][0];
            gnss_raw.visable3DMA = std::stoi(data[i + 29][0]);
            gnss_raw.prE3dMA = std::stod(data[i + 30][0]);
            gnss_raw_array.GNSS_Raws.push_back(gnss_raw);
            sv_count++;
            i += 31; // read 26 rows each time
        }
        if (sv_count == total_sv_init)
        {                                                                                                     // 当一个历元的所有卫星数据处理完成后
            gnss_raw_array.header.stamp = GPSTimeToROSTime(gnss_raw.GNSS_week * 604800 + gnss_raw.GNSS_time); // 设置消息时间戳，通过函数将GPS时间转为ROS时间

            pub.publish(gnss_raw_array); // 发布消息
            gnss_raw_array = nlosexclusion::GNSS_Raw_Array();
            gnss_raw_array.header = std_msgs::Header();
            sv_count = 0; // 重置gnss_raw_array对象和计数器，准备处理下一个历元
            rate.sleep();
        }
    }
}

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "gnss_raw_publisher");
    ros::init(argc, argv, "leo_rover_raw_publisher");
    ros::NodeHandle nh;
    int mode;
    std::string roverLEOMeasureFile;
    if (ros::param::get("roverLEOMeasureFile", roverLEOMeasureFile))
    {
        ROS_INFO("Got param: %s", roverLEOMeasureFile.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param 'roverLEOMeasureFile'");
    }
    // publish rover & base station received pseudorange and dopp informations from LEO satellites

    std::vector<std::vector<std::string>> roverdata = readCSV(roverLEOMeasureFile);
    ros::Publisher pub1 = nh.advertise<nlosexclusion::GNSS_Raw_Array>("/leo_raw_publisher_node/LEOPsrCarRov1", 1000);
    processRovLEOCSVData(roverdata, pub1);

    ROS_INFO("Data published successfully. Exiting...");
    return 0;
}