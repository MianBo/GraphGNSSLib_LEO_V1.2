#include <iostream>
#include <string>  
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>
#include <vector>
#include <ros/ros.h>
#include "../../RTKLIB/src/rtklib.h"
#include <std_msgs/Header.h>
#include <nlosExclusion/GNSS_Raw.h>
#include <nlosExclusion/GNSS_Raw_Array.h>
#include <nlosExclusion/LEO_dopp.h>
#include <nlosExclusion/LEO_dopp_Array.h>


// GPS epoch at 1980/1/6
const ros::Time gps_epoch(315964800, 0); // 315964800 from 1970/1/1 to 1980/1/6

ros::Time GPSTimeToROSTime(double gps_time) {
    int gps_week = static_cast<int>(gps_time / 604800);
    double gps_seconds = gps_time - gps_week * 604800;

    // Calculate the total seconds in ros to the given GPS time
    double total_seconds = gps_week * 604800 + gps_seconds+ gps_epoch.toSec();    

    // Transform the total seconds to seconds and nanoseconds
    int64_t sec = static_cast<int64_t>(total_seconds);
    int64_t nsec = static_cast<int64_t>((total_seconds - sec) * 1e9);

    // Create a ROS time object
    ros::Time ros_time(sec, nsec);
    
    return ros_time;
}

std::vector<std::vector<std::string>> readCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return {};
    }
    std::string line;
    std::vector<std::vector<std::string>> data;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> row;
        std::string value;
        while (std::getline(iss, value, ',')) {
            row.push_back(value);
        }
        data.push_back(row);
    }
    std::cout << "Read " << data.size() << " rows from file: " << filename << std::endl;
    file.close();
    return data;
}

void processCSVData(const std::vector<std::vector<std::string>>& data, ros::Publisher& pub1, ros::Publisher& pub2) {
    nlosExclusion::GNSS_Raw_Array gnss_raw_array; 
    nlosExclusion::LEO_dopp_Array leo_dopp_array;
    nlosExclusion::GNSS_Raw gnss_raw;
    nlosExclusion::LEO_dopp leo_dopp_element;
    gnss_raw_array.header = std_msgs::Header();
    leo_dopp_array.header = std_msgs::Header();
    ros::Rate rate(5);  // 2.5 Hz
    std::cout << "Starting GNSS Preprocessor..." << std::endl;
    std::cout << "Total lines of data: " << data.size() << std::endl;
    size_t i = 0;
    size_t sv_count = 0;
    double total_sv_init = 0;
    
    while (i < data.size()) {
        // std::cout << "Processing line: " << i << std::endl;
        if (data[i].empty()) {
            break;
        }
        
        double gnss_time = std::stod(data[i][0]);
        total_sv_init = std::stoi(data[i+1][0]);
        
        
        while (sv_count < total_sv_init) {
            gnss_raw.GNSS_time = std::stod(data[i][0]);
            gnss_raw.total_sv = std::stod(data[i+1][0]);
            gnss_raw.prn_satellites_index = std::stod(data[i+2][0]);
            gnss_raw.pseudorange = std::stod(data[i+3][0]);
            gnss_raw.raw_pseudorange = std::stod(data[i+4][0]);
            gnss_raw.carrier_phase = std::stod(data[i+5][0]);
            gnss_raw.lamda = std::stod(data[i+6][0]);
            gnss_raw.snr = std::stod(data[i+7][0]);
            gnss_raw.elevation = std::stod(data[i+8][0]);
            gnss_raw.azimuth = std::stod(data[i+9][0]);
            gnss_raw.err_tropo = std::stod(data[i+10][0]);
            gnss_raw.err_iono = std::stod(data[i+11][0]);
            gnss_raw.sat_clk_err = std::stod(data[i+12][0]);
            gnss_raw.sat_pos_x = std::stod(data[i+13][0]);
            gnss_raw.sat_pos_y = std::stod(data[i+14][0]);
            gnss_raw.sat_pos_z = std::stod(data[i+15][0]);
            gnss_raw.visable = std::stoi(data[i+16][0]);
            gnss_raw.sat_system = data[i+17][0];
            gnss_raw.visable3DMA = std::stoi(data[i+18][0]);
            gnss_raw.prE3dMA = std::stod(data[i+19][0]);

            gnss_raw_array.GNSS_Raws.push_back(gnss_raw);

            leo_dopp_element.GNSS_time = gnss_raw.GNSS_time;     
            leo_dopp_element.doppler_shifts =std::stod(data[i+25][0]);      
            leo_dopp_element.lambdas=std::stod(data[i+6][0]);
            //std::cout << "doppler_shifts: " << std::stod(data[i+25][0]) << std::endl;
            
            leo_dopp_element.n = total_sv_init;
            leo_dopp_element.rs.resize(6);
            // add by Yixin for Debug
        //if (i + 20 < data.size() && !data[i + 20].empty()) {
            leo_dopp_element.rs[0] = std::stod(data[i+13][0]);
            leo_dopp_element.rs[1] = std::stod(data[i+14][0]);
            leo_dopp_element.rs[2] = std::stod(data[i+15][0]);
            leo_dopp_element.rs[3] = std::stod(data[i+20][0]);
            leo_dopp_element.rs[4] = std::stod(data[i+21][0]);
            leo_dopp_element.rs[5] = std::stod(data[i+22][0]);
        //} else {
        //    std::cerr << "Invalid index or empty data at row: " << i + 20 << std::endl;
        //    continue;
        //}
            leo_dopp_element.dts.resize(2);
            leo_dopp_element.dts[0] = std::stod(data[i+23][0]);
            leo_dopp_element.dts[1] = std::stod(data[i+24][0]);
            leo_dopp_element.rr.resize(6);
            leo_dopp_element.rr = std::vector<double>{gnss_raw.GNSS_time, 0.0, 0.0, 0.0, 0.0, 0.0};
            leo_dopp_element.azel.resize(2);
            leo_dopp_element.azel[0]=std::stod(data[i+9][0])*D2R;
            leo_dopp_element.azel[1]=std::stod(data[i+8][0])*D2R;         
            leo_dopp_element.vsat=1; // all satellites are visible
            leo_dopp_array.LEO_Dopps.push_back(leo_dopp_element);
            sv_count++;
            i += 26;  // read 26 rows each time
        }
        if (sv_count == total_sv_init) {          
                gnss_raw_array.header.stamp = GPSTimeToROSTime(2158*604800+gnss_raw.GNSS_time);
                leo_dopp_array.header.stamp = GPSTimeToROSTime(2158*604800+gnss_raw.GNSS_time);  
                pub1.publish(gnss_raw_array);
                pub2.publish(leo_dopp_array);
                gnss_raw_array = nlosExclusion::GNSS_Raw_Array();
                gnss_raw_array.header = std_msgs::Header(); 
                leo_dopp_array = nlosExclusion::LEO_dopp_Array();
                leo_dopp_array.header = std_msgs::Header();      
                sv_count = 0;
                rate.sleep();
        }
    }
}


int main(int argc, char** argv) {
    // ros::init(argc, argv, "gnss_raw_publisher");
    ros::init(argc, argv, "leo_raw_publisher");
    ros::NodeHandle nh;
    
    
    
    std::string filename = "/home/gao-yixin/GraphGNSSLib_LEO/src/global_fusion/dataset/2021_0521_0607/StarLink_Whampoa_0521.csv";
    std::vector<std::vector<std::string>> data = readCSV(filename);
    
    ros::Publisher pub1 = nh.advertise<nlosExclusion::GNSS_Raw_Array>("/gnss_preprocessor_node/LEOPsrCarRov1", 50);
    ros::Publisher pub2 = nh.advertise<nlosExclusion::LEO_dopp_Array>("/gnss_preprocessor_node/LEO_Dopp_Array", 50);
    processCSVData(data, pub1, pub2);
    ROS_INFO("Data published successfully. Exiting...");
    return 0;
}