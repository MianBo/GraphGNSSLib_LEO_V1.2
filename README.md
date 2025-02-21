# GraphGNSSLib_LEO
### An Open-source Package for GNSS and LEO Coupled Positioning Using Factor Graph Optimization

This repository is the implementation of the open-sourced package, the GraphGNSSLib_LEO, which makes use of the LEO and GNSS coupled measurements to do positioning.

In this package, measured pseudorange and doppler shift of GNSS and simulated pseudorange and doppler shift of LEO satellites are used for positioning.  Measurements from the historical and current epochs are structured into a factor graph which is then solved by non-linear optimization. Positioning results from **single point positioning(SPP)** and **factor graph optimization (FGO)** method are compared. 

The package is based on **[GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)**, adding modules to support simulated LEO satellites' measurements. This package is based on C++ which is compatible with the robot operation system (ROS) platform. Meanwhile, this package combines the RTKLIB (**[version: 2.4.3 b33](http://www.rtklib.com/)**) to read/decode the GNSS [RINEX](https://en.wikipedia.org/wiki/RINEX) files. Users from Robotics field can easily have access to GNSS raw data for further study.

**Important Notes**: 
  - Be noted that the **GNSS-only SPP** mentioned throughout the package means estimating the positioing of the receiver based on the GNSS pseudorange Doppler measurements uisng SPP.
  - Be noted that the **GNSS-LEO SPP**mentioned throughout the package means estimating the positioing of the receiver based on the GNSS and LEO pseudorange Doppler measurements uisng SPP.
  - Be noted that the **GNSS-LEO FGO** mentioned throughout the package means estimating the positioing of the receiver based on the combination of GNSS and LEO pseudorange and Doppler measurements uisng FGO.
 
**Authors**: [Yixin Gao](https://polyu-taslab.github.io/members/gao_yixin.html),[Weisong Wen](https://www.polyu.edu.hk/aae/people/academic-staff/dr-wen-weisong/), from the [Trustworthy AI and Autonomous Systems (TAS) Laboratory]([https://polyu-taslab.github.io/]), The Hong Kong Polytechnic University. 

**Related Papers:** (paper is not exactly same with code)
  - Wen, W., & Hsu, L. T. (2021, May). [Towards robust GNSS positioning and Real-time kinematic using factor graph optimization](https://ieeexplore.ieee.org/abstract/document/9562037). In 2021 IEEE International Conference on Robotics and Automation (ICRA) (pp. 5884-5890). IEEE. 
  - Wen, W., Zhang, G., & Hsu, L. T. (2021). [GNSS outlier mitigation via graduated non-convexity factor graph optimization](https://ieeexplore.ieee.org/abstract/document/9627801). IEEE Transactions on Vehicular Technology, 71(1), 297-310.
  - Zhong, Y., Wen, W., Ng, H. F., Bai, X., & Hsu, L. T. (2022, September). [Real-time Factor Graph Optimization Aided by Graduated Non-convexity Based Outlier Mitigation for Smartphone Decimeter Challenge](https://www.ion.org/publications/abstract.cfm?articleID=18382). In Proceedings of the 35th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2022) (pp. 2339-2348).

*if you use GraphGNSSLib_LEO for your academic research, please cite our related [papers](https://ieeexplore.ieee.org/abstract/document/9562037)*

<p align="center">
  <img width="712pix" src="img/LEO-GNSS FGO.png">
</p>

<center> Software flowchart of GraphGNSSLib_LEO, more information please refer to mannual and paper.</center>


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04, ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation). We only test it on Ubuntu 18.04 with ROS Melodic. 

### 1.2. **Ceres Solver**
Follow the following instructions to install Ceres-solver instead of using the latest version of Ceres-solver.

**Step 1**: Download the [Ceres-solver](https://github.com/Gao-tech1/GraphGNSSLib_LEO/blob/main/support_files) which is compatible with GraphGNSSLib_LEO. 

**Step 2**: make and install
```bash
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# make Ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
sudo make -j4
sudo make test
sudo make install
```

### 1.3. **Extra Libraries**
```bash
sudo apt-get install ros-melodic-novatel-msgs
```
## 2. Build GraphGNSSLib_LEO
Clone the repository and catkin_make:
```bash
mkdir GraphGNSSLib_LEO/src
cd ~/GraphGNSSLib_LEO/src
mkdir result
git clone https://github.com/Gao-tech1/GraphGNSSLib_LEO.git
cd ../
# if you fail in the last catkin_make, please source and catkin_make again
catkin_make
source ~/GraphGNSSLib_LEO/devel/setup.bash
catkin_make
```
(**if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS**)


## 3.Run GNSS-LEO positioning using dataset collected in Whampoa [UrbanNav-HK-Deep-Urban-1](https://github.com/IPNL-POLYU/UrbanNavDataset)   
The GNSS-LEO positioning via SPP and FGO is validated using static dataset collected near Whampoa of Hong Kong. Several parameters are as follows:
  - GPS second span: **455342** to **456880**
  - satellite system: **GPS/BeiDou/Stralink**
  - Window Size: **Batch**
  - measurements considered: pseudorange and Doppler measurements
  - result is saved by default
    ```c++
    $(find global_fusion)/dataset/2021_0521_0607/FGO_trajectoryllh_psr_dop_fusion.csv
    
    ```
 We provide some simulated LEO data, please enjoy it!
   ```bash
  source ~/GraphGNSSLib_LEO/devel/setup.bash
  # read GNSS raw data, LEO raw data and combine them as ROS topic
  # the GNSS-only SPP positioning result will be displayed in rviz
  roslaunch global_fusion data_Whampoa_20210521_GNSS only.launch
  # You can record them as rosbag for usage in the following psr_doppler_fusion
  rosbag record -O gnss_leo_data.bag /gnssLEOmsg_combination_node/GNSS_LEO_PsrCarRov /gnssLEOmsg_combination_node/GNSS_LEO_Dopp_Array
  # You can also record all the topic for the following evaluation
  rosbag record -O GNSS_onlyspp_combine_doppmsg0209.bag -a
  # run pseudorange and doppler fusion
  roslaunch global_fusion data_Whampoa_20210521_GNSSLEO.launch
  ```

<p align="center">
  <img width="712pix" src="img/lightmap-GNSS-LEO.gif">
</p>
<center>
The positioning results with different mathods are displayed in rviz.
</center>



  - GNSS only positioning using SPP with the blue arrow in topic **/gnss_preprocessor_node/WLSENURTKLIB** 
  The result is recorded in `GNSS_only_WLS_result.csv`
  - GNSS only positioning using RTK with the red arrow in topic **/gnss_preprocessor_node/ENUIntegerRTK**
  - GNSS and LEO positioning using SPP with the green arrow in topic **/WLS_spp_psr**
  The result is recorded in `GNSS_LEO_psr_spp_result.csv`
  - GNSS positioning using FGO with purple curve in topic **/FGOGlobalPath**. 
  The result is recorded in `FGO_trajectoryllh_psr_dop_fusion.csv`
  
Please modify the file path for the result to suit your requirements.

<p align="center">
  <img width="712pix" src="img/Whampoa Positioning Results.png">
</p>

Trajectories of three methods (GNSS-only SPP using pseudorange measurements with orange line, GNSS-LEO SPP positioning using pseudorange measurements with yellow line, and GNSS-LEO FGO positioning using pseudorange and doppler shift measurements with purple line. The x-axis and y-axis denote the east and north directions, respectively.


<p align="center">
  <img width="712pix" src="img/Whampoa Positioning Error.png">
</p>

Positioning Error of Different method: Orange dots from GNSS-only SPP, Yellow dots from GNSS-LEO SPP, Purple dots from GNSS-LEO FGO.



## 7. Acknowledgements
We use [Ceres-solver](http://ceres-solver.org/) for non-linear optimization and [RTKLIB](http://www.rtklib.com/) for GNSS data decoding, etc. Some functions are originated from [VINS-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). The [rviz_satellite](https://github.com/nobleo/rviz_satellite) is used for visualization. We based GNSS FGO proposed in [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib) for positioning.

If there is any thing inappropriate, please contact [Yixin GAO](https://polyu-taslab.github.io/members/gao_yixin.html) through yixin.gao@connect.polyu.hk or [Weisong WEN](https://weisongwen.wixsite.com/weisongwen) through welson.wen@polyu.edu.hk.

## 8. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license. We are still working on improving the code reliability. 

For any technical issues, please contact Yixin GAO <yixin.gao@connect.polyu.hk>, from the [Trustworthy AI and Autonomous Systems (TAS) Laboratory]([https://polyu-taslab.github.io/]), The Hong Kong Polytechnic University. 

For commercial inquiries, please contact Weisong WEN <welson.wen@polyu.edu.hk>, from the [Trustworthy AI and Autonomous Systems (TAS) Laboratory]([https://polyu-taslab.github.io/]), The Hong Kong Polytechnic University. 