# This script is used to align the time of two topics in a rosbag file.
# The time of the first topic is used to align the time of the second topic.
# The aligned rosbag file is saved as a new file.
# You can reference https://wiki.ros.org/rosbag/Cookbook
# The script is written by Yixin.
# just run the script in the terminal: python rostopic_align.py

#!/usr/bin/env python3

#from importlib import reload
import rospy
import rosbag
import sys
# 设置utf-8编码
if sys.getdefaultencoding() != 'utf-8': 
    reload(sys)
    sys.setdefaultencoding('utf-8')
bag_name = '2025-04-04-23-57-18.bag' # 输入的ROS bag
out_bag_name = 'gnss_leo_msg_0405.bag'  # 输出的ROS bag
# Please change the path to your own absolute path
# add by Yixin
dst_dir = '/home/gao-yixin/下载/软件/Dropbox/GraphGNSSLib_LEO/src/global_fusion/dataset/2021_0521_0607/' 
# 打开输出bag文件用于写入；遍历输入bag文件所有消息
with rosbag.Bag(dst_dir+out_bag_name, 'w') as outbag:
    stamp = None
    for topic, msg, t in rosbag.Bag(dst_dir+bag_name).read_messages():
        if topic == '/leo_raw_publisher_node/LEOPsrCarRov1':
            stamp = msg.header.stamp # 如果消息主题是xxx（流动站数据），则保存该消息的时间戳到stamp变量
        elif topic == '/leo_raw_publisher_node/LEOPsrCarStation1' and stamp is not None: 
            outbag.write(topic, msg, stamp) # 如果详细主题的xxx（基准站数据），则使用之前保存的时间戳写入消息
            continue
        outbag.write(topic, msg, msg.header.stamp) # 对于其他所有消息，使用消息本身的时间戳写入

print("finished")
