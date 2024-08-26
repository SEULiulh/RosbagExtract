#!/home/txb/anaconda3/envs/pytorch1_8_0/bin/python
#coding:utf-8

'''
Author: Tang
Date: 2023-13-13 20:36:53
LastEditTime: 2023-12-07 17:38:04
LastEditors: your name
Description: Converting rosbag to RGB, depth image(.png) and imu data(.txt)
'''

'''
深度图转换问题：
https://blog.csdn.net/zxxxiazai/article/details/111616502
'''

import roslib;  
import rosbag
import rospy
import cv2
import numpy as np
import tifffile as tiff
from sensor_msgs.msg import Image
import sys
# sys.path.insert(0,"/home/txb/cv_bridge/install/lib/python3/dist-packages/")
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import os
 
root_path="/media/llh/FrenzyLLH/VCU_RVI_handheld/motion/motion_1/data/Mydata/" #存放图片的位置
#for zed camera
#rgb_topic = "/zed2/zed_node/left_raw/image_raw_color"
#depth_topic = "/zed2/zed_node/depth/depth_registered"
#imu_topic = "/zed2/zed_node/imu/data"
#for d435i camera
# rgb_topic = "/camera/color/image_raw"
# depth_topic = "/camera/depth/image_rect_raw"
#for EuRoC Dataset
rgb_topic = "/cam0/color"
depth_topic = "/cam0/depth"
imu_topic = "/imu"

class ImageCreator():
    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('/media/llh/FrenzyLLH/VCU_RVI_handheld/motion/motion_1/data/lab-motion1.bag', 'r') as bag:   #要读取的bag文件；
            ###RGB image topic###
            # rgb_path = os.path.join(root_path,"rgb/")
            rgb_path = root_path + "rgb/"
            for topic,msg,t in bag.read_messages():
                if topic == rgb_topic:  #图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")    # bgr8
                    except CvBridgeError as e:
                        print(e)
                    timeq = msg.header.stamp.to_sec() 
                    timestr = "{:.6f}".format(timeq)
                    #%.6f表示小数点后带有6位，可根据精确度需要修改；"%.6f" %  msg.header.stamp.to_sec() * pow(10, 9)
                    image_name = timestr+ ".png" #图像命名：时间戳.png
                    cv2.imwrite(rgb_path+image_name, cv_image)  #保存；
            print("RGB done.")
            
            ###Depth image topic###
            # depth_path = os.path.join(root_path,"/scenes/13/depth/")
            depth_path = root_path + "depth/"
            for topic,msg,t in bag.read_messages():
                if topic == depth_topic:  #图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"passthrough") #passthrough
                    except CvBridgeError as e:
                        print(e)
                    # cv_image = cv_image * 255
                    timeq = msg.header.stamp.to_sec() 
                    timestr = "{:.6f}".format(timeq)
                    #%.6f表示小数点后带有6位，可根据精确度需要修改；"%.6f" %  msg.header.stamp.to_sec() * pow(10, 9)
                    image_name = timestr + ".tif" #图像命名：时间戳.png
                    tiff.imwrite(depth_path+image_name, cv_image)  #保存；
            print("Depth done.")
            
            #IMU topic###
            # imu_path = os.path.join(root_path,"/scenes/13/")
            imu_path = root_path+"imu0.txt"
            imu = open(imu_path,'w')
            for topic,msg,t in bag.read_messages():
                if topic == imu_topic : #imu topic；
                    acc_y = "%.6f" % msg.linear_acceleration.y
                    acc_x = "%.6f" % msg.linear_acceleration.x
                    acc_z = "%.6f" % msg.linear_acceleration.z
                    w_y = "%.6f" % msg.angular_velocity.y
                    w_x = "%.6f" % msg.angular_velocity.x
                    w_z = "%.6f" % msg.angular_velocity.z
                    timeimu = "%.6f" %  msg.header.stamp.to_sec()
                    imudata = timeimu + "," + w_x + "," + w_y + "," + w_z + "," + acc_x + "," + acc_y + "," + acc_z
                    imu.write(imudata)
                    imu.write('\n')
            imu.close()
            print("IMU done.")
 
 
if __name__ == '__main__':
 
    #rospy.init_node(PKG)
 
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
