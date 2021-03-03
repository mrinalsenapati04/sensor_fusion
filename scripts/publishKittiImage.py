#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from os import listdir
from os.path import isfile, join
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import cv2


# basedir='/home/vm/catkin_build/src/sensor_fusion/Dataset/2011_09_26/2011_09_26_drive_0009_sync/image_02/data'
basedir='/home/vm/catkin_mrinal/src/darknet_ros/darknet_ros/Dataset/2011_09_26/2011_09_26_drive_0009_sync/image_02/data_'

if __name__ == '__main__':

    '''Sample code to publish a pcl2 with python'''
    rospy.init_node('image_publish')
    image_pub = rospy.Publisher("/camera_images", Image)
    # loop_rate=rospy.Rate(10)
    # rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    # rospy.sleep(1.)
    bridge = CvBridge()
    
    for f in sorted(listdir(basedir)) :
    #for i in range (1000):
        
        f_path=basedir+'/'+f
        # print(f_path)
        cv_image=cv2.imread(f_path)
        print(cv_image.shape)
        try:
            img_msg=bridge.cv2_to_imgmsg(cv_image,"bgr8")
        except CvBridgeError as e:
            print('here\n')
        img_msg.header.stamp=rospy.Time.now()
        img_msg.header.frame_id='velodyne'
        image_pub.publish(img_msg)
        print('Image is published!!!!')
        # loop_rate.sleep()





        
        # rospy.sleep(0.01)
