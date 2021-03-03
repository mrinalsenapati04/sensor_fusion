#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

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
import numpy as np

def talker():
    image_pub = rospy.Publisher('/camera_images', Image, queue_size=10)
    rospy.init_node('publishImage', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()
    f_path='/home/vm/catkin_mrinal/src/sensor_fusion/Dataset/2011_09_26/2011_09_26_drive_0009_sync/image_02/data/0000000360.png'
    cv_image=cv2.imread(f_path)
    while not rospy.is_shutdown():
        
        print(cv_image.shape)
        try:
            img_msg=bridge.cv2_to_imgmsg(cv_image,"bgr8")
        except CvBridgeError as e:
            print('here\n')
        img_msg.header.stamp=rospy.Time.now()
        img_msg.header.frame_id='velodyne'
        image_pub.publish(img_msg)
        print('Image is published!!!!')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass