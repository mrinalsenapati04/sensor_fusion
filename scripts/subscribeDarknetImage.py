#!/usr/bin/env python2
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
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String


basedir='/home/vm/catkin_mrinal/src/darknet_ros/darknet_ros/Dataset/2011_09_26/2011_09_26_drive_0009_sync/detected_image02/'
file_no=0000000000
def callback(data):
    # gloabal file_no
    print(data.header.stamp.secs)
    global file_no
    bridge = CvBridge()
    # rospy.loginfo(rospy.get_caller_id() + "I heard")
    try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # cv_image=bridge.imgmsg_to_cv2(data,"bgr8")
        cvimage=bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)

    (rows,cols,channels) = cvimage.shape
    print(cvimage.shape)
    filename=basedir+ str(file_no)+'.png'
    # print(filename)
    # cv2.imwrite(filename, cvimage) 
    # file_no=file_no+1
    file_no=file_no+1
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_bbox', anonymous=True)
    print("inside main")

    rospy.Subscriber('/darknet_ros/detection_image', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # file_no=0000000000
    listener()


