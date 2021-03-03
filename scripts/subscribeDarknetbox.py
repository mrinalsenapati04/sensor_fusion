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
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

def callback(data):
    print("inside callback")
    bbox_arr=data
    print(bbox_arr.header.stamp.secs)
    for bbox in bbox_arr.bounding_boxes:
        rospy.loginfo("Class: {}, Probability: {}, Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(bbox.Class,bbox.probability, bbox.xmin, bbox.xmax, bbox.ymin, bbox.ymax)
        )
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_bbox', anonymous=True)
    print("inside main")

    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


