#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from os import listdir
from os.path import isfile, join
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import copy
import numpy as np
import open3d as o3d

filename='/home/vm/catkin_build/src/0000000000.txt'

with open(filename) as f:
    lines = f.readlines()
i=0
pts=[]
for line in lines:
    # print(line)
    line =line.split(' ')
    # print(line[3][:-1])
    # pts[i][0]=float(line[0])
    # pts[i][1]=float(line[1])
    # pts[i][2]=float(line[2])
    # pts[i][3]=float(line[3][:-1])
    point=[float(line[0]),float(line[1]),float(line[2])]
    pts.append(point)
    i=i+1

pts = np.asarray(pts, dtype=np.float32)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)
o3d.io.write_point_cloud("/home/vm/catkin_build/src/0000000000.pcd", pcd)

