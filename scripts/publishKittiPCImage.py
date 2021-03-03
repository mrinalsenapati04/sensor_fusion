#!/usr/bin/env python
import rospy
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
import cv2

import numpy as np
import rospy
import sensor_msgs.msg as sensor_msgs
import ros_numpy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header




# basedir='/home/vm/catkin_build/src/sensor_fusion/Dataset/2011_09_26/2011_09_26_drive_0009_sync/image_02/data'
basedir='/home/vm/catkin_mrinal/src/sensor_fusion/Dataset/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data'

if __name__ == '__main__':

    '''Sample code to publish a pcl2 with python'''
    rospy.init_node('pc_publish')
    pc_pub = rospy.Publisher("/kitti_pc", PointCloud2,queue_size=2)
    image_pub = rospy.Publisher("/camera_images", Image)
    loop_rate=rospy.Rate(2) 
    bridge = CvBridge()
    for f in sorted(listdir(basedir)) :
        start=rospy.Time.now()
        f_path=basedir+'/'+f
        print(f_path)
        i_path=f_path.split('velodyne_points')[0]
        # print(i_path)
        f_no=f_path.split('velodyne_points')[1].split('bin')[0]
        # print(f_no)
        i_path=i_path+'image_02'+f_no+'png'
        print(i_path)
      
        pc_velo = np.fromfile(f_path, dtype=np.float32).reshape((-1, 4))
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]
        header= Header()
        # header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        pc_msg = point_cloud2.create_cloud(header, fields, pc_velo)
        
        # end=rospy.Time.now()
        # print('Time taken for publishing ', (end-start).to_sec())


        cv_image=cv2.imread(i_path)
        print(cv_image.shape)
        # try:
        img_msg=bridge.cv2_to_imgmsg(cv_image,"bgr8")
        # except CvBridgeError as e:
            # print('here\n')
        


        img_msg.header.frame_id='velodyne'
        
        pc_msg.header.stamp=rospy.Time.now()
        img_msg.header.stamp=rospy.Time.now()
        
        pc_pub.publish(pc_msg)
        image_pub.publish(img_msg)
        
        print('data is published!!!')
        print('Image is published!!!!')
        loop_rate.sleep()


        
        
