#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import sys
import rospy
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes

import message_filters
from std_msgs.msg import Int32, Float32
# from utils import *
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

bridge = CvBridge()
image_pub = rospy.Publisher("/new_images", Image,queue_size=10)
calib = read_calib_file('/home/vm/catkin_mrinal/src/sensor_fusion/Dataset/2011_09_26/calib.txt')

proj_velo2cam2 = project_velo_to_cam2(calib)

def render_lidar_on_image(pts_velo,img_width, img_height,proj_velo2cam2):
    # apply projection
    pts_2d = project_to_image(pts_velo.transpose(), proj_velo2cam2)
#    print(pts_2d.shape)
    # Filter lidar points to be within image FOV
    inds = np.where((pts_2d[0, :] < img_width) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < img_height) & (pts_2d[1, :] >= 0) &
                    (pts_velo[:, 0] > 0))[0]

    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]

    # Retrieve depth from lidar
    imgfov_pc_velo = pts_velo[inds, :]
    print(imgfov_pc_velo.shape)

    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(imgfov_pc_velo)
    # o3d.io.write_point_cloud('new_pcd.pcd', pcd)





def callback(image_data,pc_data):
    print('image time stamp: ',image_data.header.stamp.secs)
    print('pc_data time stamp: ',pc_data.header.stamp.secs)

    image=CvBridge().imgmsg_to_cv2(image_data,"bgr8")
    img_height, img_width, img_channel = image.shape 
    
    print(image.shape)
    points_list = []

    for data in point_cloud2.read_points(pc_data, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])
    
    img_height, img_width, img_channel = image.shape 
    render_lidar_on_image(points_list,img_width,img_height)
    



def listener():


    rospy.init_node('sub_ImageBbox', anonymous=True)
    

    # spin() simply keeps python from exiting until this node is stopped
    image_sub=message_filters.Subscriber('/camera_images', Image)
    pc_sub=message_filters.Subscriber('/kitti_pc', PointCloud2)
   
    
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pc_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
   
    listener()
