#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 29 11:23:50 2021

@author: vm
"""

import os
import rospy
import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# print(sys.path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from utils import *
import open3d as o3d





def render_image_with_boxes(img, objects, calib):
    """
    Show image with 3D boxes
    """
    # projection matrix
    P_rect2cam2 = calib['P2'].reshape((3, 4))

    img1 = np.copy(img)
    for obj in objects:
        if obj.type == 'DontCare':
            continue
        box3d_pixelcoord = map_box_to_image(obj, P_rect2cam2)
        img1 = draw_projected_box3d(img1, box3d_pixelcoord)

    cv2.imwrite('box_img.jpg', img1)
    plt.imshow(img1)
    plt.yticks([])
    plt.xticks([])
    plt.show()
    
    
    
def render_lidar_with_boxes(pc_velo, objects, calib, img_width, img_height):
    # projection matrix (project from velo2cam2)
    proj_velo2cam2 = project_velo_to_cam2(calib)

    # apply projection
    pts_2d = project_to_image(pc_velo.transpose(), proj_velo2cam2)

    # Filter lidar points to be within image FOV
    inds = np.where((pts_2d[0, :] < img_width) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < img_height) & (pts_2d[1, :] >= 0) &
                    (pc_velo[:, 0] > 0)
                    )[0]
    imgfov_pc_velo = pc_velo[inds, :]

    fig = mlab.figure(figure=None, bgcolor=(0, 0, 0),
                      fgcolor=None, engine=None, size=(1000, 500))

    draw_lidar(imgfov_pc_velo, fig=fig)

    # Projection matrix
    proj_cam2_2_velo = project_cam2_to_velo(calib)

    # Draw objects on lidar
    for obj in objects:
        if obj.type == 'DontCare':
            continue
        # Project boxes from camera to lidar coordinate
        boxes3d_pts = project_camera_to_lidar(obj.in_camera_coordinate(), proj_cam2_2_velo)

        # Draw boxes
        draw_gt_boxes3d(boxes3d_pts, fig=fig)
    mlab.show()

def render_lidar_on_image(pts_velo, img, calib, img_width, img_height,proj_velo2cam2):
    # projection matrix (project from velo2cam2)
    # proj_velo2cam2 = project_velo_to_cam2(calib)
#    print(proj_velo2cam2)

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
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(imgfov_pc_velo)
    o3d.io.write_point_cloud('new_pcd.pcd', pcd)
    print('pcd file is saved')

    imgfov_pc_velo = np.hstack((imgfov_pc_velo, np.ones((imgfov_pc_velo.shape[0], 1))))
    imgfov_pc_cam2 = proj_velo2cam2 @ imgfov_pc_velo.transpose()

    cmap = plt.cm.get_cmap('hsv', 256)
    cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255

    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        color = cmap[int(640.0 / depth), :]
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i])),
                         int(np.round(imgfov_pc_pixel[1, i]))),
                   2, color=tuple(color), thickness=-1)
    
    cv2.imwrite('/home/vm/catkin_mrinal/src/sensor_fusion/lidar_img.jpg', img)
    # plt.imshow(img)
    # plt.yticks([])
    # plt.xticks([])
    # plt.show()
    # return img

if __name__ == '__main__':
    

#    rgb = cv2.cvtColor(cv2.imread(os.path.join('data/000114_image.png')), cv2.COLOR_BGR2RGB)
#    img_height, img_width, img_channel = rgb.shape
#
#    calib = read_calib_file('data/000114_calib.txt')
#    print(calib)
#
#    pc_velo = load_velo_scan('data/000114.bin')[:, :3]
#    print('shape of velo',pc_velo.shape)
#    
#    render_lidar_on_image(pc_velo, rgb, calib, img_width, img_height)
    rospy.init_node('projection')
    

    rgb = cv2.cvtColor(cv2.imread(os.path.join('/home/vm/catkin_mrinal/src/sensor_fusion/Dataset/2011_09_26/2011_09_26_drive_0009_sync/image_02/data/0000000381.png')), cv2.COLOR_BGR2RGB)

    
    img_height, img_width, img_channel = rgb.shape 
    # print(rgb.shape)
    calib = read_calib_file('/home/vm/catkin_mrinal/src/sensor_fusion/Dataset/2011_09_26/calib.txt')
    # print(calib)
    
    pc_velo = load_velo_scan('/home/vm/catkin_mrinal/src/sensor_fusion/Dataset/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data/0000000381.bin')[:, :3]
#    print('shape of velo',pc_velo.shape)
    proj_velo2cam2 = project_velo_to_cam2(calib)

    start=rospy.Time.now()
    # render_lidar_on_image(pc_velo, rgb, calib, img_width, img_height)
    render_lidar_on_image(pc_velo, rgb, calib, img_width, img_height,proj_velo2cam2)
    end=rospy.Time.now()
    print('Time taken for projection ', (end-start).to_sec())
    
    

    