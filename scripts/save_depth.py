#!/usr/bin/env python

import numpy as np
import cv2
from PIL import Image
import os
import os.path as osp
from tqdm import tqdm
import rospy
from sensor_msgs.msg import Image as ROS_Image
from cv_bridge import CvBridge


class VDO_Depth_Saver:
    def __init__(self, out_folder, bf, DepthMapFactor):
        self.out_folder = out_folder
        self.bf = bf
        self.DepthMapFactor = DepthMapFactor
        self.n = 0
        self.bridge = CvBridge()
        self.listener = rospy.Subscriber("/stereo/depth", ROS_Image, self.save_depth)

    def save_depth(self, depth_image):
        ar = self.bridge.imgmsg_to_cv2(depth_image, '32FC1')
        ar[np.isnan(ar) | np.isinf(ar)] = 0
        ar[ar > 0] = self.bf * self.DepthMapFactor / ar[ar > 0]
        ar = ar.astype(np.int32)
        im = Image.fromarray(ar)
        im.save(osp.join(self.out_folder, str(self.n).zfill(6) + '.png'))
        self.n += 1


if __name__ == '__main__':
    rospy.init_node('save_depth_vdo', anonymous=True)
    out_folder = rospy.get_param('~out_folder')
    calib_file = rospy.get_param('~calib_file')
    calib = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
    bf = calib.getNode('Camera.bf').real()
    DepthMapFactor = calib.getNode('DepthMapFactor').real()
    depth_saver = VDO_Depth_Saver(out_folder, bf, DepthMapFactor)
    rospy.spin()

