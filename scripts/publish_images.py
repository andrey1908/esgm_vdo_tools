#!/usr/bin/env python

from glob import glob
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header


class VDO_Image_Publisher:
    def __init__(self,
                 path,
                 topic,
                 encoding,
                 queue,
                 camera_info=None,
                 camera_topic=None):
        self.images_paths = list()
        self.load_images_paths(path)
        self.image_pub = None
        self.camera_pub = None
        self.encoding = encoding
        self.bridge = CvBridge()
        if len(self.images_paths) > 0:
            self.image_pub = rospy.Publisher(topic, Image, queue_size=queue)
            if camera_info is not None and camera_topic is not None:
                self.camera_pub = rospy.Publisher(camera_topic, CameraInfo, queue_size=queue)
                self.camera_info = camera_info

    def load_images_paths(self, path):
        for fname in sorted(glob(path)):
            self.images_paths.append(fname)
        self.images_paths = [self.images_paths[0]] + self.images_paths

    def publish_image(self, header, index):
        if self.image_pub is not None:
            if index < len(self):
                img = cv2.imread(self.images_paths[index], cv2.IMREAD_UNCHANGED)
                msg = self.bridge.cv2_to_imgmsg(img, self.encoding)
            else:
                return
            msg.header = header
            self.image_pub.publish(msg)
            if self.camera_pub is not None:
                self.camera_info.header = header
                self.camera_pub.publish(self.camera_info)

    def __len__(self):
        return len(self.images_paths)


def cam_info_from_proj_mat(P):
    info = CameraInfo()
    P = np.array(P).reshape(3, 4)
    info.K = P[:, :3].ravel().tolist()
    info.P = P.ravel().tolist()
    return info


def publisher():
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.get_param('~rate', 10)
    queue = rospy.get_param('~queue_size', 10)
    frame_id = rospy.get_param('~frame_id', 'stereo')
    fname_left = rospy.get_param('~fname_left')
    fname_right = rospy.get_param('~fname_right')
    encoding = rospy.get_param('~encoding', 'bgr8')
    calib_file = rospy.get_param('~calib_file')

    calib = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
    fx = calib.getNode('Camera.fx').real()
    fy = calib.getNode('Camera.fy').real()
    cx = calib.getNode('Camera.cx').real()
    cy = calib.getNode('Camera.cy').real()
    bf = calib.getNode('Camera.bf').real()
    calib_left = [fx, 0., cx, 0.,
                  0., fy, cy, 0.,
                  0., 0., 1., 0.]
    calib_right = [fx, 0., cx, -bf,
                   0., fy, cy, 0.,
                   0., 0., 1., 0.]
    calib_left = cam_info_from_proj_mat(calib_left)
    calib_right = cam_info_from_proj_mat(calib_right)
    width = calib.getNode('Camera.width').real()
    height = calib.getNode('Camera.height').real()
    calib_left.width = width
    calib_left.height = height
    calib_right.width = width
    calib_right.height = height

    publishers = []
    publishers.append(
        VDO_Image_Publisher(fname_left, '/stereo/left/image_rect', encoding, queue, calib_left, '/stereo/left/camera_info'))
    publishers.append(
        VDO_Image_Publisher(fname_right, '/stereo/right/image_rect', encoding, queue, calib_right, '/stereo/right/camera_info'))

    n = 0
    header = Header(frame_id=frame_id)
    rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        for pub in publishers:
            pub.publish_image(header, n)
        n += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
