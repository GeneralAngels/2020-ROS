#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2 as cv
from threading import Thread

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# from utills.geometry_utills import calc_distance, calc_center_point

import ConfigParser


class RealSenseHandler(Thread):
    MAT_SIZE = (640, 480)  # PIXELS * PIXELS
    FRAME_RATE = 30  # FPS

    def __init__(self, config):
        Thread.__init__(self)
        self.config = config
        print(config.sections())

        self.running = False
        self.pipeline = rs.pipeline()

        self.depth = None
        self.rgb = None

        self.cv_bride = CvBridge()

    def set_config(self, mat_size=(640, 480), frame_rate=30):
        self.MAT_SIZE = mat_size
        self.FRAME_RATE = frame_rate

        config = rs.config()
        config.enable_stream(rs.stream.depth, self.MAT_SIZE[0], self.MAT_SIZE[1], rs.format.z16, self.FRAME_RATE)
        config.enable_stream(rs.stream.color, self.MAT_SIZE[0], self.MAT_SIZE[1], rs.format.bgr8, self.FRAME_RATE)

    def run(self):
        self.running = True
        self.pipeline.start()

        while self.running:
            frames = self.pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            rgb = frames.get_color_frame()

            self.depth = np.asanyarray(depth)
            self.rgb = np.asanyarray(rgb)

    def get_frames(self):
        return self.rgb, self.depth

    def get_ros_msg(self):
        rgb_msg = self.cv_bride.cv2_to_imgmsg(self.rgb)
        depth_msg = self.cv_bride.cv2_to_imgmsg(self.depth)
        return rgb_msg, depth_msg

    def stop(self):
        self.running = False
        self.pipeline.stop()


def detect_ball(frame, display):
    """
    input: RGB opencv mat
    output: x, y, radius of ball in pixels.
    """
    # Basic processing
    frame_ = cv.resize(frame, (320, 240))
    hsv = cv.cvtColor(frame_, cv.COLOR_RGB2HSV)
    frame_gray = cv.cvtColor(hsv, cv.COLOR_BGR2GRAY)

    # Detecting using opencv-haar-cascade-classifier
    ball_cascade = 'ball_cascade.xml'
    cascade_classifier = cv.CascadeClassifier()
    if not cascade_classifier.load(cv.samples.findFile(ball_cascade)):
        print('cascade not found')

    balls = cascade_classifier.detectMultiScale(frame_gray,
                                                scaleFactor=1.1,
                                                minNeighbors=5,
                                                minSize=(16, 16),
                                                flags=cv.CASCADE_SCALE_IMAGE)

    if len(balls) == 0:
        return None

    calc_area = lambda radius: float(((radius[3] + radius[2]) / 4) ** 2) * np.pi
    ball = max(balls, key=calc_area)

    print(ball)


def main():
    # TODO: READ CONFIG.JSON FILE

    # init ros node
    rospy.init_node('main_image_processing')
    rate = rospy.Rate(30)

    # setup publishers, subscribers and params
    rgb_publisher = rospy.Publisher("rgb_image", Image, queue_size=10)
    depth_publisher = rospy.Publisher("depth_image", Image, queue_size=10)

    # start camera thread
    rs_handler = RealSenseHandler(config)
    rs_handler.start()

    while not rospy.is_shutdown():
        # TODO: processing here

        rgb_msg, depth_msg = rs_handler.get_ros_msg()
        rgb_publisher.publish(rgb_msg)
        depth_publisher.publish(depth_msg)

        rate.sleep()


if __name__ == '__main__':
    main()
