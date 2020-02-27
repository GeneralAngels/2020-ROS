#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

import numpy as np
import cv2 as cv
from math import *

from FRC_utils.math_utils import get_dis, get_center_point
from FRC_utils.ros_utils import get_config, create_pose
from FRC_utils.image_proccessing_utils import order_points, LogitechC922, show_images, publish_image, \
    get_mass_dir, get_x, get_y, draw_axis, get_camera_by_name


def target_pose_estimation(mat,
                           model_points,
                           dist_coeffs,
                           camera_matrix,
                           is_view_image=False,
                           is_publish_image=False,
                           image_publisher=None,
                           target_pose_publisher=None,
                           rotation_publisher=None):
    """
    Find the pose of a target relatively to the camera.
    sets the rosparam '/target_in_frame' if the target appears in the mat.
    publishes the pose of the target under the name '/target_pose'.

    :param mat: RGB mat.
    :param model_points: The points of the target in real space ([ [x, y, z], [x, y, z], ... ]
    :param dist_coeffs: Distortion coeff matrix from camera calibration.
    :param camera_matrix: Camera calibration matrix.
    :param is_view_image: bool - for showing the image.
    :param is_publish_image: bool - for publishing the image.
    :param image_publisher: ROS image publisher.
    :param target_pose_publisher: ROS Target pose publisher ([x, y, z, roll, pitch, yaw])
    :param rotation_publisher: Float32 publisher for the turret rotation.
    :return: None
    """
    if mat is None:
        return None

    # declarations.
    success = False
    ROI = None
    target_pose = None
    rotation = 0.0

    # rotate mat 180 degrees
    mat = cv.rotate(mat, cv.ROTATE_180)

    # transforms the image to HSV and grayscale
    hsv = cv.cvtColor(mat, cv.COLOR_RGB2HSV)
    gray = cv.cvtColor(mat, cv.COLOR_RGB2GRAY)

    # blurs the image
    blur = cv.GaussianBlur(gray, (3, 3), 0)

    # find the brightest point on the mat
    minVal, maxVal, minLoc, maxLoc = cv.minMaxLoc(blur)

    # filters out the unwanted colors.
    lower_thresh = np.array([0, 0, 0])
    upper_thresh = np.array([255, 255, 112])
    # mask = cv.bitwise_not(cv.inRange(hsv, lower_thresh, upper_thresh))  # TODO: Calibrate mask thresholds
    mask = cv.inRange(hsv, (36, 25, 40), (100, 255, 255))

    # threshold and contours
    ret, thresh = cv.threshold(mask, 20, 255, 0)
    contours, hierarchy = cv.findContours(thresh, 1, 2)

    # finds the target using check_shape()
    for cnt in contours:
        test, box = check_shape(cnt, maxLoc)
        if test:
            cv.drawContours(mat, [cnt], -1, (255, 255, 255))
            box = order_points(box)
            offsets = [box[0][0] - 10, box[0][1] - 10]
            height = np.size(mask, 0)
            width = np.size(mask, 1)
            ROI = np.zeros((height, width))
            cv.fillPoly(ROI, [cnt], (255, 255, 255))
            ROI = ROI[(box[0][1] - 10): (box[2][1] + 10), (box[0][0] - 10): (box[2][0] + 10)]

            test, temp, key_points = find_key_points_2(ROI, offsets)
            if not test:
                continue

            center_point_2d = get_center_point(key_points[0], key_points[-1])
            for p in key_points:
                cv.circle(mat, (int(p[0]), int(p[1])), 3, (255, 0, 255), -1)

            (success, rotation_vector, translation_vector) = cv.solvePnP(model_points,
                                                                         key_points,
                                                                         camera_matrix,
                                                                         dist_coeffs)

            if success:
                drawn, center_point_3d = draw_axis(mat, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
                dis = get_dis(center_point_2d, center_point_3d)
                euler = rotation_to_euler(rotation_vector)
                rotation = ((center_point_2d[0] / width) - 0.5) * (-1.2)

                if success and (dis < 6):
                    mat = drawn
                target_pose = [i[0] for i in rotation_vector] + euler
                break

    if is_view_image:
        show_images([mat, thresh, ROI])

    if is_publish_image:
        publish_image(mat, image_publisher)
        rospy.set_param('target_in_frame', success)
        rotation_publisher.publish(float(rotation))

        if success:
            pose = create_pose(target_pose, '/target_pose')
            target_pose_publisher.publish(pose)


def rotation_to_euler(rotation_vector):
    R = cv.Rodrigues(rotation_vector)[0]
    roll = 180 * atan2(-R[2][1], R[2][2]) / pi
    pitch = 180 * asin(R[2][0]) / pi
    yaw = 180 * atan2(-R[1][0], R[0][0]) / pi
    rot_params = [radians(roll), radians(pitch), radians(yaw)]
    return rot_params


def check_location(point, cnt):
    return cv.pointPolygonTest(cnt, point, False)


def check_shape(contour, max_point):
    max_test = cv.pointPolygonTest(contour, max_point, False)
    box, mass_dir = get_mass_dir(contour)

    area = cv.contourArea(box)
    if area == 0:
        area_test = False
    else:
        area_test = 0.23 > (cv.contourArea(contour) / area)
    ratio_test = 0

    x_t = get_dis(box[0], box[1])
    y_t = get_dis(box[1], box[2])
    if y_t != 0:
        ratio = x_t / y_t
        ratio_test = (ratio > 1.85 or ratio < 0.75) and x_t > 20 and y_t > 40
    test = area_test and max_test and (-1 < mass_dir < 1) and (area > 1000) and ratio_test
    return test, box


def find_key_points_2(gray, offsets):
    if np.size(gray, 0) < 20 or np.size(gray, 1) < 20:
        return False, gray, None

    gray = cv.GaussianBlur(gray, (3, 3), 0)

    gray = gray.astype('uint8') * 255
    corners = cv.goodFeaturesToTrack(gray, 4, 0.000001, 30, blockSize=15)
    corners = np.int0(corners)

    # noinspection PyTypeChecker
    res = [[int(p[0][0] + offsets[0]), int(p[0][1] + offsets[1])] for p in corners]
    temp = sorted(res, key=get_y)[-2:]
    extremes = [tuple(min(res, key=get_x)), tuple(max(res, key=get_x)), tuple(max(temp, key=get_x)),
                tuple(min(temp, key=get_x))]
    extremes.sort(key=get_x)

    return True, gray, np.array(extremes, dtype="double")


def find_key_points(gray, offsets):
    if np.size(gray, 0) < 20 or np.size(gray, 1) < 20:
        return False, gray, None
    # corner detection
    gray = np.float32(gray)
    gray = cv.GaussianBlur(gray, (3, 3), 3)
    dst = cv.cornerHarris(gray, 2, 3, 0.01)
    ret, dst = cv.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 250, 0.005)
    res = cv.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)

    # include offset
    res = [[int(p[0] + offsets[0]), int(p[1] + offsets[1])] for p in res]

    # sort and filter the key points
    temp = sorted(res, key=get_y)[-2:]
    extremes = [tuple(min(res, key=get_x)), tuple(max(res, key=get_x)), tuple(max(temp, key=get_x)),
                tuple(min(temp, key=get_x))]
    extremes.sort(key=get_x)

    return True, dst, np.array(extremes, dtype="double")


def main():
    # node setup
    config = get_config()
    rospy.init_node('reflective_tape_detector')
    np.set_printoptions(suppress=True)
    rate = rospy.Rate(20)

    # ROS Subscribers, publishers and params setup
    rospy.set_param('target_in_frame', False)
    image_pub = rospy.Publisher('/camera/ir/processed', Image, queue_size=30)
    target_pose_publisher = rospy.Publisher('/target/position', PoseStamped, queue_size=5)
    rotation_publisher = rospy.Publisher('/target/rotation', Float32, queue_size=5)

    # camera configuration
    camera_config = config['cameras']['logitech_c922']
    camera = LogitechC922(camera_config)
    calibration_mat = np.array(camera_config['calibration_matrix'])
    distortion = np.array(camera_config['dist_coeff'])
    port_points = np.array([tuple(i) for i in config['field']['outer_port_outline']])  # The real location of the port

    # processing
    while not rospy.is_shutdown():
        target_pose_estimation(camera.mat, port_points,
                               dist_coeffs=distortion,
                               camera_matrix=calibration_mat,
                               is_view_image=False,
                               is_publish_image=True,
                               image_publisher=image_pub,
                               target_pose_publisher=target_pose_publisher,
                               rotation_publisher=rotation_publisher)

        rate.sleep()
    camera.close()


if __name__ == "__main__":
    main()
