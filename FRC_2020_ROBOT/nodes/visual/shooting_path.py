#!/usr/bin/env python

import rospy, tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *

import time
from FRC_utils.ros_utils import make_sphere_marker
from FRC_utils.ros_utils import get_config
from FRC_utils.ballistics import BallisticsGraph


class PosesCallback:
    def __init__(self):
        self.start_point = None

    def update_start_point(self, data):
        self.start_point = data


def main():
    config = get_config()

    rospy.init_node('shooting_path_calc')
    rate = rospy.Rate(40)

    rospy.set_param('inner', 'none')
    rospy.set_param('outer', 'none')

    poses = PosesCallback()
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, poses.update_start_point)

    ball_path_publisher = rospy.Publisher('/quad_path', Path, queue_size=20)
    ball_publisher = rospy.Publisher('/ball', Marker, queue_size=10)
    ball_radius = float(config['field']['ball']['radius']) / 100.0

    inner_port = [x[1] / 100.0 for x in config['field']['inner_port'].items()[:3]]
    outer_port = [x[1] / 100.0 for x in config['field']['outer_port'].items()[:3]]
    shooter_height = config['robot']['shooter']['z'] / 100.0

    start_time = time.time()
    last_x_y_z = [0, 0, 0]
    while not rospy.is_shutdown():
        start_point = poses.start_point

        if start_point is not None:
            x, y, z = start_point.pose.position.x, start_point.pose.position.y, shooter_height
            bg = BallisticsGraph([x, y, z], inner_port)
            if [x, y, z] != last_x_y_z:
                start_time = time.time()
                print 'shooting from ', [x, y, z], ' to target at ', inner_port
                print 'velocity: ', bg.velocity
                print 'alpha: ', bg.alpha

            ball_path = bg.get_ros_path()

            ball_location = bg.get_location_at_time(time.time() - start_time - 3)
            ball = make_sphere_marker(ball_location, [ball_radius * 2 for i in range(3)], '/ball', [255, 255, 0])
            ball_publisher.publish(ball)
            ball_path_publisher.publish(ball_path)
            rate.sleep()

            last_x_y_z = [x, y, z]


if __name__ == "__main__":
    main()
