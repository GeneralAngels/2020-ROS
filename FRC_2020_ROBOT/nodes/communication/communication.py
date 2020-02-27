#!/usr/bin/env python

import socket
import json
from threading import Thread
import tf
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from FRC_utils.ros_utils import get_config, arrow_from_pose
from FRC_utils.ballistics import BallisticsGraph
from math import *
import time


class Communication(Thread):
    def __init__(self, config, is_logging=True):
        super(Communication, self).__init__()
        self.log = is_logging
        self.config = config

        # Connectivity configuration
        self.rate = int(config['networking']['rate'])
        self.ip = str(config['networking']['RIO_IP'])
        self.port = int(config['networking']['RIO_PORT'])
        self.buffer_size = int(config['networking']['BUFFER_SIZE'])
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Declarations
        self.items = None
        self.last_receive = None
        self.is_running = False
        self.rem = ''

        self.connect()

    def set_items(self, items):
        self.items = items

    def connect(self):
        if self.log:
            print('trying to connect to ' + str(self.ip) + ' on port ' + str(self.port) + '...')

        self.socket.connect((self.ip, self.port))

        if self.log:
            print('connected.')
        self.is_running = True

    def send_cmd(self, cmd):
        if self.log:
            print('sending: ' + str(cmd))
        self.socket.send(str(cmd) + '\n')

    def receive(self, callback):
        buff = self.rem
        while '\n' not in buff:
            buff += self.socket.recv(self.buffer_size)
        buff, self.rem = buff.split('\n')

        if callback is not None:
            callback(buff)

        if self.log:
            print('received: ' + str(buff))
        self.last_receive = buff

    def run(self):
        self.is_running = True

        while self.is_running:
            if self.items is None:
                self.stop()

            for generate_cmd, callback in self.items:
                cmd = generate_cmd()
                self.send_cmd(cmd)
                self.receive(callback)
                time.sleep(1.0 / self.rate)

    def stop(self):
        self.is_running = False


class CommunicationHandler:
    """
    items:
    {
        generate_cmd: subscriber_callback,  # for sending data
        generate_cmd: data_handler          # for receiving data
        #test
    }
    """
    rotation = 0
    shooter_velocity = 0
    last_shooter_velocity = 0
    hood = 0
    last_hood = 0
    shooter_alpha = 0.1
    c = 0
    sum = 0

    def __init__(self):
        # Communication setup:
        config = get_config()
        self.communication = Communication(config, is_logging=False)

        self.items = [
            # (self.generate_rotation_cmd, None),  # for sending the rotation velocity of the turret
            (self.generate_odometry_cmd, self.odometry_handler),  # for receiving the odometry from the robot
            # (self.generate_hood_cmd, None),  # for the angle of the hood
            # (self.generate_shooter_velocity_cmd, None)  # for the velocity of the shooter
            # (self.generate_shooter_setpoint_cmd, None)
            (self.generate_setpoints_cmd, None)
        ]

        rospy.set_param('target_in_frame', False)
        # Subscribers:
        rospy.Subscriber('/target/rotation', Float32, callback=self.rotation_sub_callback)
        rospy.Subscriber('/target/position', PoseStamped, callback=self.pose_sub_callback)

        # Publishers:
        self.odometry_publisher = rospy.Publisher('/robot_odometry', Odometry, queue_size=100)
        self.robot_arrow_publisher = rospy.Publisher('/robot_location', Marker, queue_size=10)

        self.communication.set_items(self.items)

    @staticmethod
    def generate_odometry_cmd():
        return 'odometry json'

    def odometry_handler(self, data):
        data = json.loads(data)
        if len(data.items()) == 0:
            return None

        x = float(data['x'])
        y = float(data['y'])
        distance = float(data['distance'])
        theta = radians(float(data['theta']))
        current_time = rospy.Time.now()

        odom_broadcaster = tf.TransformBroadcaster()
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = '/robot_odometry'
        odom.child_frame_id = '/base_link'

        odom_quat = tf.transformations.quaternion_from_euler(distance, 0, theta + pi / 2)
        odom_quat_tf = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
        odom.pose.pose = pose

        arrow = arrow_from_pose(pose, [255, 0, 0], '/world', 'robot_location')
        self.robot_arrow_publisher.publish(arrow)
        odom_broadcaster.sendTransform((x, y, 0), odom_quat_tf, current_time, '/base_link', '/robot_odometry')
        self.odometry_publisher.publish(odom)

    def generate_rotation_cmd(self):
        in_frame = rospy.get_param('target_in_frame')

        cmd = 'shooter camera 0'
        if in_frame:
            cmd = 'shooter camera ' + str(self.rotation)
        return cmd

    def generate_hood_cmd(self):
        in_frame = rospy.get_param('target_in_frame')

        cmd = 'shooter hood ' + str(self.last_hood)
        if in_frame:
            h = (self.hood * self.shooter_velocity) + ((1 - self.shooter_alpha) * self.last_hood)
            cmd = 'shooter hood ' + str(h)
        self.last_hood = self.hood
        return cmd

    def generate_shooter_velocity_cmd(self):
        in_frame = rospy.get_param('target_in_frame')

        cmd = 'shooter shooter ' + str(self.last_shooter_velocity)
        if in_frame:
            vel = (self.shooter_alpha * self.shooter_velocity) + ((1 - self.shooter_alpha) * self.last_shooter_velocity)
            cmd = 'shooter shooter ' + str(vel)
        self.last_shooter_velocity = self.shooter_velocity
        return cmd

    def generate_shooter_setpoint_cmd(self):
        cmd = 'robot shooter ' + str(self.shooter_velocity)
        return cmd

    def rotation_sub_callback(self, data):
        self.rotation = data.data

    def pose_sub_callback(self, data):
        x = data.pose.position.x * cos(radians(25)) * 1.7
        y = data.pose.position.y * sin(radians(25)) + abs(data.pose.position.x) * 1.7 * sin(radians(25))
        target_point = [abs(x), data.pose.position.z, y]
        # print('target: ' + str(target_point))
        bg = BallisticsGraph([0, 0, 0], target_point)
        self.shooter_velocity = abs(bg.velocity)
        self.hood = abs(degrees(bg.alpha))
        # print('v: ' + str(self.shooter_velocity) + ' a: ' + str(self.hood))

    def generate_setpoints_cmd(self):
        in_frame = rospy.get_param('target_in_frame')
        if in_frame:
            if abs(self.last_shooter_velocity - self.shooter_velocity) > 22.0:
                self.sum = 0
                self.c = 0

            self.sum += self.shooter_velocity
            self.c += 1
            v = float(self.sum) / float(self.c)
            # v = (v * v) / 22.0
            h = self.hood - 2.0

            r = self.rotation * -4.0
            if 0.06 < r < 0.4:
                r = 0.4
            elif -0.06 > r > -0.4:
                r = -0.4

        else:
            v = 0
            h = 36
            r = 0
        print('v: ' + str(v * 2.55) + ' a: ' + str(h) + ' r: ' + str(r))
        return 'shooter setpoints ' + str(v * 2.55) + ' ' + str(h) + ' ' + str(r)


def main():
    rospy.init_node('communication')
    com = CommunicationHandler()
    com.communication.start()


if __name__ == '__main__':
    main()
