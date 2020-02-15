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
from math import *
import time


class Client(Thread):
    def __init__(self, cmd, config, callback=None, log=True):
        super(Client, self).__init__()
        self.log = log
        self.callback = callback
        self.cmd = cmd
        self.config = config
        self.last_receive = None
        self.rate = int(config['networking']['rate'])
        self.ip = str(config['networking']['RIO_IP'])
        self.port = int(config['networking']['RIO_PORT'])
        self.buffer_size = int(config['networking']['BUFFER_SIZE'])
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.is_running = False
        self.start()

    def run(self):
        self.is_running = True
        if self.log:
            print('trying to connect to ' + str(self.ip) + ' on port ' + str(self.port) + '...')

        self.socket.connect((self.ip, self.port))

        if self.log:
            print('connected.')

        try:
            while self.is_running:
                self.send_cmd()
                self.receive()
                time.sleep(1.0 / self.rate)
        except:
            self.stop()
            self.start()

        finally:
            self.stop()

    def send_cmd(self):
        # if self.log:
        #     print('sending: ' + str(self.cmd))
        self.socket.send(str(self.cmd) + '\n')

    def receive(self):
        if self.callback is not None:
            buff = ''
            while '\n' not in buff:
                buff += self.socket.recv(self.buffer_size)
            buff = buff.split('\n')[0]
            self.callback(buff)
            if self.log:
                print('received: ' + str(buff))
            self.last_receive = buff

    def stop(self):
        self.is_running = False


class Communication(Thread):
    commands = {'odom': 'odometry json',
                'robot': 'robot json',
                'turret': 'shooter turret '
                }

    def __init__(self, config):
        super(Communication, self).__init__()

        # odometry - from RIO
        # self.odom_publisher = rospy.Publisher('/robot_odometry', Odometry, queue_size=100)
        # self.robot_arrow_publisher = rospy.Publisher('/robot_location', Marker, queue_size=10)
        # self.odom_client = Client(cmd=self.commands['odom'], config=config, callback=self.handle_odom, log=True)

        # shooting velocity and alpha - from ROS

        # Turret - rotation:
        self.turret_client = Client(cmd='shooter turret false 0',
                                    config=config, log=True)

        rospy.Subscriber('/target/rotation', Float32, callback=self.handle_rotation)

    def handle_odom(self, data):
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
        self.odom_publisher.publish(odom)

    def handle_rotation(self, data):
        cmd = 'shooter turret ' + str(rospy.get_param('target_in_frame')) + ' ' + str(data.data)
        # print(cmd)
        self.turret_client.cmd = cmd


def main():
    rospy.init_node('communication')
    config = get_config()
    com = Communication(config)


if __name__ == '__main__':
    main()
