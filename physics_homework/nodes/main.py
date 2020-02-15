#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
import tf
from math import *
import time
import numpy as np



class Vect:
    def __init__(self, x, y, z, ax, ay, az, potential_value=0, is_charge=False):
        self.is_charge = is_charge
        self.potential_value = potential_value
        self.point = Point(float(x), float(y), float(z))
        self.quaternion = Quaternion(float(ax), float(ay) ,float(az), 1.0)

    def get_marker(self, id):
        pose = Pose(self.point, self.quaternion)
        marker = Marker()
        marker.action = Marker.ADD
        marker.scale = Vector3(0.3, 0.05, 0.05)
        marker.id = id
        r, g, b = 220.0, 95.0 ,205.0
        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0
        marker.color.a = 1.0 - float(self.potential_value)/float(9)
        marker.type = Marker.ARROW
        if self.is_charge:
            marker.scale = Vector3(0.3, 0.3, 0.3)
            marker.type = Marker.SPHERE
            marker.id = id + 10000000
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        marker.ns = 'basic_shapes'
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/map'
        marker.pose = pose
        marker.lifetime = rospy.Duration()


        return marker


class ElectricField:
    vectors = []
    charges = []

    def __init__(self, size):
        self.x, self.y, self.z = size
        for x in range(self.x):
            arr_x = []
            for y in range(self.y):
                arr_y = []
                for z in range(self.z):
                    arr_y.append(None)
                arr_x.append(arr_y)
            self.vectors.append(arr_x)
        self.update_vectors()

    def update_vectors(self):
        for x1 in range(self.x):
            for y1 in range(self.y):
                for z1 in range(self.z):
                    vectors = []
                    for c in self.charges:
                        size = (((c.point.x - x1)**2) + ((c.point.y - y1)**2) + ((c.point.z - z1)**2))**0.5

                        vectors.append(size)
                    size = np.sum(vectors, axis=0)
                    self.vectors[x1][y1][z1] = Vect(x1, y1, z1, 0, 0 ,0, size)

    def add_charge(self, charge):
        self.charges.append(charge)

    def to_marker_array(self):
        markers = []
        for x in range(self.x):
            for y in range(self.y):
                for z in range(self.z):
                    markers.append(self.vectors[x][y][z].get_marker((x*(self.x**2) + y*self.y + z)))
        return markers

    def get_charges_as_poses(self):
        markers = []
        for e, c in enumerate(self.charges):
            markers.append(c.get_marker(e))
        return markers





def main():
    rospy.init_node('electric_field_simulator')
    rate = rospy.Rate(20)

    vectors_publisher = rospy.Publisher('/field_vectors', MarkerArray, queue_size=1)
    charges_publisher = rospy.Publisher('/charges', MarkerArray, queue_size=1)\

    ef = ElectricField((20, 20, 20))

    ef.add_charge(Vect(0, 0, 0, 0, 0, 0, is_charge=True))
    # ef.add_charge(Vect(19, 19, 19, 0, 0, 0, is_charge=True))

    while not rospy.is_shutdown():
        ef.update_vectors()

        vectors = MarkerArray()
        vectors.markers =  ef.to_marker_array()
        vectors_publisher.publish(vectors)

        charges = MarkerArray()
        charges.markers = ef.get_charges_as_poses()
        charges_publisher.publish(charges)

        rate.sleep()

if __name__ == '__main__':
    main()
