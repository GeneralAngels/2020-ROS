#!/usr/bin/env python

import rospy
from visualization_msgs.msg import *
from FRC_utils.ros_utils import get_config, make_box_marker, make_sphere_marker

config = get_config()


def main():
    rospy.init_node('field_publisher_low_res')
    rate = rospy.Rate(30)

    inner_port = [x[1] / 100.0 for x in config['field']['inner_port'].items()]
    outer_port = [x[1] / 100.0 for x in config['field']['outer_port'].items()]
    field_area = [config['field']['width'] / 100.0, config['field']['length'] / 100.0]

    inner_port_publisher = rospy.Publisher('inner_port', Marker, queue_size=10)
    outer_port_publisher = rospy.Publisher('outer_port', Marker, queue_size=10)
    field_area_publisher = rospy.Publisher('field_area', Marker, queue_size=10)

    example_vector_from_image_processing = [2.85662, -0.03321, -0.00436, -1375.27073, -499.0836, 4498.90039]

    while not rospy.is_shutdown():
        inner_color = make_color(rospy.get_param('inner'))
        inner_port_publisher.publish(make_sphere_marker(inner_port[:3], [outer_port[3], 0.02, outer_port[3]], 'inner', inner_color))

        outer_color = make_color(rospy.get_param('outer'))
        outer_port_publisher.publish(make_sphere_marker(outer_port[:3], [outer_port[3], 0.02, outer_port[3]], 'outer', outer_color))
        field_area_publisher.publish(make_box_marker([0, 0, 0], [field_area[0], field_area[1], 0.02], 'field'))

        rate.sleep()


def make_color(param):
    if param == 'true':
        return [0, 1, 0]
    if param == 'false':
        return [1, 0, 0]
    return [0.8, 0.8, 0.8]


if __name__ == "__main__":
    main()


