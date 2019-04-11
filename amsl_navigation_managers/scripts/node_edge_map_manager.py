#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import threading
import yaml
import pprint

import rospy
import tf
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

np.set_printoptions(linewidth=200)

class NodeEdgeMapManager:
    def __init__(self):
        rospy.init_node('node_edge_map_manager')

        MAP_PATH = rospy.get_param('~MAP_PATH')
        with open(MAP_PATH) as file:
            self.map_data = yaml.load(file)

        pprint.pprint(self.map_data)

        self.node_marker = MarkerArray()
        self.node_marker_pub = rospy.Publisher('/node_edge_map/viz/node', MarkerArray, queue_size=1, latch=True)
        self.edge_marker = Marker()
        self.edge_marker_pub = rospy.Publisher('/node_edge_map/viz/edge', Marker, queue_size=1, latch=True)
        self.id_marker = MarkerArray()
        self.id_marker_pub = rospy.Publisher('/node_edge_map/viz/id', MarkerArray, queue_size=1, latch=True)
        self.lock = threading.Lock()

        print '=== node edge map manager ==='

    def process(self):
        r = rospy.Rate(10)
        self.make_node_marker()
        self.make_edge_marker()
        self.make_id_marker()
        while not rospy.is_shutdown():
            self.node_marker_pub.publish(self.node_marker)
            self.edge_marker_pub.publish(self.edge_marker)
            self.id_marker_pub.publish(self.id_marker)
            r.sleep()

    def make_node_marker(self):
        self.node_marker = MarkerArray()
        time = rospy.get_rostime()
        for node in self.map_data['NODE']:
            n = Marker()
            n.ns = "node"
            n.header.frame_id = self.map_data['MAP_FRAME']
            n.header.stamp = time
            n.id = node['id']
            n.action = Marker().ADD
            n.type = Marker().CUBE
            n.lifetime = rospy.Duration()
            self.set_marker_scale(n, 0.5, 0.5, 0.5)
            self.set_marker_rgb(n, 0.0, 1.0, 0.0)
            self.set_marker_position(n, node['point']['x'], node['point']['y'], 0)
            self.node_marker.markers.append(n)

    def make_edge_marker(self):
        self.edge_marker = Marker()
        self.edge_marker.header.frame_id = self.map_data['MAP_FRAME']
        self.edge_marker.header.stamp = rospy.get_rostime()
        self.edge_marker.id = 0
        self.edge_marker.action = Marker().ADD
        self.edge_marker.type = Marker().LINE_LIST
        self.edge_marker.lifetime = rospy.Duration()
        self.edge_marker.scale.x = 0.3
        self.edge_marker.ns = "edge"
        for edge in self.map_data['EDGE']:
            p0 = Point()
            p0.x = self.map_data['NODE'][edge['node_id'][0]]['point']['x']
            p0.y = self.map_data['NODE'][edge['node_id'][0]]['point']['y']
            p1 = Point()
            p1.x = self.map_data['NODE'][edge['node_id'][1]]['point']['x']
            p1.y = self.map_data['NODE'][edge['node_id'][1]]['point']['y']
            self.edge_marker.points.append(p0)
            self.edge_marker.points.append(p1)
            color = ColorRGBA()
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
            color.a = 0.7
            self.edge_marker.colors.append(color)
            self.edge_marker.colors.append(color)

    def make_id_marker(self):
        self.id_marker = MarkerArray()
        time = rospy.get_rostime()
        for node in self.map_data['NODE']:
            m = Marker()
            m.ns = "id"
            m.header.frame_id = self.map_data['MAP_FRAME']
            m.header.stamp = time
            m.id = node['id']
            m.action = Marker().ADD
            m.type = Marker().TEXT_VIEW_FACING
            m.lifetime = rospy.Duration()
            m.scale.z = 1.0
            self.set_marker_position(m, node['point']['x'], node['point']['y'], 1)
            self.set_marker_rgb(m, 1.0, 1.0, 1.0)
            m.text = str(node['id'])
            self.id_marker.markers.append(m)

    def set_marker_scale(self, marker, x, y, z):
        marker.scale.x = x
        marker.scale.y = y
        marker.scale.z = z

    def set_marker_position(self, marker, x, y, z):
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

    def set_marker_rgb(self, marker, r, g, b, a=0.7):
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

if __name__ == '__main__':
    node_edge_map_manager = NodeEdgeMapManager()
    node_edge_map_manager.process()
