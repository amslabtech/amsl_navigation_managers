#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import threading
import yaml
import pprint
import os
import time
import datetime
from stat import *

import rospy
import tf
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap

np.set_printoptions(linewidth=200)

class NodeEdgeMapManager:
    def __init__(self):
        rospy.init_node('node_edge_map_manager')

        self.MAP_PATH = rospy.get_param('~MAP_PATH')

        self.map_data = self.load_map_from_yaml()
        pprint.pprint(self.map_data)

        self.node_marker = MarkerArray()
        self.node_marker_pub = rospy.Publisher('/node_edge_map/viz/node', MarkerArray, queue_size=1, latch=True)
        self.edge_marker = Marker()
        self.edge_marker_pub = rospy.Publisher('/node_edge_map/viz/edge', Marker, queue_size=1, latch=True)
        self.id_marker = MarkerArray()
        self.id_marker_pub = rospy.Publisher('/node_edge_map/viz/id', MarkerArray, queue_size=1, latch=True)

        self.node_edge_map = NodeEdgeMap()
        self.node_edge_map_pub = rospy.Publisher('/node_edge_map', NodeEdgeMap, queue_size=1, latch=True)

        self.lock = threading.Lock()

        print '=== node edge map manager ==='

    def process(self):
        r = rospy.Rate(10)

        self.make_and_publish_map()

        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.MAP_PATH)

        while not rospy.is_shutdown():
            for file in os.listdir(dir_name):
                if 0 is file.find('.'):
                    continue
                file_timestamp = os.stat(dir_name+'/'+file)[ST_MTIME]
                if timestamp < file_timestamp:
                    timestamp = file_timestamp
                    try:
                        self.map_data = self.load_map_from_yaml()
                        self.make_and_publish_map()
                        print 'map updated!'
                    except:
                        print 'failed to update map'
            r.sleep()

    def load_map_from_yaml(self):
        with open(self.MAP_PATH) as file:
            map_data = yaml.load(file)
        return map_data

    def make_and_publish_map(self):
        self.make_node_marker()
        self.make_edge_marker()
        self.make_id_marker()
        self.make_node_edge_map()
        self.node_marker_pub.publish(self.node_marker)
        self.edge_marker_pub.publish(self.edge_marker)
        self.id_marker_pub.publish(self.id_marker)
        self.node_edge_map_pub.publish(self.node_edge_map)

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
            self.set_marker_orientation(n, 0, 0, 0)
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
            self.set_marker_orientation(m, 0, 0, 0)
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

    def set_marker_orientation(self, marker, r, p, y):
        q = tf.transformations.quaternion_from_euler(r, p, y)
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 

    def make_node_edge_map(self):
        time = rospy.get_rostime()
        self.node_edge_map.header.stamp = time
        self.node_edge_map.header.frame_id = self.map_data['MAP_FRAME'] 
        self.node_edge_map.nodes = [] 
        self.node_edge_map.edges = []
        for node in self.map_data['NODE']:
            n = Node()
            n.type = node['type']
            n.label = node['label']
            n.id = node['id']
            n.point.x = node['point']['x']
            n.point.y = node['point']['y']
            self.node_edge_map.nodes.append(n)
        for edge in self.map_data['EDGE']:
            e = Edge()
            x0 = self.map_data['NODE'][edge['node_id'][0]]['point']['x']
            y0 = self.map_data['NODE'][edge['node_id'][0]]['point']['y']
            x1 = self.map_data['NODE'][edge['node_id'][1]]['point']['x']
            y1 = self.map_data['NODE'][edge['node_id'][1]]['point']['y']
            e.length = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
            e.node0_id = edge['node_id'][0]
            e.node1_id = edge['node_id'][1]
            self.node_edge_map.edges.append(e)

if __name__ == '__main__':
    node_edge_map_manager = NodeEdgeMapManager()
    node_edge_map_manager.process()
