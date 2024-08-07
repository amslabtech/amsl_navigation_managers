#!/usr/bin/env python3

import copy
import datetime
import math
import os
import pprint
import threading
import time
from stat import *

import numpy as np
import rospy
import tf
import yaml
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Edge, Node, NodeEdgeMap
from amsl_navigation_msgs.srv import UpdateCheckpoint, UpdateCheckpointResponse


class CheckpointManager:
    def __init__(self):
        rospy.init_node("checkpoint_manager")

        if rospy.has_param("~CHECKPOINT_PATH"):
            self.CHECKPOINT_PATH = rospy.get_param("~CHECKPOINT_PATH")

        self.HZ = 10
        if rospy.has_param("~HZ"):
            self.HZ = rospy.get_param("~HZ")

        self.cp_data = self.load_cp_from_yaml()
        pprint.pprint(self.cp_data)

        self.cp_passed = []

        self.cp_marker = MarkerArray()
        self.cp_marker_pub = rospy.Publisher(
            "/node_edge_map/viz/node/checkpoint",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.checkpoint_list = Int32MultiArray()
        self.checkpoint_list_pub = rospy.Publisher(
            "/node_edge_map/checkpoint",
            Int32MultiArray,
            queue_size=1,
            latch=True,
        )

        self.node_markers = MarkerArray()
        self.node_sub = rospy.Subscriber(
            "/node_edge_map/viz/node", MarkerArray, self.node_callback
        )

        self.current_edge = Edge()
        self.edge_sub = rospy.Subscriber(
            "/estimated_pose/edge", Edge, self.edge_callback
        )

        self.update_checkpoint_server = rospy.Service(
            "/node_edge_map/update_checkpoint",
            UpdateCheckpoint,
            self.update_checkpoint_handler,
        )

        self.lock = threading.Lock()

        print("=== checkpoint manager ===")

    def process(self):
        r = rospy.Rate(self.HZ)

        self.make_and_publish_checkpoint()

        timestamp = time.mktime(datetime.datetime.now().utctimetuple())
        dir_name = os.path.dirname(self.CHECKPOINT_PATH)

        while not rospy.is_shutdown():
            for file in os.listdir(dir_name):
                if file.find(".") == 0:
                    continue
                file_timestamp = os.stat(dir_name + "/" + file)[ST_MTIME]
                if timestamp < file_timestamp:
                    timestamp = file_timestamp
                    try:
                        self.cp_data = self.load_cp_from_yaml()
                        print("checkpoint updated!")
                    except:
                        print("failed to update checkpoint")
            self.make_and_publish_checkpoint()
            r.sleep()

    def load_cp_from_yaml(self):
        with open(self.CHECKPOINT_PATH) as file:
            cp_data = yaml.safe_load(file)
        return cp_data

    def make_and_publish_checkpoint(self):
        self.make_checkpoint()
        self.update_node_color()
        self.checkpoint_list_pub.publish(self.checkpoint_list)
        self.cp_marker_pub.publish(self.cp_marker)

    def make_checkpoint(self):
        self.checkpoint_list = Int32MultiArray()
        for cp in self.cp_data["checkpoints"]:
            self.checkpoint_list.data.append(cp)

    def node_callback(self, node):
        with self.lock:
            self.node_markers = node

    def edge_callback(self, edge):
        with self.lock:
            self.current_edge = edge
            if self.cp_data["checkpoints"][0] == self.current_edge.node0_id:
                self.cp_passed.append(self.cp_data["checkpoints"][0])
                del self.cp_data["checkpoints"][0]

    def update_node_color(self):
        self.cp_marker = MarkerArray()
        markers = copy.deepcopy(self.node_markers.markers)
        for node in markers:
            appended = False
            for cp_id in self.checkpoint_list.data:
                if cp_id == node.id:
                    self.set_marker_rgb(node, 1.0, 0.0, 0.0)
                    self.cp_marker.markers.append(node)
                    appended = True
                    break
            if not appended:
                self.cp_marker.markers.append(node)

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

    def update_checkpoint_handler(self, request):
        if request.operation == request.ADD:
            try:
                flag = False
                for node in self.node_markers.markers:
                    if node.id == request.id:
                        flag = True
                if flag:
                    self.cp_data["checkpoints"].insert(0, request.id)
                else:
                    raise Exception
                self.make_and_publish_checkpoint()
                return UpdateCheckpointResponse(True)
            except Exception as e:
                print(e)
                print("failed to update checkpoint")
                return UpdateCheckpointResponse(False)
        elif request.operation == request.DELETE:
            try:
                self.cp_data["checkpoints"].remove(request.id)
                self.make_and_publish_checkpoint()
                return UpdateCheckpointResponse(True)
            except Exception as e:
                print(e)
                print("failed to update checkpoint")
                return UpdateCheckpointResponse(False)


if __name__ == "__main__":
    checkpoint_manager = CheckpointManager()
    checkpoint_manager.process()
