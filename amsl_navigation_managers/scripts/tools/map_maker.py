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
import copy
from pprint import pprint

import rospy
import tf
from std_msgs.msg import ColorRGBA, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap

class MapMaker:
    def __init__(self):
        rospy.init_node('map_maker')

        if rospy.has_param('~OUTPUT_MAP_PATH'):
            self.OUTPUT_MAP_PATH = rospy.get_param('~OUTPUT_MAP_PATH')
        if rospy.has_param('~INPUT_MAP_PATH'):
            self.INPUT_MAP_PATH = rospy.get_param('~INPUT_MAP_PATH')
        else:
            self.INPUT_MAP_PATH = None

        self.lock = threading.Lock()

        self.origin_utm = np.empty(2)

        self.map_data = None
        self.nodes = []

        self.utm = UTMUtils()

        if self.INPUT_MAP_PATH is not None:
            print "load map from ", self.INPUT_MAP_PATH
            self.map_data = self.load_map_from_yaml(self.INPUT_MAP_PATH)

            self.nodes = self.map_data['NODE']
            self.origin_utm = np.array([self.nodes[0]['point']['x'], self.nodes[0]['point']['y']])
            ###### from xy to utm #####
            print self.origin_utm
            #pprint(self.nodes)
        print '=== map_maker ==='

    def process(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            input_data = raw_input("input latitude and longitude like XXX.XXXXXX, YYY.YYYYYY\n(type 'save' or 'exit' to finish editting map)\n:")
            if input_data == 'save':
                pass
            elif input_data == 'exit':
                exit(0)
            input_data = input_data.split(",")

            if len(input_data) < 2:
                print '\033[91m' + 'ERROR: both latitude and longitude are required\033[00m'
                continue
            elif len(input_data) > 2:
                print '\033[91m' + 'ERROR: only two values (latitude and longitude) are required\033[00m'
                continue

            try:
                input_data = [float(data) for data in input_data]
            except:
                print '\033[91m' + 'ERROR: numbers are required\033[00m'
                continue

            utm_position = self.utm.get_xy_from_latlng(input_data[0], input_data[1])

            if len(self.nodes) is 0:
                self.origin_utm = utm_position
            print "origin(utm):", self.origin_utm
            print "utm positin:", utm_position

            position = utm_position -  self.origin_utm
            node = {'id': len(self.nodes),
                    'type': 'intersection',
                    'point': {'x': position[0], 'y': position[1]},
                    'label': ''}
            # print node
            self.nodes.append(node)
            print self.nodes

            print ""

            r.sleep()

    def load_map_from_yaml(self, path):
        with open(path) as file:
            map_data = yaml.load(file)
        return map_data

class UTMUtils:
    def __init__(self):
        pass

    def get_xy_from_latlng(self, lat, lng):
        '''
        references:
            https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
        '''

        # zone 54
        lambda0 = np.deg2rad((138 + 144) * 0.5)

        E0 = 500# [km]
        N0 = 0# [km]

        phi_ = np.deg2rad(lat)
        lambda_ = np.deg2rad(lng)

        # constants
        k0 = 0.9996
        a = 6378137. * 1e-3
        F = 1.0 / 298.257222101# GRS80
        #F = 1.0 / 298.257223563# WGS84

        n = F / (2 - F)

        A = self.get_A(a, n)

        alpha = self.get_alpha(n)

        t = np.sinh(np.arctanh(np.sin(phi_)) - 2. * np.sqrt(n) / (1 + n) * np.arctanh(2. * np.sqrt(n) / (1 + n) * np.sin(phi_)))

        xi_ = np.arctan(t / np.cos(lambda_ - lambda0))

        eta_ = np.arctanh(np.sin(lambda_ - lambda0) / np.sqrt(1 + t**2))

        j = np.arange(1, 4)

        E = E0 + k0 * A * (eta_ + np.sum(alpha[1:] * np.cos(2 * j * xi_) * np.sinh(2 * j * eta_)))

        N = N0 + k0 * A * (xi_ + np.sum(alpha[1:] * np.sin(2 * j * xi_) * np.cosh(2 * j * eta_)))

        return np.array([float(E), float(N)]) * 1e3# [m]

    def get_latlng_from_xy(self, x, y):
        '''
        references:
            https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
        '''

        # zone 54
        zone = 54
        lambda0 = zone * 6 - 183# [deg]

        E0 = 500# [km]
        N0 = 0# [km]

        # northern hemisphere
        hemi = 1

        E0 = 500# [km]
        N0 = 0# [km]

        # unit of x and y are [m]
        E = x * 1e-3
        N = y * 1e-3

        # constants
        k0 = 0.9996
        a = 6378137. * 1e-3
        F = 1.0 / 298.257222101# GRS80
        #F = 1.0 / 298.257223563# WGS84

        n = F / (2 - F)

        A = self.get_A(a, n)

        alpha = self.get_alpha(n)

        xi = (N - N0) / (k0 * A)
        eta = (E - E0) / (k0 * A)

        j = np.arange(1, 4)

        beta = self.get_beta(n)
        delta = self.get_delta(n)

        xi_ = xi - np.sum(beta[1:] * np.sin(2 * j * xi) * np.cosh(2 * j * eta))
        eta_ = eta - np.sum(beta[1:] * np.cos(2 * j * xi) * np.sinh(2 * j * eta))

        chi = np.arcsin(np.sin(xi_) / np.cosh(eta_))

        phi_ = chi + np.sum(delta[1:] * np.sin(2 * j * chi))

        lambda_ = np.deg2rad(lambda0) + np.arctan(np.sinh(eta_) / np.cos(xi_))

        return phi_, lambda_# [rad]

    def get_A(self, a, n):
        A = a / (1 + n) * (1 + n**2 / 4. + n**4 / 64.)
        return A

    def get_alpha(self, n):
        alpha = np.array([0,
                          1. / 2. * n - 2. / 3. * n**2 + 5. / 16. * n**3,
                          13. / 48. * n**2 - 3. / 5. * n**3,
                          61. / 240. * n**3,
                         ])
        return alpha

    def get_beta(self, n):
        beta = np.array([0,
                         1. / 2. * n - 2. / 3. * n**2 + 37. / 96. * n**3,
                         1. / 48. * n**2 + 1. / 15. * n**3,
                         17. / 480. * n**3,
                        ])
        return beta

    def get_delta(self, n):
        delta = np.array([0,
                          2. * n - 2. / 3. * n**2 - 2. * n**3,
                          7. / 3. * n**2 - 8. / 5. * n**3,
                          56. / 15. * n**3,
                         ])
        return delta

if __name__ == '__main__':
    map_maker = MapMaker()
    map_maker.process()
    # x, y = map_maker.get_xy_from_latlng(36.0, 139 + 50/60.)
    # print x, y
    # lat, lng = map_maker.get_latlng_from_xy(x, y)
    # print np.rad2deg(lat), np.rad2deg(lng)
