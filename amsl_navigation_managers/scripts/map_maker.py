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

            utm_position = self.get_xy_from_latlng(input_data[0], input_data[1])

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

    def get_xy_from_latlng(self, lat, lng):
        '''
        references:
            https://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/algorithm/bl2xy/bl2xy.htm
            https://sw1227.hatenablog.com/entry/2018/11/30/200702#3-%E7%B5%8C%E7%B7%AF%E5%BA%A6%E3%81%8B%E3%82%89%E5%B9%B3%E9%9D%A2%E7%9B%B4%E8%A7%92%E5%BA%A7%E6%A8%99%E3%81%B8%E3%81%AE%E5%A4%89%E6%8F%9B%E5%BC%8F
            https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
            https://www.gsi.go.jp/sokuchikijun/datum-main.html#p5
        '''
        # IX from https://www.gsi.go.jp/LAW/heimencho.html
        # Noda, Chiba (36.0000, 139.8333)
        lat0 = np.deg2rad(36.)
        lng0 = np.deg2rad(139 + 50. / 60)

        lat = np.deg2rad(lat)
        lng = np.deg2rad(lng)

        # constants
        m0 = 0.9999
        a = 6378137.
        F = 298.257222101# GRS80
        #F = 298.257223563# WGS84

        n = 1. / (2 * F - 1)
        A = self.get_a(n)
        alpha = self.get_alpha(n)

        A_ = ((m0 * a) / (1. + n)) * A[0]
        S_ = ((m0 * a) / (1. + n)) * (A[0] * lat0 + np.dot(A[1:], np.sin(2 * lat0 * np.arange(1 ,6))))

        lambda_c = np.cos(lng - lng0)
        lambda_s = np.sin(lng - lng0)

        t = np.sinh(np.arctanh(np.sin(lat)) - ((2 * np.sqrt(n)) / (1 + n)) * np.arctanh(((2 * np.sqrt(n)) / (1 + n)) * np.sin(lat)))
        t_ = np.sqrt(1 + t**2)

        xi2 = np.arctan(t / lambda_c)
        eta2 = np.arctanh(lambda_s / t_)

        x = A_ * (xi2 + np.sum(np.multiply(alpha[1:], np.multiply(np.sin(2 * xi2 * np.arange(1, 6)), np.cosh(2 * eta2 * np.arange(1, 6)))))) - S_
        y = A_ * (eta2 + np.sum(np.multiply(alpha[1:], np.multiply(np.cos(2 * xi2 * np.arange(1, 6)), np.sinh(2 * eta2 * np.arange(1, 6))))))

        return np.array([float(x), float(y)])

    def get_latlng_from_xy(self, x, y):
        # IX from https://www.gsi.go.jp/LAW/heimencho.html
        # Noda, Chiba (36.0000, 139.8333)
        lat0 = np.deg2rad(36.)
        lng0 = np.deg2rad(139 + 50. / 60)
        # constants
        m0 = 0.9999
        a = 6378137.
        F = 298.257222101# GRS80
        #F = 298.257223563# WGS84

        n = 1. / (2 * F - 1)
        A = self.get_a(n)
        alpha = self.get_alpha(n)
        beta = self.get_beta(n)
        delta = self.get_delta(n)

        A_ = ((m0 * a) / (1. + n)) * A[0]
        S_ = ((m0 * a) / (1. + n)) * (A[0] * lat0 + np.dot(A[1:], np.sin(2 * lat0 * np.arange(1 ,6))))

        xi = (x + S_) / A_
        eta = y / A_
        xi_ = xi - np.sum(np.multiply(beta[1:], np.multiply(np.sin(2 * np.arange(1, 6) * xi), np.cosh(2 * np.arange(1, 6) * eta))))
        eta_ = eta - np.sum(np.multiply(beta[1:], np.multiply(np.cos(2 * np.arange(1, 6) * xi), np.sinh(2 * np.arange(1, 6) * eta))))
        chi = np.arcsin(np.sin(xi_) / np.cosh(eta_))

        lat = chi + np.sum(np.multiply(delta, np.sin(2 * chi * np.arange(1, 7))))
        lng = lng0 + np.arctan(np.sinh(eta_) / np.cos(xi_))
        return lat, lng

    def get_a(self, n):
        A0 = 1 + n**2 / 4. + n**4 / 64.
        A1 = - 3. * (n - n**3 / 8. - n**5 / 64.) / 2.
        A2 = 15. * (n**2 - n**4 / 4.) / 16.
        A3 = -35. * (n**3 - 5. * n**5 / 16.) / 48.
        A4 = 315. * n**4 / 512.
        A5 = -693 * n**5 / 1280.
        return np.array([A0, A1, A2, A3, A4, A5])

    def get_alpha(self, n):
        alpha0 = np.nan
        alpha1 = n / 2. - 2. * n**2 / 3 + 5. * n**3 / 16 + 41. * n**4 / 180 - 127. * n**5 / 288
        alpha2 = 13. * n**2 / 48. - 3. * n**3 / 5 + 557. * n**4 / 1440 + 281. * n**5 / 630
        alpha3 = 61. * n**3 / 240 - 103. * n**4 / 140 + 15061. * n**5 / 26880
        alpha4 = 49561. * n**4 / 161280 - 179. * n**5 / 168
        alpha5 = 34729. * n**5 / 80640
        return np.array([alpha0, alpha1, alpha2, alpha3, alpha4, alpha5])

    def get_beta(self, n):
        beta0 = np.nan
        beta1 = n / 2. - 2. * n**2 / 3 + 37. * n**3 / 96 - n**4 / 360. - 81. * n**5 / 512
        beta2 = n**2 / 48. + n**3 / 15. - 437. * n**4 / 1440 + 46. * n**5 / 105
        beta3 = 17. * n**3 / 480 - 37. * n**4 / 840 - 209 * n**5 / 4480
        beta4 = 4397. * n**4 / 161280 - 11. * n**5 / 504
        beta5 = 4583. * n**5 / 161280
        return np.array([beta0, beta1, beta2, beta3, beta4, beta5])

    def get_delta(self, n):
        delta0 = 2 * n - 2. * n**2 / 3 + 116. * n**4 / 45 + 26. * n**5 / 45 - 2854. * n**6 / 675
        delta1 = 7. * n**2 / 3 - 8. * n**3 / 5 - 227. * n**4 / 45 + 2704. * n**5 / 315 + 2323. * n**6 / 945
        delta2 = 56. * n**3 / 15 - 136. * n**4 / 35 - 1262. * n**5 / 105 + 73814. * n**6 / 2835
        delta3 = 4279. * n**4 / 630 - 332. * n**5 / 35 - 399572. * n**6 / 14175
        delta4 = 4174. * n**5 / 315 - 144838. * n**6 / 6237
        delta5 = 601676. * n**6 / 22275
        return np.array([delta0, delta1, delta2, delta3, delta4, delta5])

if __name__ == '__main__':
    map_maker = MapMaker()
    map_maker.process()
    # x, y = map_maker.get_xy_from_latlng(36.103774791666666, 140.08785504166664)
    # x, y = map_maker.get_xy_from_latlng(36.0, 139 + 50/60.)
    # x, y = map_maker.get_xy_from_latlng(36.082882, 140.077230)
    # print x, y
    # lat, lng = map_maker.get_latlng_from_xy(x, y)
    # print np.rad2deg(lat), np.rad2deg(lng)
