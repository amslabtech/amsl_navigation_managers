#!/usr/bin/env python
#! coding:utf-8

from __future__ import print_function

import numpy as np
import yaml
import os
import time
import datetime
from pprint import pprint
import argparse
import csv

class MapMaker:
    def __init__(self, args):
        print('=== map_maker ===')
        self.origin_utm = np.zeros(2)

        self.map_data = dict()
        self.nodes = list()

        self.utm = UTMUtils()

        if args.INPUT_LATLNG_PATH is not None:
            self.make_map_from_latlng(args.INPUT_LATLNG_PATH, args.OUTPUT_MAP_PATH)

        if args.INPUT_MAP_PATH is not None:
            print("load map from ", self.INPUT_MAP_PATH)
            self.map_data = self.load_map_from_yaml(self.INPUT_MAP_PATH)

            self.nodes = self.map_data['NODE']
            self.origin_utm = np.array([self.map_data['ORIGIN_UTM']['x'], self.map_data['ORIGIN_UTM']['y']])
            print(self.origin_utm)
            #pprint(self.nodes)

    def make_map_from_latlng(self, latlng_path, map_path):
        print('--- make map from latlng ---')
        self.map_data['MAP_FRAME'] = 'map'
        data = list()
        with open(latlng_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append(row)
        data = np.array(data).astype(np.float)
        print('latlng')
        print(data)

        utm_positions = list()
        for latlng in data:
            x, y = self.utm.get_xy_from_latlng(latlng[0], latlng[1])
            utm_positions.append([x, y])

        utm_positions = np.array(utm_positions)
        print('utm')
        print(utm_positions)
        origin = utm_positions[0].copy().tolist()
        self.map_data['ORIGIN_UTM'] = {'x' : origin[0], 'y' : origin[1]}
        for utm_position in utm_positions:
            utm_position -= origin
        print('map coordinate')
        print(utm_positions)
        utm_positions = utm_positions.tolist()
        nodes = list()
        for position in utm_positions:
            node = {'id': len(nodes),
                    'type': 'intersection',
                    'point': {'x': position[0], 'y': position[1]},
                    'label': ''}
            nodes.append(node)
        self.map_data['NODES'] = nodes
        self.map_data['EDGES'] = [{'node_id': [0, 0]}]
        print('map')
        pprint(self.map_data)
        self.save_map_to_yaml(self.map_data, map_path)

    def load_map_from_yaml(self, path):
        with open(path, 'r') as f:
            map_data = yaml.load(f)
        return map_data

    def save_map_to_yaml(self, map_data, path):
        with open(path, 'w') as f:
            f.write(yaml.dump(map_data))
        print('\033[32msuccessfully saved!\033[0m')

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
    parser = argparse.ArgumentParser(description='Edge Node Map Maker')
    parser.add_argument('--INPUT_MAP_PATH', type=str, help='map path to load (yaml)', default=None)
    parser.add_argument('--OUTPUT_MAP_PATH', type=str, help='map path to save (yaml)', default='test.yaml')
    parser.add_argument('--INPUT_LATLNG_PATH', type=str, help='input data path (csv)', default=None)
    args = parser.parse_args()

    map_maker = MapMaker(args)
