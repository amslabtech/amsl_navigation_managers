#! /usr/bin/env python
#! coding: utf-8

import sys
import os
import argparse
import yaml
import csv
import pprint

if __name__=="__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('input_path', help='input map file path (ex. /hoge/hoge_input.csv)')
    parser.add_argument('output_path', help='output map file path (ex. /hoge/hoge_output.yaml)')

    args = parser.parse_args()

    if not os.path.exists(args.input_path):
        print '\033[91m' + 'ERROR: input file does not exist'
        exit()

    input_file = os.path.splitext(args.input_path)
    if input_file[1] != '.csv':
        print '\033[91m' + 'ERROR: csv file is required as input file'

    output_file = os.path.splitext(args.output_path)
    if output_file[1] != '.yaml' and output_file[1] != '.yml':
        print '\033[91m' + 'ERROR: yaml file is required as output file'

    data = None
    with open(args.input_path, 'r') as input_csv:
        reader = csv.reader(input_csv)
        data = {'MAP_FRAME': '/map'} 
        nodes = []
        edges = []
        for row in reader:
            value =  (row[0]).split()
            if value[0] == 'VERTEX':
                node = {'id': int(value[1]),
                        'type': 'intersection',
                        'point': {'x': float(value[2]), 'y': float(value[3])},
                        'label': ''}
                nodes.append(node)
            elif value[0] == 'EDGE':
                edge = {'node_id': [int(value[1]), int(value[2])]}
                edges.append(edge)
        data.update({'NODE': nodes})
        data.update({'EDGE': edges})

    with open(args.output_path, 'w') as output_yaml: 
        yaml.dump(data, output_yaml) 
