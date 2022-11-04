#!/usr/bin/env python3
#! coding:utf-8

#注：rwrc21_task_managerとは仕様が全く違います
import yaml

import rospy
from std_msgs.msg import Bool, Int32

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')
        print('=== task manager ===')

        self.TASK_LIST_PATH = rospy.get_param('~TASK_LIST_PATH')

        self.current_checkpoint_id_sub = rospy.Subscriber('/current_checkpoint', Int32, self.checkpoint_id_callback)
        self.is_detected_sub = rospy.Subscriber('/hoge', Bool, self.detected_line_callback)

        self.detect_line_flag_pub = rospy.Publisher('/detect_line', Bool, queue_size=1)
        self.task_stop_flag_pub = rospy.Publisher('/task/stop' , Bool, queue_size=1)

        self.get_task = False
        self.last_checkpoint_id = -1
        self.current_checkpoint_id= -1
        self.detect_line_flag = False
        self.task_data = self.load_task_from_yaml()

        self.last_line_flag = False

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # print(self.get_task)
            task_type = self.search_task_from_node_id(self.last_checkpoint_id, self.current_checkpoint_id)
            # print('node0_id =', self.last_checkpoint_id, 'node1_id = ', self.current_checkpoint_id)
            # print('task_type = %s' % task_type)
            if task_type == 'detect_line':
                self.detect_line_flag = True
            else:
                self.detect_line_flag = False
            self.detect_line_flag_pub.publish(self.detect_line_flag)
            r.sleep()

    def load_task_from_yaml(self):
        with open(self.TASK_LIST_PATH) as file:
            task_data = yaml.safe_load(file)
            self.get_task = True
            # print('get task')
        return task_data

    def checkpoint_id_callback(self, checkpoint_id):
        if self.current_checkpoint_id != int(checkpoint_id.data):
            self.last_checkpoint_id = self.current_checkpoint_id
            self.current_checkpoint_id = int(checkpoint_id.data)

    def detected_line_callback(self, flag):
        # print('current flag = ', flag.data, 'last flag = ', self.last_line_flag)
        if self.last_line_flag == True and flag.data == False:
            self.task_stop_flag_pub.publish(False)
        if self.last_line_flag == False and flag.data == True:
            self.task_stop_flag_pub.publish(True)
        self.last_line_flag = flag.data

    def search_task_from_node_id(self, node0_id, node1_id):
        if self.get_task == True:
            # print('get_task is true')
            for count, task in enumerate(self.task_data['task']):
                # print(task)
                if (task['edge']['node0_id'] == node0_id) and (task['edge']['node1_id'] == node1_id):
                    return task['task_type']
            else:
                return ''
        else:
            return ''

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.process()
