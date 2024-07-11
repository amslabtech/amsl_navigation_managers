#!/usr/bin/env python3

import unittest
import rostest
import rospy

import time

class TestCase(unittest.TestCase):
    def setUp(self):
        pass

    def test_mytest(self):
        self.assertTrue(True)

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_amsl_navigation_managers')
    rostest.rosrun('test_case', 'test_case', TestCase)
