#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import threading

import rospy
import tf
from nav_msgs.msg import Odometry
import message_filters
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from landmark_slam_msgs.msg import Landmark, LandmarkArray

np.set_printoptions(linewidth=200)

CHI_2 = 9.21934 # X^2, 99%

class ObjectDetectionBasedLandmarkSLAM:
    def __init__(self):
        self.Q = np.diag([0.5, 0.5, np.radians(10.0)])**2
        self.R = np.diag([0.3, 0.3])**2 # input noise
        self.LIMIT_MAHALANOBIS_DISTANCE = 5.99196 # 2DoF, 5%
        self.ROBOT_STATE_SIZE = 3 # x, y, yaw
        self.LANDMARK_STATE_SIZE = 2 # x, y

        rospy.init_node('object_detection_based_landmark_slam')

        self.estimated_pose_pub = rospy.Publisher('/estimated_pose', Odometry, queue_size=1)
        self.marker_pub = rospy.Publisher('/error_ellipse', MarkerArray, queue_size=1)

        self._odom_sub = rospy.Subscriber('/odom/sim/noise', Odometry, self.odom_callback)
        self._lm_sub = rospy.Subscriber('/observation/sim', LandmarkArray, self.landmark_callback)

        self.last_time = rospy.get_time()
        self.current_pose = None
        self.last_pose = None
        self.first_flag = True
        self.x_est = np.zeros((self.ROBOT_STATE_SIZE, 1))
        self.p_est = np.eye(self.ROBOT_STATE_SIZE)
        self.estimated_pose = Odometry()
        self.error_ellipse = MarkerArray()

        self.lock = threading.Lock()

        print 'object detection based landmark SLAM'

        self.send_transform(self.x_est)

    def move(self, x, u, dt):
        F = np.array([[1.0, 0, 0],
                      [0, 1.0, 0],
                      [0, 0, 1.0]])

        B = np.array([[dt * math.cos(x[2, 0]), 0],
                      [dt * math.sin(x[2, 0]), 0],
                      [0.0, dt]])

        x = np.dot(F, x) + np.dot(B, u)
        return x

    def predict(self, x_est, p_est, u, dt):
        print "=== predict ==="
        print x_est[0:self.ROBOT_STATE_SIZE]
        print p_est[0:self.ROBOT_STATE_SIZE, 0:self.ROBOT_STATE_SIZE]
        S_ = self.ROBOT_STATE_SIZE
        x_est[0:S_] = self.move(x_est[0:S_], u, dt)
        jf = self.get_jacobian_f(x_est[0:S_], u, dt)
        p_est[0:S_, 0:S_] = np.dot(np.dot(jf, p_est[0:S_, 0:S_]), jf.T) + self.Q
        x_est[2] = self.pi_2_pi(x_est[2])
        print x_est[0:self.ROBOT_STATE_SIZE]
        print p_est[0:self.ROBOT_STATE_SIZE, 0:self.ROBOT_STATE_SIZE]
        return x_est, p_est

    def update(self, x_est, p_est, z):
        print "=== update ==="
        print(str(len(z)) + " landmarks detected")
        for iz in range(len(z)):  # for each observation
            minid = self.get_correspond_landmark_index(x_est, p_est, z[iz, 0:2])

            nLM = self.calculate_landmark_num(x_est)
            if minid == nLM:
                print("New LM")
                # Extend state and covariance matrix
                xAug = np.vstack((x_est, self.calculate_landmark_position(x_est, z[iz, :])))
                PAug = np.vstack((np.hstack((p_est, np.zeros((len(x_est), self.LANDMARK_STATE_SIZE)))),
                                  np.hstack((np.zeros((self.LANDMARK_STATE_SIZE, len(x_est))), np.eye(self.LANDMARK_STATE_SIZE)))))
                x_est = xAug
                p_est = PAug
            lm = self.get_estimated_landmark_position(x_est, minid)
            e, S, H = self.calculate_innovation(lm, x_est, p_est, z[iz, 0:2], minid)

            K = np.dot(np.dot(p_est, H.T), np.linalg.inv(S))
            x_est = x_est + np.dot(K, e)
            p_est = np.dot((np.eye(len(x_est)) - np.dot(K, H)), p_est)

        x_est[2] = self.pi_2_pi(x_est[2])
        print x_est[0:self.ROBOT_STATE_SIZE]
        print p_est[0:self.ROBOT_STATE_SIZE, 0:self.ROBOT_STATE_SIZE]
        return x_est, p_est

    def get_jacobian_f(self, x, u, dt):
        Fx = np.hstack((np.eye(self.ROBOT_STATE_SIZE), np.zeros(
            (self.ROBOT_STATE_SIZE, self.LANDMARK_STATE_SIZE * self.calculate_landmark_num(x)))))

        jF = np.array([[1.0, 0.0, -dt * u[0] * math.sin(x[2, 0])],
                       [0.0, 1.0, dt * u[0] * math.cos(x[2, 0])],
                       [0.0, 0.0, 1.0]])

        return jF

    def get_jacobian_h(self, delta, i, n):
        d = math.sqrt(delta[0, 0]**2 + delta[1, 0]**2)
        H = np.array([[-d * delta[0, 0], - d * delta[1, 0], 0, d * delta[0, 0], d * delta[1, 0]],
                      [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

        H = H / d**2
        H1 = np.hstack((np.eye(3), np.zeros((3, 2 * n))))
        H2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * n - 2 * i))))

        _H = np.vstack((H1, H2))

        H = np.dot(H, _H)

        return H

    def calculate_landmark_num(self, x):
        n = int((len(x) - self.ROBOT_STATE_SIZE) / self.LANDMARK_STATE_SIZE)
        return n

    def get_estimated_landmark_position(self, x, index):
        lm = x[self.ROBOT_STATE_SIZE + self.LANDMARK_STATE_SIZE * index: self.ROBOT_STATE_SIZE + self.LANDMARK_STATE_SIZE * (index + 1), :]
        return lm

    def calculate_landmark_position(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

        return zp

    def get_correspond_landmark_index(self, x_est, p_est, zi):
        nLM = self.calculate_landmark_num(x_est)
        mahalanobis_distance = []

        for i in range(nLM):
            lm = self.get_estimated_landmark_position(x_est, i)
            e, S, H = self.calculate_innovation(lm, x_est, p_est, zi, i)
            mahalanobis_distance.append(np.dot(np.dot(e.T, np.linalg.inv(S)), e))

        mahalanobis_distance.append(self.LIMIT_MAHALANOBIS_DISTANCE)  # new landmark
        minid = mahalanobis_distance.index(min(mahalanobis_distance))
        return minid

    def calculate_innovation(self, lm, x_est, p_est, z, index):
        # dx, dy
        delta = lm - x_est[0:2]
        # distance^2
        d2 = np.dot(delta.T, delta)[0, 0]
        # angle in robot frame
        zangle = math.atan2(delta[1, 0], delta[0, 0]) - x_est[2, 0]
        # polar coordinates
        zp = np.array([[math.sqrt(d2), self.pi_2_pi(zangle)]])
        # error in polar coordinates (estimated pose - observed pose)
        e = (z - zp).T
        e[1] = self.pi_2_pi(e[1])
        H = self.get_jacobian_h(delta, index + 1, self.calculate_landmark_num(x_est))
        S = np.dot(np.dot(H, p_est), H.T) + self.R
        return e, S, H

    def calculate_error_ellipse(self, P):
        _lambda, _v = np.linalg.eig(P)
        max_index = np.argmax(_lambda)
        min_index = np.argmin(_lambda)
        a = math.sqrt(CHI_2 * _lambda[max_index])
        b = math.sqrt(CHI_2 * _lambda[min_index])
        ellipse_angle = math.atan2(_v[max_index, 1], _v[max_index, 0])
        return a, b, ellipse_angle

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_observation_from_landmark_msg(self, landmark):
        z = []
        for lm in landmark.landmarks:
            _z = [math.sqrt(lm.pose.pose.position.x * lm.pose.pose.position.x + lm.pose.pose.position.y * lm.pose.pose.position.y), math.atan2(lm.pose.pose.position.y, lm.pose.pose.position.x)]
            z.append(_z)
        z = np.array(z)
        return z

    def send_transform(self, x):
        br = tf.TransformBroadcaster()
        br.sendTransform((x[0], x[1], 0),
                         self.get_quaternion_from_yaw(x[2, 0]),
                         rospy.Time.now(),
                         "base_link",
                         "world"
                         )

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_transform(self.x_est)
            r.sleep()

    def get_quaternion_from_yaw(self, yaw):
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        return q

    def publish_estimated_pose(self, x, p):
        self.estimated_pose.header.frame_id = "world"
        self.estimated_pose.header.stamp = rospy.get_rostime()
        self.estimated_pose.child_frame_id = "base_link"
        self.estimated_pose.pose.pose.position.x = x[0]
        self.estimated_pose.pose.pose.position.y = x[1]
        q = self.get_quaternion_from_yaw(x[2, 0])
        self.estimated_pose.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        '''
        self.estimated_pose.pose.covariance[0] = p[0, 0]
        self.estimated_pose.pose.covariance[1] = p[0, 1]
        self.estimated_pose.pose.covariance[5] = p[0, 2]
        self.estimated_pose.pose.covariance[6] = p[1, 0]
        self.estimated_pose.pose.covariance[7] = p[1, 1]
        self.estimated_pose.pose.covariance[11] = p[1, 2]
        self.estimated_pose.pose.covariance[30] = p[2, 0]
        self.estimated_pose.pose.covariance[31] = p[2, 1]
        self.estimated_pose.pose.covariance[35] = p[2, 2]
        '''
        self.estimated_pose_pub.publish(self.estimated_pose);

    def publish_error_ellipse_markers(self, x, p):
        self.error_ellipse = MarkerArray()
        m = self.get_error_ellipse_marker(x[0:self.ROBOT_STATE_SIZE-1], p[0:self.ROBOT_STATE_SIZE-1, 0:self.ROBOT_STATE_SIZE-1], 0)

        self.error_ellipse.markers.append(m)

        n = self.calculate_landmark_num(x)
        for i in range(n):
            _x = x[self.ROBOT_STATE_SIZE + i * self.LANDMARK_STATE_SIZE:self.ROBOT_STATE_SIZE + (i+1) * self.LANDMARK_STATE_SIZE]
            _p = p[self.ROBOT_STATE_SIZE + i * self.LANDMARK_STATE_SIZE:self.ROBOT_STATE_SIZE + (i+1) * self.LANDMARK_STATE_SIZE, self.ROBOT_STATE_SIZE + i * self.LANDMARK_STATE_SIZE:self.ROBOT_STATE_SIZE + (i+1) * self.LANDMARK_STATE_SIZE]

            m = self.get_error_ellipse_marker(_x, _p, i+1)
            self.error_ellipse.markers.append(m)

        self.marker_pub.publish(self.error_ellipse)

    def get_error_ellipse_marker(self, x, p, i):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = rospy.get_rostime()
        m.ns = "error_ellipse"
        m.action = Marker().ADD
        m.type = Marker().SPHERE
        m.lifetime = rospy.Duration()
        m.scale.z = 0.001
        m.color.a = 1
        m.color.g = 1
        m.id = i
        m.pose.position.x = x[0]
        m.pose.position.y = x[1]
        a, b, angle = self.calculate_error_ellipse(p[0:self.ROBOT_STATE_SIZE-1, 0:self.ROBOT_STATE_SIZE-1])
        q = self.get_quaternion_from_yaw(angle)
        m.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        m.scale.x = a
        m.scale.y = b
        return m

    def odom_callback(self, odom):
        with self.lock:
            current_time = rospy.get_time()
            dt = current_time - self.last_time
            self.last_time = current_time
            self.last_pose = self.current_pose
            self.current_pose = odom.pose.pose
            if(self.last_pose is not None):
                dx = self.current_pose.position.x - self.last_pose.position.x
                dy = self.current_pose.position.y - self.last_pose.position.y
                d = math.sqrt(dx * dx + dy * dy)
                v = d / dt
                _, _, current_yaw = tf.transformations.euler_from_quaternion((self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w))
                _, _, last_yaw = tf.transformations.euler_from_quaternion((self.last_pose.orientation.x, self.last_pose.orientation.y, self.last_pose.orientation.z, self.last_pose.orientation.w))
                omega = (current_yaw - last_yaw) / dt
                u = np.array([[v, omega]]).T
                self.x_est, self.p_est = self.predict(self.x_est, self.p_est, u, dt)
                self.publish_estimated_pose(self.x_est, self.p_est)

    def landmark_callback(self, lm):
        with self.lock:
            z = self.get_observation_from_landmark_msg(lm)
            self.x_est, self.p_est = self.update(self.x_est, self.p_est, z)
            self.publish_error_ellipse_markers(self.x_est, self.p_est)
            '''
            self.publish_estimated_pose(self.x_est, self.p_est)
            self.publish_error_ellipse_markers(self.x_est, self.p_est)
            '''

if __name__ == '__main__':
    odlm_slam = ObjectDetectionBasedLandmarkSLAM()
    odlm_slam.process()
