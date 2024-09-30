#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

class AMCLNode:
    def __init__(self):
        rospy.init_node('amcl_node', anonymous=True)

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.particle_pub = rospy.Publisher("/particle_cloud", PoseArray, queue_size=10)
        
        # tham so cua bo loc hat
        self.num_particles = 100
        self.particles = np.zeros((self.num_particles, 3))  # 3 tham so (x, y, theta)

        # du lieu raw lay tu phan cung
        self.laser_scan = None
        self.odom = None

    def odom_callback(self, msg):
        # cap nhat du lieu tu cac node odem
        self.odom = msg
        self.motion_update()

    def scan_callback(self, msg):
        # cap nhat du lieu tu camera
        self.laser_scan = msg
        self.sensor_update()

    def motion_update(self):
        # gia dinh chuyen dong va cap nhat lai vi tri cac hat dua tren cac node odom
        for i in range(self.num_particles):
            delta_x = np.random.normal(0, 0.05)
            delta_y = np.random.normal(0, 0.05)
            delta_theta = np.random.normal(0, 0.01)
            self.particles[i, 0] += delta_x
            self.particles[i, 1] += delta_y
            self.particles[i, 2] += delta_theta

    def sensor_update(self):
        #  cap nhat lai trong so
        if self.laser_scan is None:
            return
        # tinh toan sau do so sanh de gan lai trong so cho cac hat co vi tri gan voi vi tri thưc te hon
        weights = np.ones(self.num_particles)
        for i in range(self.num_particles):
            weights[i] = np.random.random()

        weights /= np.sum(weights)

        # lay mau
        indices = np.random.choice(self.num_particles, self.num_particles, p=weights)
        self.particles = self.particles[indices]

        self.publish_particles()

    def publish_particles(self):
        # chuyen sang kieu du lieu PoseArray de quan sat tren RViz
        particle_cloud = PoseArray()
        particle_cloud.header.frame_id = "map"
        particle_cloud.header.stamp = rospy.Time.now()

        for i in range(self.num_particles):
            pose = PoseStamped()
            pose.pose.position.x = self.particles[i, 0]
            pose.pose.position.y = self.particles[i, 1]

            # theta sang quaternion
            quaternion = quaternion_from_euler(0, 0, self.particles[i, 2])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            particle_cloud.poses.append(pose.pose)

        self.particle_pub.publish(particle_cloud)

if __name__ == '__main__':
    try:
        amcl_node = AMCLNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
