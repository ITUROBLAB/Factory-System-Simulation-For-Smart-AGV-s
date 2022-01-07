#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2 , LaserScan
from laser_geometry import LaserProjection

class Laser2PointCloud():
    def __init__(self):
        rospy.init_node('laser2pointcloud')
        self.project = LaserProjection()
        self.pointcloud1_pub = rospy.Publisher('laser/point_cloud',PointCloud2,queue_size=10)
        self.pointcloud2_pub = rospy.Publisher('laser_2/point_cloud2',PointCloud2,queue_size=10)

        self.lasersub1 =rospy.Subscriber('/lms/laser/scan',LaserScan,self.lasercb)
        self.lasersub2 =rospy.Subscriber('/lms/laser_2/scan',LaserScan,self.lasercb2)

    
    def lasercb(self,msg):

        cloud_data = self.project.projectLaser(msg)

        self.pointcloud1_pub.publish(cloud_data)

    def lasercb2(self,msg):

        cloud_data = self.project.projectLaser(msg)

        self.pointcloud2_pub.publish(cloud_data)


Laser2PointCloud()
rospy.spin()