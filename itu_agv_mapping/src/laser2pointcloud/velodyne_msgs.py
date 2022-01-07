#!/usr/bin/env python3

import rospy
from rospy.topics import Publisher
from sensor_msgs.msg import PointCloud 
class VelodyneData():

    def __init__(self):
        

        rospy.init_node('velodyne_data_publisher')
        rospy.Subscriber("/velodyne/laser/scan",PointCloud,self.velodyne_data_pub)

        self.data_pub =rospy.Publisher('velodyne_data',PointCloud,queue_size=10)



    
    def velodyne_data_pub(self,msg):


        data = msg
        data.header.frame_id = "Velodyne"

        self.data_pub.publish(data)


        
        
        
VelodyneData()
rospy.spin()