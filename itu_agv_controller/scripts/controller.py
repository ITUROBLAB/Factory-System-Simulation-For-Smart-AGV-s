#!/usr/bin/env python3

from numpy.lib import twodim_base
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import atan2
import threading


class Diff_Drive_Controller():

    def __init__(self) :
        

        rospy.init_node('agv_controller')
        rospy.Subscriber('/AGV_1/odom',Odometry,self.odom_tf)
        self.cmd_pub = rospy.Publisher('/AGV_1/cmd_vel',Twist,queue_size=10)
        

        rospy.sleep(2)

    def odom_tf(self,msg):
        
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y

        rot_agv = msg.pose.pose.orientation

        roll,pitch,self.theta = euler_from_quaternion([rot_agv.x,rot_agv.y,rot_agv.z,rot_agv.w])

    def move_agv(self,goal_pos):

        
        ## goal_pos will be Point() msg later.
        diff_x = goal_pos[0] - self.x_current
        diff_y = goal_pos[1] - self.y_current

        diff_drive_cmd = Twist()

        
        goal_reached = False
        goal_theta = atan2(diff_y,diff_x)

        
        while not goal_reached:
           try:
            rospy.loginfo('goal_theta {} and theta {} and diff {}'.format(goal_theta,self.theta,goal_theta-self.theta))
            if abs(goal_theta - self.theta) < 0.005:
                diff_drive_cmd.linear.x = 0.0
                diff_drive_cmd.angular.z = 0.01

            elif goal_pos[0]== self.x_current and goal_pos[1] == self.y_current: 
                goal_reached = True
                diff_drive_cmd.linear.x = 0.0
                diff_drive_cmd.angular.z = 0.0    
            else:
                diff_drive_cmd.linear.x = 0.7
                diff_drive_cmd.angular.z = 0.0

            self.cmd_pub.publish(diff_drive_cmd)    

           except (KeyboardInterrupt):
                diff_drive_cmd.linear.x = 0.0
                diff_drive_cmd.angular.z = 0.0

                self.cmd_pub.publish(diff_drive_cmd)   
                break
             



while not rospy.is_shutdown():

    try:
        control = Diff_Drive_Controller()

        control.move_agv([7,1])
    except KeyboardInterrupt:
        break

