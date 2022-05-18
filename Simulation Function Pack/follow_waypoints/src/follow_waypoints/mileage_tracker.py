#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped, PoseWithCovariance
from std_msgs.msg import Empty, Float64
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time
from nav_msgs.msg import Odometry

class Tracker:
	def __init__(self):
		odom_topic = rospy.get_param('~odom_topic','/odom')
		mileage_pub_topic = rospy.get_param('~mileage_pub_topic','/mileage_travelled')
		self.mileage = 0.0
		self.pretick_pos = PoseWithCovariance()
		self.odom_listener = rospy.Subscriber(odom_topic, Odometry, self.calculate_journey_length)
		self.pubmileage = rospy.Publisher(mileage_pub_topic, Float64, queue_size=10)
		rospy.loginfo('mileage tracker: initialization done')


	def calculate_journey_length(self, odommsg):
		#rospy.loginfo('Recieved Coverage Path')
		deltax = odommsg.pose.pose.position.x - self.pretick_pos.pose.position.x
		deltay = odommsg.pose.pose.position.y - self.pretick_pos.pose.position.y
		delta_dis=math.sqrt(deltax ** 2 + deltay ** 2)
		if delta_dis < 0.001: # handle noise
			delta_dis =0
		self.mileage = self.mileage + delta_dis
		self.pretick_pos = odommsg.pose
		self.pubmileage.publish(self.mileage)
		#rospy.loginfo("Robot travelled %s m" % round(self.mileage, 2))

if __name__ == '__main__':
	rospy.loginfo('mileage tracker: program starts')
	rospy.init_node('mileage_tracker',anonymous=True)
	tracker = Tracker()
	rospy.spin()
