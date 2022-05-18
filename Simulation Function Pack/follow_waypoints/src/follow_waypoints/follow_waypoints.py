#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped
from std_msgs.msg import Empty, Float64
from tf import TransformListener
import dynamic_reconfigure.client
import tf
import math
import rospkg
import csv
import time
from geometry_msgs.msg import PoseStamped
import os.path

# change Pose with covariance to the correct frame 
def changePose(waypoint,target_frame):
	if waypoint.header.frame_id == target_frame:
		# already in correct frame
		return waypoint
	if not hasattr(changePose, 'listener'):
		changePose.listener = tf.TransformListener()
	tmp = PoseStamped()
	tmp.header.frame_id = waypoint.header.frame_id
	tmp.pose = waypoint.pose.pose
	try:
		changePose.listener.waitForTransform(
			target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
		pose = changePose.listener.transformPose(target_frame, tmp)
		ret = PoseWithCovarianceStamped()
		ret.header.frame_id = target_frame
		ret.pose.pose = pose.pose
		return ret
	except:
		rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
		exit()

# change Posestamp to the correct frame 
def changePoseStamp(waypoint_posestamp,target_frame):
	if waypoint_posestamp.header.frame_id == target_frame:
		# already in correct frame
		return waypoint_posestamp
	if not hasattr(changePoseStamp, 'listener'):
		changePoseStamp.listener = tf.TransformListener()
	tmp = PoseStamped()
	tmp.header.frame_id = waypoint_posestamp.header.frame_id
	tmp.pose = waypoint_posestamp.pose
	try:
		changePoseStamp.listener.waitForTransform(
			target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
		pose = changePoseStamp.listener.transformPose(target_frame, tmp)
		ret = PoseStamped()
		ret.header.frame_id = target_frame
		ret.pose = pose.pose
		return ret
	except:
		rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
		exit()
		
#Path for saving and retreiving the pose.csv file 
input_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/outputpose.csv"
file_exists = os.path.isfile(input_file_path) 
if not file_exists:
	f = open(input_file_path, "w")
	f.close()

output_log_handler = open(output_file_path,"w")
waypoints = []

class FollowPath(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
		self.frame_id = rospy.get_param('~goal_frame_id','map')
		self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
		self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
		self.mileage_pub_topic = rospy.get_param('~mileage_pub_topic','/mileage_travelled')
		self.duration = rospy.get_param('~wait_duration', 0.0)
		self.failure_limit = rospy.get_param('~consecutive_failure_threshold', 10)
		self.recovery_enable_dis_threshold = rospy.get_param('~recovery_enable_dis_threshold', 0.5)
		# Get a move_base action client and enable recovery behavior
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.recovery_allowed = True
		self.cfg_change_client = dynamic_reconfigure.client.Client("move_base")
		self.cfg_change_client.update_configuration({"recovery_behavior_enabled":self.recovery_allowed})
		#make sure initially the recovery mode is alway enabled
		self.last_recovery_mileage = -0.01 - self.recovery_enable_dis_threshold
		rospy.loginfo('Connecting to move_base...')
		self.client.wait_for_server()
		rospy.loginfo('Connected to move_base.')
		rospy.loginfo('Starting a tf listner.')
		self.tf = TransformListener()
		self.listener = tf.TransformListener()
		self.distance_tolerance = rospy.get_param('~smooth_tolerance', 0.0)
		self.path_deviance_warning_tolerance =  rospy.get_param('~deviation_tolerance', 0.1)
		self.angle_diff_threshold = rospy.get_param('~angle_diff_threshold', 0.0)
		rospy.loginfo('current and next goal angle tolerance in degree: %s' % (self.angle_diff_threshold/3.14*180.0))

	def execute(self, userdata):
		global waypoints
		global output_log_handler
		# Execute waypoints each in sequence
		consecutive_failure_counter = 0
		previous_goal_pass = True #logging goal status, if consecutive failing goals number > 
		expected_travel_dis = 0.0
		real_travel_dis = 0.0
		for count,waypoint in enumerate(waypoints):
			current_goal_pass = True
			# Break if preempted
			if waypoints == []:
				rospy.loginfo('The waypoint queue has been reset.')
				break
			# Otherwise publish next waypoint as goal
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = self.frame_id
			goal.target_pose.pose.position = waypoint.pose.pose.position
			goal.target_pose.pose.orientation = waypoint.pose.pose.orientation

			smooth_distance_threshold = 0.0
			#rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' % (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
			rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
			
			now = rospy.Time.now()
			self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
			trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
			if self.distance_tolerance > 0.0:
				current_loc_theta = tf.transformations.euler_from_quaternion(rot)[2]
				#below has bug. should use current location to decide smooth distance
				current_goal_quaternion = (waypoint.pose.pose.orientation.x, waypoint.pose.pose.orientation.y, waypoint.pose.pose.orientation.z, waypoint.pose.pose.orientation.w)
				current_goal_theta = tf.transformations.euler_from_quaternion(current_goal_quaternion)[2]
				if count + 1 < len(waypoints):
					try:
						next_goal_theta = 10.0 
						next_waypoint = waypoints[count+1]
						next_goal_quaternion = (next_waypoint.pose.pose.orientation.x, next_waypoint.pose.pose.orientation.y, next_waypoint.pose.pose.orientation.z, next_waypoint.pose.pose.orientation.w)
						next_goal_theta = tf.transformations.euler_from_quaternion(next_goal_quaternion)[2]
						if abs(next_goal_theta - current_loc_theta) < self.angle_diff_threshold and abs(next_goal_theta - current_goal_theta) < self.angle_diff_threshold:
							smooth_distance_threshold = self.distance_tolerance
					except ValueError:
						rospy.loginfo("can't calculate next goal angle, maybe already at the end of list")
			expected_travel_dis = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))
			rospy.loginfo('Expected distance to goal: %s' % expected_travel_dis)

			goal_start_mileage = rospy.wait_for_message(self.mileage_pub_topic, Float64)
			goal_end_mileage = goal_start_mileage
			if (goal_start_mileage.data - self.last_recovery_mileage) > self.recovery_enable_dis_threshold:
				self.recovery_allowed = True				
			else:
				self.recovery_allowed = False
			self.cfg_change_client.update_configuration({"recovery_behavior_enabled":self.recovery_allowed})
			self.client.send_goal(goal)
			rospy.loginfo('smooth distance tolerance: %s' % smooth_distance_threshold)
			rospy.loginfo('recovery enable status: %s' % self.recovery_allowed)
			if not smooth_distance_threshold > 0.0:
				self.client.wait_for_result()
				actionlib_goal_status = self.client.get_state() # if return 3 means goal succeed
				#actionlib_goal_status = self.client.get_goal_status_text() 
				if actionlib_goal_status != 3: 
					current_goal_pass = False
					rospy.logwarn("Current Goal is not achieveable, skip to the next goal")
					if self.recovery_allowed == True:
						self.last_recovery_mileage = rospy.wait_for_message(self.mileage_pub_topic, Float64).data
				else:
					rospy.loginfo("Waiting for %f sec..." % self.duration)
					time.sleep(self.duration)
				goal_end_mileage = rospy.wait_for_message(self.mileage_pub_topic, Float64)
			else:
				#This is the loop which exist when the robot is near a certain GOAL point.
				distance = 10
				while(distance > smooth_distance_threshold):
					rospy.sleep(0.1)
					try:
						cur_feedback = rospy.wait_for_message("/move_base/feedback",MoveBaseActionFeedback,timeout = 2)
						cur_status = cur_feedback.status.status
						cur_posestamp = changePoseStamp(cur_feedback.feedback.base_position,"map")
						cur_location = cur_posestamp.pose.position
						quaternion = (cur_posestamp.pose.orientation.x, cur_posestamp.pose.orientation.y, \
						cur_posestamp.pose.orientation.z, cur_posestamp.pose.orientation.w)
						cur_theta = tf.transformations.euler_from_quaternion(quaternion)[2]
						distance = math.sqrt(pow(waypoint.pose.pose.position.x-cur_location.x,2)+pow(waypoint.pose.pose.position.y-cur_location.y,2))
						rospy.loginfo('Line Following, current distance to goal: %s' % distance)
						cur_status = rospy.wait_for_message("/move_base/status",GoalStatusArray,timeout = 2)
						#print cur_status
						if cur_status.status_list[0].status == 4:
							rospy.logwarn('Line Following, detected path planning failure, force smooth_distance tolerance to 0')
							self.client.wait_for_result()
							actionlib_goal_status = self.client.get_state()
							if actionlib_goal_status != 3: 
								current_goal_pass = False
								rospy.logwarn("Line Following, Current Goal is not achieveable, skip to the next goal")
							else:
								rospy.loginfo("Waiting for %f sec..." % self.duration)
								time.sleep(self.duration)
							break
					except rospy.ROSException as e:
						if 'timeout exceeded' in e.message:
							rospy.logwarn("Line Following, pos tracking timeout, skip to the next goal")
							current_goal_pass = False
							break
						else:
							current_goal_pass = False
							raise e 
				goal_end_mileage = rospy.wait_for_message(self.mileage_pub_topic, Float64)
				

			if (not (current_goal_pass or previous_goal_pass)):
				consecutive_failure_counter = consecutive_failure_counter + 1
			else:
				consecutive_failure_counter = 0
			
			previous_goal_pass = current_goal_pass
			real_travel_dis = goal_end_mileage.data - goal_start_mileage.data
			if (abs(real_travel_dis - expected_travel_dis) > self.path_deviance_warning_tolerance):
				rospy.logwarn("Path Follow Is Not Successful, expected traveling %s m, real travelling %s m" % (expected_travel_dis,real_travel_dis))
			output_log_handler.write(str(waypoint.pose.pose.position.x) + ',' + str(waypoint.pose.pose.position.y) \
				+ ',' + str(waypoint.pose.pose.position.z) + ',' + str(waypoint.pose.pose.orientation.x) + \
				',' + str(waypoint.pose.pose.orientation.y) + ',' + str(waypoint.pose.pose.orientation.z) +\
				',' + str(waypoint.pose.pose.orientation.w)+ ',' + str(current_goal_pass) +"," + str(round(expected_travel_dis,2))\
				 + "," + str(round(real_travel_dis,2)) + "," + str(round(smooth_distance_threshold ,3)) +'\n')
			
			if consecutive_failure_counter > self.failure_limit:
				rospy.logfatal("Failed %s goals in a roll, Robot stuck. " % self.failure_limit)
				break
					#now = rospy.Time.now()
					#self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
					#trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
					#distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))
					#rospy.loginfo('current distance to goal: %s' % distance)
		return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
	"""Used to publish waypoints as pose array so that you can see them in rviz, etc."""
	poses = PoseArray()
	poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
	poses.poses = [pose.pose.pose for pose in waypoints]
	return poses

class GetPath(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
		# Subscribe to pose message to get new waypoints
		self.addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
		# Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
		self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
		self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

		# Start thread to listen for reset messages to clear the waypoint queue
		def wait_for_path_reset():
			"""thread worker function"""
			global waypoints
			while not rospy.is_shutdown():
				data = rospy.wait_for_message('/path_reset', Empty)
				rospy.loginfo('Recieved path RESET message')
				self.initialize_path_queue()
				rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
							   # for three seconds and wait_for_message() in a
							   # loop will see it again.
		reset_thread = threading.Thread(target=wait_for_path_reset)
		reset_thread.start()

	def initialize_path_queue(self):
		global waypoints
		waypoints = [] # the waypoint queue
		# publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
		self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

	def execute(self, userdata):
		global waypoints
		self.initialize_path_queue()
		self.path_ready = False

		# Start thread to listen for when the path is ready (this function will end then)
		# Also will save the clicked path to pose.csv file
		def wait_for_path_ready():
			"""thread worker function"""
			data = rospy.wait_for_message('/path_ready', Empty)
			rospy.loginfo('Recieved path READY message')
			self.path_ready = True
			with open(input_file_path, 'w') as file:
				for current_pose in waypoints:
					file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) \
					+ ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + \
					',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) +\
					 ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
			rospy.loginfo('poses written to '+ input_file_path)
		ready_thread = threading.Thread(target=wait_for_path_ready)
		ready_thread.start()

		self.start_journey_bool = False

		# Start thread to listen start_jorney 
		# for loading the saved poses from follow_waypoints/saved_path/poses.csv
		def wait_for_start_journey():
			"""thread worker function"""
			data_from_start_journey = rospy.wait_for_message('start_journey', Empty)
			rospy.loginfo('Recieved path READY start_journey')
			with open(input_file_path, 'r') as file:
				reader = csv.reader(file, delimiter = ',')
				for row in reader:
					print row
					current_pose = PoseWithCovarianceStamped() 
					current_pose.pose.pose.position.x	 =	float(row[0])
					current_pose.pose.pose.position.y	 =	float(row[1])
					current_pose.pose.pose.position.z	 =	float(row[2])
					current_pose.pose.pose.orientation.x = float(row[3])
					current_pose.pose.pose.orientation.y = float(row[4])
					current_pose.pose.pose.orientation.z = float(row[5])
					current_pose.pose.pose.orientation.w = float(row[6])
					waypoints.append(current_pose)
					self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
			self.start_journey_bool = True
			
			
		start_journey_thread = threading.Thread(target=wait_for_start_journey)
		start_journey_thread.start()

		topic = self.addpose_topic
		rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
		rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
		rospy.loginfo("OR")
		rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")


		# Wait for published waypoints or saved path  loaded
		while (not self.path_ready and not self.start_journey_bool):
			try:
				pose = rospy.wait_for_message(topic, PoseStamped, timeout=1)
				cur_pose_w_cov = PoseWithCovarianceStamped() 
				cur_pose_w_cov.header.frame_id = pose.header.frame_id
				cur_pose_w_cov.pose.pose.position.x	 =  pose.pose.position.x
				cur_pose_w_cov.pose.pose.position.y	 =  pose.pose.position.y
				cur_pose_w_cov.pose.pose.position.z	 =  pose.pose.position.z
				cur_pose_w_cov.pose.pose.orientation.x  =  pose.pose.orientation.x 
				cur_pose_w_cov.pose.pose.orientation.y  =  pose.pose.orientation.y 
				cur_pose_w_cov.pose.pose.orientation.z  =  pose.pose.orientation.z 
				cur_pose_w_cov.pose.pose.orientation.w  =  pose.pose.orientation.w 
				
			except rospy.ROSException as e:
				if 'timeout exceeded' in e.message:
					continue  # no new waypoint within timeout, looping...
				else:
					raise e
			rospy.loginfo("Recieved new waypoint")
			waypoints.append(changePose(cur_pose_w_cov, "map"))
			# publish waypoint queue as pose array so that you can see them in rviz, etc.
			self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

		# Path is ready! return success and move on to the next state (FOLLOW_PATH)
		return 'success'

class PathComplete(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		global output_log_handler
		output_log_handler.close() # close log file
		rospy.loginfo('###############################')
		rospy.loginfo('##### REACHED FINISH GATE #####')
		rospy.loginfo('###############################')
		return 'success'

def main():
	rospy.init_node('follow_waypoints')

	sm = StateMachine(outcomes=['success'])

	with sm:
		StateMachine.add('GET_PATH', GetPath(),
						   transitions={'success':'FOLLOW_PATH'},
						   remapping={'waypoints':'waypoints'})
		StateMachine.add('FOLLOW_PATH', FollowPath(),
						   transitions={'success':'PATH_COMPLETE'},
						   remapping={'waypoints':'waypoints'})
		StateMachine.add('PATH_COMPLETE', PathComplete(),
						   transitions={'success':'GET_PATH'})

	outcome = sm.execute()
