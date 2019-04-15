#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy; import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import IPython; import cv2; from copy import deepcopy; import numpy as np; import cPickle as pickle
import os

class image_subscriber:
	def __init__(self, topic, is_color_image):
		topic_name = topic
		self._bridge = CvBridge()
		self.is_color_image = is_color_image
		self._input_image = None
		self._image_sub = rospy.Subscriber(topic_name, Image, self._color_image_cb)
		rospy.wait_for_message(topic_name, Image, timeout=5.0)
	def _color_image_cb(self, data):
		try:
			if self.is_color_image:
				self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
			else:
				self._input_image = self._bridge.imgmsg_to_cv2(data)
		except CvBridgeError as cv_bridge_exception:
			rospy.logerr(cv_bridge_exception)

class state_subscriber:
	def __init__(self, topic_name):
		self.state_sub = rospy.Subscriber(topic_name, JointState, self.state_cb)
		rospy.wait_for_message(topic_name, JointState, timeout=5.0)
	def state_cb(self, data):
		try:
			self.state = data
		except:
			print 'Error getting robot state.'

class odometry_subscriber:
	def __init__(self, topic_name):
		self.state_sub = rospy.Subscriber(topic_name, Odometry, self.odom_cb)
		rospy.wait_for_message(topic_name, Odometry, timeout=5.0)
	def odom_cb(self, data):
		try:
			self.odom = data
		except:
			print 'Error getting robot odometry.'

class voice_subscriber:
	def __init__(self, topic_name):
		self.grab_object = 'grab none'
		self.task_feedback = 'task none'
		self.grasp_config = 'config rotation'
		self.switch = 'switch none'
		self.message = 'none'
		self.voice_sub = rospy.Subscriber(topic_name, String, self.voice_cb)
	def voice_cb(self, data):
		self.message = data.data
		message_type = self.message.split()[0]
		if message_type == 'grab':
			self.grab_object = self.message
		elif message_type == 'task':
			self.task_feedback = self.message
		elif message_type == 'config':
			self.grasp_config = self.message
		elif message_type == 'switch':
			self.switch = self.message
