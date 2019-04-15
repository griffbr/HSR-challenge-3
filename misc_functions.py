#!/usr/bin/python
# -*- coding: utf-8 -*-

import cPickle as pickle; import IPython; import sys; import time; from copy import deepcopy; import time
import cv2; import numpy as np; import glob; import os; import math; import scipy
from skimage.measure import label
import matplotlib.pyplot as plt
import sys
from osvos import *
from data_subscriber import image_subscriber, state_subscriber, odometry_subscriber
from cyclops import *
import hsrb_interface;
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState, PointCloud2
from nav_msgs.msg import Odometry

## Functions. #########################################################################

def get_mask(img_sub, OSVOS):
	vs_img = deepcopy(img_sub._input_image)
	mask = OSVOS.segment_image(vs_img)
	mask, n_mask_pxls = largest_region_only(mask)
	return mask, n_mask_pxls, vs_img
def find_mask_centroid(mask):
	centroid_idx = np.array(scipy.ndimage.measurements.center_of_mass(mask))
	return centroid_idx
def largest_region_only(init_mask):
	labels = label(init_mask)
	bin_count = np.bincount(labels.flat)
	if len(bin_count)>1:
		mask_bin = np.argmax(bin_count[1:]) + 1
		n_mask_pixels = bin_count[mask_bin]
		single_mask = labels == mask_bin
	else: single_mask = init_mask; n_mask_pixels = 0
	return single_mask, n_mask_pixels
def mask_to_PCA(single_mask):
	mask_idx = np.where(single_mask)
	X = np.array([mask_idx[0],mask_idx[1]]).T
	M = np.mean(X, axis=0)
	cov_matrix = np.cov((X - M).T)
	eig_values, eig_vectors = np.linalg.eig(cov_matrix)
	eig_vectors[[0,1],[1,0]] *= -1
	PCA_idx = np.argmin(eig_values)
	PCA_angle = math.atan(eig_vectors[PCA_idx,1]/eig_vectors[PCA_idx,0])
	return X, M, eig_vectors, eig_values, PCA_angle
def write_seg_image(img_in, mask, file_name, overlay=[0,0,255], transparency=0.6):
	img = deepcopy(img_in) 
	for i, hue in enumerate(overlay):                                           
		img[mask,i] = hue*transparency + img[mask,i] * (1 - transparency)
	cv2.imwrite(file_name, img)
def mask_count(mask):
	return np.count_nonzero(mask > 0.6)

def load_pose(whole_body, pickle_file, filter_list = []):
	pose = pickle.load(open(pickle_file, 'rb'))
	for i, pose_name in enumerate(filter_list):
		del pose[pose_name]
	whole_body.move_to_joint_positions(pose)
def filter_pose(pose):
	del pose['base_roll_joint']
	del pose['hand_l_spring_proximal_joint']
	del pose['hand_r_spring_proximal_joint']
	del pose['base_l_drive_wheel_joint']
	del pose['base_r_drive_wheel_joint']
	return pose
def move_joint_amount(whole_body, joint, amount):
	pose = whole_body.joint_positions
	pose[joint] += amount
	pose = filter_pose(pose)
	whole_body.move_to_joint_positions(pose)
def save_pose(whole_body, pickle_file):
	pose = whole_body.joint_positions
	pose = filter_pose(pose)
	pickle.dump(pose, open(pickle_file, 'wb'))

def smart_grasp(whole_body, gripper, grip_min = -0.70, force = 0.5):
	print('applying force of ' + str(force)) 
	gripper.apply_force(force)
	#gripper.apply_force(0.5, delicate=True)
	init_grasp = whole_body.joint_positions['hand_motor_joint']
	print('init grasp is ' + str(init_grasp))
	hand_motor_joint = np.max([init_grasp, grip_min])
	print('commanding grasp of ' + str(hand_motor_joint))
	gripper.command(hand_motor_joint)
	return init_grasp
