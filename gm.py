#!/usr/bin/python
# -*- coding: utf-8 -*-

# Functions for mask-based grasp selection.
# 181004
# Questions? griffb@umich.edu

import numpy as np
import imutils
import cv2
import os
import glob
import IPython

def combine_masks(mask_list):
	n_masks = len(mask_list)
	mask_out = mask_list[0]
	for i in range(1,n_masks):
		mask_out = mask_out | mask_list[i]
	return mask_out > 0

def select_grasp_angle(object_mask, grasp_dir):
	object_mask = np.atleast_3d(object_mask)[...,0]
	grasp_img_list = glob.glob(os.path.join(grasp_dir, '*.png'))
	n_grasps = len(grasp_img_list)
	grasp_cost = np.zeros(n_grasps)
	for i, grasp_img in enumerate(grasp_img_list):
		grasp_candidate = np.atleast_3d(cv2.imread(grasp_img))[...,0]
		grasp_cost[i] = eval_intersect(grasp_candidate, object_mask)
	best_grasp_img = grasp_img_list[np.argmin(grasp_cost)]
	grasp_angle = float(best_grasp_img.split('/')[-1].split('.')[0])
	return grasp_angle

def eval_intersect(mask1, mask2):
	msk1 = mask1.astype(np.bool)
	msk2 = mask2.astype(np.bool)
	return np.sum((msk1 & msk2))

def grasp_angle_to_pm90(grasp_angle, angle_mod = 90):
	if grasp_angle < -angle_mod:
		grasp_angle %= angle_mod
	elif grasp_angle > angle_mod:
		grasp_angle = grasp_angle % angle_mod - angle_mod
	return grasp_angle
