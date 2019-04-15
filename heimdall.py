#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
Brent Griffin (griffb@umich.edu)

Script for running HSR challenge 3 (181212).
'''

import IPython
import time

from manip_object import *
from vs import *
from osvos import *
from gm import *

# With ROS depend.
from data_subscriber import *
from cyclops import *
from misc_functions import *
import hsrb_interface

# Misc. parameters.
IV_MAX = 350
VS_PXL_MIN = 200
GRIP_MIN = -0.8
POSE_DIR = './data/pose/'
GRASP_DIR = './data/g_msk/'
VIS_DIR = './data/models/'	

# Vision functions.
def initial_object_location():
	seg.change_model(VIS_DIR + obj.vision_model)
	track_object()
	obj.init_center = deepcopy(obj.mask_center)
def track_object():
	img = deepcopy(grasp_cam._input_image)
	mask_location(img)
def mask_location(img):
	mask = seg.segment_image(img)
	obj.mask, obj.n_mask_pixels = largest_region_only(mask)
	obj.mask_center = find_mask_centroid(obj.mask)

# Grab object.
def grab_object():
	initial_object_location()
	if obj.init_center[1] < IV_MAX: reach = 'near'
	else: reach = 'far'
	center_on_object('high'+reach)
	center_on_object('low'+reach)
	grasp_object(reach)
def center_on_object(config):
	vs.set_config(config)
	load_pose(whole_body, POSE_DIR + vs.pose)
	error_sum = 1000; scale = 1; rst_pose = False
	while error_sum > vs.error_max * scale:
		if not voice.switch == 'switch none':
			switch_object(); scale = 1
		track_object()
		if obj.n_mask_pixels > VS_PXL_MIN:
			dq, error_sum = vs.delta_q(obj.mask_center)
			base.go_rel(dq[0], dq[1], 0)
			scale *= 1.2
			if rst_pose:
				load_pose(whole_body, POSE_DIR + vs.pose)
				rst_pose = False	
		else:
			move_joint_amount(whole_body, 'arm_lift_joint', 0.05)
			base.go_rel(-0.1,0,0)
			rst_pose = True
def grasp_object(reach):
	tts.say('Grasping %s, let me know how this goes!' % obj.name)
	object_grasped = False
	while not object_grasped:
		center_on_object('low'+reach)
		rotate_grip(reach)
		object_grasped = try_grasp()
		if not voice.switch == 'switch none':
			switch_object()
	voice.grasp_config = 'config rotation'
def try_grasp():
	grasped = False; answered = False
	try:
		init_grip = smart_grip()
		if init_grip > GRIP_MIN or obj.thin_grip:
			move_joint_amount(whole_body, 'arm_lift_joint', 0.2)
			gripper.apply_force(0.5)
			while not answered:
				answer = voice.task_feedback.split(' ', 1)[1]
				if answer == 'none':
					time.sleep(0.250)
				else:
					answered = True
					if answer == 'success':
						grasped = True
						#tts.say('Object grasped!')
					voice.task_feedback = 'task none'
		else:
			gripper.command(1.0)
	except:
		tts.say('I could not grasp the %s that time.' % obj.name)
	return grasped
def smart_grip(grip_min = -0.70, force = 0.5):
	gripper.apply_force(force)
	init_grip = whole_body.joint_positions['hand_motor_joint']
	grip_pos = np.max([init_grip, grip_min])
	gripper.command(grip_pos)
	return init_grip
def rotate_grip(reach):
	config = voice.grasp_config.split(' ', 1)[1]
	if reach == 'far' or config == 'horizontal':
		grasp_angle = 0.0
	elif config == 'vertical':
		grasp_angle = 1.5708
	else:
		grasp_angle = wrist_grip_angle()
	whole_body.move_to_joint_positions({'wrist_roll_joint': grasp_angle})
def wrist_grip_angle():
	track_object()
	grasp_ang = np.radians(select_grasp_angle(obj.mask, GRASP_DIR))
	cur_ang = whole_body.joint_positions['wrist_roll_joint']
	cmd_wrist_angle = grasp_angle_to_pm90(cur_ang-grasp_ang, angle_mod=1.5708)
	return cmd_wrist_angle
def switch_object():
	target_name = voice.switch.split(' ', 1)[1]
	obj.set_object(target_name)
	seg.change_model(VIS_DIR + obj.vision_model)
	voice.switch = 'switch none'

# Place object.
def place_object():
	placed = False
	while not placed:
		placed = move_and_place()	
def move_and_place():
	placed = False
	try:
		load_pose(whole_body, POSE_DIR + obj.place_pose, ['hand_motor_joint'])
		for _, pos in enumerate(obj.position):
			go_to_amcl_pose(base, pos)
		#IPython.embed()
		tts.say('%s has been placed. What should I do next?' % obj.name)
		gripper.command(1.0)
		placed = True
		go_to_amcl_pose(base, obj.position[-2])
		# get_amcl_pose()
	except:
		tts.say('Cannot move to desired location.')	
		base.go_rel(-0.1,0,0)
	return placed

# Used in main.
def set_object():
	target_name = voice.grab_object.split(' ', 1)[1]
	obj.set_object(target_name)
	voice.grab_object = 'grab none'
def return_home():
	go_to_map_home(base)
	load_pose(whole_body, POSE_DIR + 'initial_view.pk')

# Main challenge script.
def	main():
	# Misc. Initialization.
	#load_pose(whole_body, POSE_DIR + 'init_pose2.pk')
	print('\n\nRobot moves next, make sure that you are ready!\n\n')
	load_pose(whole_body, POSE_DIR + 'initial_view.pk')

	asked = False
	while True:
		# Get voice specification of target object.
		if voice.grab_object == 'grab none':
			if not asked:
				tts.say('Which object should I grab?')
				asked = True
			time.sleep(0.250)
			continue
		set_object()
		grab_object()
		place_object()
		return_home()
		asked = False;

if __name__ == '__main__':
	with hsrb_interface.Robot() as robot:
		base = robot.try_get('omni_base')
		whole_body = robot.get('whole_body')
		gripper = robot.get('gripper')
		tts = robot.try_get('default_tts')
		tts.language = tts.ENGLISH
		grasp_cam = image_subscriber('/hsrb/hand_camera/image_raw', True)
		robot_state = state_subscriber('/hsrb/joint_states')
		base_odom = odometry_subscriber('/hsrb/odom')
		voice = voice_subscriber('/zavengers/jarvis')
		seg = osvos_seg('./data/models/r.ckpt-10003')
		obj = manipulation_objects()
		vs = visual_servo()
		main()
