#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
Brent Griffin (griffb@umich.edu)

Class for storing and using visual servoing configurations.
Class is defined such that object dictionary is extendable.
'''

import numpy as np
import IPython

# TODO: 
# Tune lowfar Je_pinv, suspect 0.001 -> 0.0004

class visual_servo:
	def __init__(self):
		self.config_dict = {
			'highnear': {
			's_des'    : [240,310],
			'Je_pinv'  : np.array([[0,-0.0012],[0.0012,0]]),
			'error_max': 150,
			'pose'     : 'initial_view.pk',
			'name'     : 'highnear'},
			'highfar': {
			's_des'    : [240,310],
			'Je_pinv'  : np.array([[0,0],[0.0012,0]]),
			'error_max': 30,
			'pose'     : 'initial_view.pk',
			'name'     : 'highfar'},
			'lownear': {
			's_des'    : [240,220],
			'Je_pinv'  : np.array([[0,-0.0003],[0.0003,0]]),
			'error_max': 30,
			'pose'     : 'ps3_x_grasp.pk',
			'name'     : 'lownear'},
			'lowfar': {
			's_des'    : [240,180],
			'Je_pinv'  : np.array([[0,-0.0005],[0.0002,0]]),
			'error_max': 20,
			'pose'     : 'ps3_tri_tilt_low.pk',
			'name'     : 'lownear'}
			}

	def set_config(self, name):
		if name in self.config_dict.keys():
			instance_dict = self.config_dict[name]
			self.properties = instance_dict.keys()
			for _, key in enumerate(self.properties):
				setattr(self, key, instance_dict[key])
		else:
			print ('Error: object instance %s currently undefined.' % name)

	def delta_q(self, s):
		error_logic = abs(self.Je_pinv).sum(axis=0) > 0
		e = self.s_des - s
		delta_q = np.matmul(self.Je_pinv, e)
		e_sum = np.sum(abs(e*error_logic))
		return delta_q, e_sum
		
