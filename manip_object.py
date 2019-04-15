#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
Brent Griffin (griffb@umich.edu)

Class for storing and using manipulation object information.
Class instances inherit properties of the initial object specified.
Class is defined such that object dictionary is extendable.
'''

import IPython

# TODO: Add other c3 objects and appropriate properties (ideally, just change elevation of pose).

class manipulation_objects:
	def __init__(self):
		self.object_dict = {
			'dollar bill': {
			'overlay_color': [0,255,0],
			'vision_model' : 'd.ckpt-10003',
			'place_pose'   : 'bl3.pk', 
			'position'     : [(-0.5582,0.5326,0.2055),(-0.3882,0.9442,-0.0519)],
			'thin_grip'    : True,
			'name'         : 'dollar bill'},
			'paper towel': {
			'overlay_color': [255,255,255],
			'vision_model' : 'p.ckpt-10003',
			'place_pose'   : 'br3.pk', 
			'position'     : [(0.3032,-0.1199,0.972),(0.48,0.109,1.0794)],
			'thin_grip'    : True,
			'name'         : 'paper towel'},
			'tong ends': {
			'overlay_color': [0,0,0],
			'vision_model' : 't2.ckpt-10003',
			'place_pose'   : 'bl3.pk', 
			'position'     : [(-0.1963,0.0395, 1.003),(-0.6922,0.3662,0.5386),(-0.5093, 0.7992, 0.4734)],
			'thin_grip'    : True,
			'name'         : 'tong ends'},
			'kitchen tongs': {
			'overlay_color': [0,0,0],
			'vision_model' : 't.ckpt-10003',
			'place_pose'   : 'bl3.pk', 
			'position'     : [(-0.1963,0.0395, 1.003),(-0.6922,0.3662,0.5386),(-0.5093, 0.7992, 0.4734)],
			'thin_grip'    : True,
			'name'         : 'kitchen tongs'},
			'place_pose'   : 'bl3.pk', 
			'red block': {
			'overlay_color': [0,0,255],
			'vision_model' : 'r.ckpt-10003',
			'place_pose'   : 'tl2.pk', 
			'position'     : [(-0.5327,0.5648,0.5903),(-0.5019,0.7865,0.4463)],
			'thin_grip'    : False,
			'name'         : 'red block'},
			'blue block': {
			'overlay_color': [255,0,0],
			'vision_model' : 'b.ckpt-10003',
			'place_pose'   : 'tl2.pk', 
			'position'     : [(-0.5582,0.5326,0.2055),(-0.3882,0.9442,-0.0519)],
			'thin_grip'    : False,
			'name'         : 'blue block'},
			'yellow block': {
			'overlay_color': [0,255,255],
			'vision_model' : 'y.ckpt-10003',
			'place_pose'   : 'tr2.pk', 
			'position'     : [(0.3458,-0.0431,1.6524),(0.5568,0.2032,1.672)],
			'thin_grip'    : False,
			'name'         : 'yellow block'}
			}

	def set_object(self, obj_name):
		if obj_name in self.object_dict.keys():
			instance_dict = self.object_dict[obj_name]
			self.properties = instance_dict.keys()
			for _, key in enumerate(self.properties):
				setattr(self, key, instance_dict[key])
		else:
			print ('Error: object instance %s currently undefined.' % obj_name)
