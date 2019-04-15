import rospy
import geometry_msgs.msg
import hsrb_interface
from math import pi, cos, sin
import tf_conversions
import numpy as np
import IPython
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import String

def init_amcl(base):
	base.go_rel(0, 0, -pi / 4)
	base.go_rel(0, 0, pi / 2)
	base.go_rel(0, 0, -pi / 4)
def init_amcl_fast(base, frac):
	base.go_rel(0, 0, frac*pi)
	base.go_rel(0, 0, -frac*pi)
def init_amcl_360(base):
	base.go_rel(0,0, pi)
	base.go_rel(0,0, pi)	
def get_amcl_pose():
	stop_client = rospy.ServiceProxy('/request_nomotion_update', Empty)
	stop_client.call(EmptyRequest())
	msg = rospy.wait_for_message("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped)
	pose = msg.pose.pose
	x = pose.position.x
	y = pose.position.y
	quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
	theta = tf_conversions.transformations.euler_from_quaternion(quaternion)
	return (x,y,theta[2])
def go_to_amcl_pose(base, end_amcl_pose):
	current_amcl_pose = get_amcl_pose()
	(x_rob, y_rob, thet_rob) = current_amcl_pose
	
	(del_x, del_y, del_thet) = np.subtract(end_amcl_pose, current_amcl_pose)
	rel = (cos(thet_rob)*del_x + sin(thet_rob)*del_y, -sin(thet_rob)*del_x + cos(thet_rob)*del_y, del_thet)
	base.go_rel(rel[0], rel[1], rel[2])
def go_to_map_home(base):
	msg = rospy.wait_for_message("/lab_map_file", String)
	file_name = msg.data.split('/')[-1]
	file_origins = {
		"open_door_map.yaml": (-0.16252551477656937, 0.19710061801234022, 0.9766003077757),
		"closed_door_map.yaml": (-0.3736979638999808, -0.7756700053752261, -2.3591506944974125) 
	}	
	go_to_amcl_pose(base, file_origins[file_name])

if __name__=='__main__':
	with hsrb_interface.Robot() as robot:
        	base = robot.try_get('omni_base')
		IPython.embed()
