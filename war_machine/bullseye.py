#!/usr/bin/env python
import rospy, rosnode
import hsrb_interface
from tmc_navigation_msgs.msg import OccupancyGridUint
import numpy as np

if __name__ == '__main__':
    rospy.init_node('bullseye')
    robot = hsrb_interface.Robot()
    base = robot.try_get("omni_base")

    rosnode.kill_nodes(['/obstacle_grid_mapper'])
    rate = rospy.Rate(1)
    n = 1
    msg = OccupancyGridUint()
    msg.data = tuple(np.zeros((140 * 140), dtype=np.uint8))
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    msg.info.map_load_time = rospy.Time(0)
    msg.info.resolution = 0.05
    msg.info.width = 140
    msg.info.height = 140
    msg.info.origin.orientation.x = 0
    msg.info.origin.orientation.y = 0
    msg.info.origin.orientation.z = 0
    msg.info.origin.orientation.w = 1

    while not rospy.is_shutdown():

        msg.header.seq = n
        pose = base.pose
        msg.info.origin.position.x = -3.5 + pose[0]
        msg.info.origin.position.y = -3.5 + pose[1]
        msg.info.origin.position.z = 0

        pub = rospy.Publisher('dynamic_obstacle_map', OccupancyGridUint, queue_size=10, latch=True)
        pub.publish(msg)
        print('msg published')
        n += 1
        rate.sleep()

