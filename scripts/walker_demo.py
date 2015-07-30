#!/usr/bin/env python

import rospy
from phantomx_gazebo.phantomx import PhantomX


if __name__ == '__main__':
    rospy.init_node('walker_demo')

    rospy.loginfo('Instantiating robot Client')
    robot = PhantomX()
    rospy.sleep(1)

    rospy.loginfo('Walker Demo Starting')

    robot.set_walk_velocity(0.2, 0, 0)
    rospy.sleep(3)
    robot.set_walk_velocity(1, 0, 0)
    rospy.sleep(3)
    robot.set_walk_velocity(0, 1, 0)
    rospy.sleep(3)
    robot.set_walk_velocity(0, -1, 0)
    rospy.sleep(3)
    robot.set_walk_velocity(-1, 0, 0)
    rospy.sleep(3)
    robot.set_walk_velocity(1, 1, 0)
    rospy.sleep(5)
    robot.set_walk_velocity(0, 0, 0)

    rospy.loginfo('Walker Demo Finished')
