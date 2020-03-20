#!/usr/bin/env python 
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from object_recognition.msg import RecognizedObjects

#Start Commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interfice_tutorial',anonymous=True)
robot = moveit_commander.RobotCommander()

group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

arm_group = moveit_commander.MoveGroupCommander("arm")
#Go to the start position
arm_group.set_named_target("start")
plan1 = arm_group.go()
#Open hand

hand_group = moveit_commander.MoveGroupCommander("hand")
hand_group.set_named_target("open")
plan2 = hand_group.go()

#Set Pose Hand made can grasping

#Position ahead can
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.33
pose_target.position.y = 0.-15
pose_target.position.z = 1.50

arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

pose_target.position.z = 1.35
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

hand_group.set_named_target("close")
plan3 = hand_group.go()

pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

#Stop Commander
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
