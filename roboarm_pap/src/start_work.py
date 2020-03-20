#!/usr/bin/env python 
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from object_recognition.msg import RecognizedObjects

class main_loop:

	def __init__(self):		
		#Start Commander
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interfice_tutorial',anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.object_pos_sub = rospy.Subscriber("recognized_objects",RecognizedObjects,self.callback, queue_size = 10)

	def callback(self,data):
		if (data.number_of_objects) > 0:
			x_pos = data.points_data[0][0]
			y_pos = data.points_data[0][1]
			z_pos = data.points_data[0][2]
			group_names = self.robot.get_group_names()
			#print "============ Available Planning Groups:", robot.get_group_names()
			
			#Go to the start position
			arm_group = moveit_commander.MoveGroupCommander("arm")
			arm_group.set_named_target("start")
			plan1 = arm_group.go()

			#Open hand
			hand_group = moveit_commander.MoveGroupCommander("hand")
			hand_group.set_named_target("open")
			plan2 = hand_group.go()

			#Position ahead object
			pose_target = geometry_msgs.msg.Pose()
			pose_target.orientation.w = 0.5
			pose_target.orientation.x = -0.5
			pose_target.orientation.y = 0.5
			pose_target.orientation.z = -0.5
			pose_target.position.x = x_pos
			pose_target.position.y = y_pos
			pose_target.position.z = 1.50

			arm_group.set_pose_target(pose_target)
			plan1 = arm_group.go()

			pose_target.position.z = 1.35
			arm_group.set_pose_target(pose_target)
			plan1 = arm_group.go()

			hand_group.set_named_target("close")
			plan2 = hand_group.go()

			pose_target.position.z = 1.5
			arm_group.set_pose_target(pose_target)
			plan1 = arm_group.go()

			#Go to the position above bin
			arm_group.set_named_target("trash")
			plan1 = arm_group.go()

			hand_group.set_named_target("close")
			plan2 = hand_group.go()

		else:
			#Stop Commander
			rospy.sleep(5)
			moveit_commander.roscpp_shutdown()
			#Shut down
			rospy.signal_shutdown('no objects')


#--------------- MAIN LOOP
def main(args):
	#--- Create the object to do loop
	loop = main_loop()

	#--- Initialize the ROS node
	rospy.init_node('roboarm_pap', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		#Stop Commander
		rospy.sleep(5)
		moveit_commander.roscpp_shutdown()
		print("Shutting down")
	


if __name__ == '__main__':
	main(sys.argv)
