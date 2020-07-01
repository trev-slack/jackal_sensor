#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Sensor():
	def __init__(self):
		# get namespace
		self.ns = rospy.get_namespace()
		self.sensor()

	def noise(self,data):
		#get rid of the / on either side of the robot name
		robot_name = self.ns.replace("/","")
		# index of the robot in the gazebo/model_states data
		robot_index = data.name.index(robot_name)
		# isolate the robot's pose data
		gazebo_location = data.pose[robot_index]
		#add noise
		noise = np.random.normal(0,1)
		#publish the pose data
		self.pub.publish(gazebo_location)

	    

	def sensor(self):
		#create a publisher to push the adjusted sensor data to
		pubstr = self.ns+'virtual_Sensor/Pose'
		self.pub = rospy.Publisher(pubstr,Pose,queue_size=1)
		#subscribe to the gazebo model_states (this is the truth data)
		self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.noise)


if __name__ == '__main__':
	try:
		# Create ROS Node
		rospy.init_node('Virtual_Sensor', anonymous=True)
		#run the sensor
		mySensor=Sensor()
		#keep ros from exiting
		rospy.spin()
	except rospy.ROSInterruptException:
		pass