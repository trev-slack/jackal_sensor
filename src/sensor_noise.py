#!/usr/bin/env python
import rospy
#for random number generation
import numpy as np
#transform global to local coords
import tf
#message from gazebo/model_states
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class Sensor():
	def __init__(self):
		# get namespace
		self.ns = rospy.get_namespace()
		self.sensor()

	def noise(self,data):
		#get transformation to local coordinates
		tf_listener = tf.TransformListener()
		#target node
		tg_node = self.ns[1:-1]+'/odom'
		(trans,rot) = tf_listener.lookupTransform(self.ns,'map',rospy.Time(0))
		#add noise, each pose has 7 data points * number of objects in gazebo
		noise = np.random.normal(0,0.05,12*len(data.name))
		#multiply each state by a noise value
		for i in range(len(data.name)):
			#modify pose message type
			(data.pose[i]).position.x = (data.pose[i]).position.x+noise[i]
			(data.pose[i]).position.y = (data.pose[i]).position.y+noise[i+1]
			(data.pose[i]).position.z = (data.pose[i]).position.z+noise[i+2]
			(data.pose[i]).orientation.x = (data.pose[i]).orientation.x+noise[i+3]
			(data.pose[i]).orientation.y = (data.pose[i]).orientation.y+noise[i+4]
			(data.pose[i]).orientation.z = (data.pose[i]).orientation.z+noise[i+5]
			(data.pose[i]).orientation.w = (data.pose[i]).orientation.w+noise[i+6]
			#modify geometry/Twist message type
			(data.twist[i]).linear.x = (data.twist[i]).linear.x+noise[i+7]
			(data.twist[i]).linear.y = (data.twist[i]).linear.y+noise[i+8]
			(data.twist[i]).linear.z = (data.twist[i]).linear.z+noise[i+9]
			(data.twist[i]).angular.x = (data.twist[i]).angular.x+noise[i+10]
			(data.twist[i]).angular.y = (data.twist[i]).angular.y+noise[i+11]
			(data.twist[i]).angular.z = (data.twist[i]).angular.z+noise[i+12]

		#publish the pose data
		self.pub.publish(data)

	    

	def sensor(self):
		#create a publisher to push the adjusted sensor data to
		pubstr = self.ns+'virtual_sensor'
		self.pub = rospy.Publisher(pubstr, ModelStates, queue_size=1)
		#subscribe to odom in order to create transform
		odom_node = self.ns+'odom'
		self.tf_sub = rospy.Subscriber(odom_node,Twist,self.transform_handle)
		#subscribe to the gazebo model_states (this is the truth data)
		self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.noise)
	
	def transform_handle(self, msg):
		self.br = tf.TransfromBroadcaster()
		#create transform from map to agent
		self.br.sendTransform((msg.x, msg.y, msg.z),tf.transformations.quaternion_from_euler(0, 0, msg.theta),rospy.Time.now(),self.ns,"map")
		rospy.loginfo('here')


if __name__ == '__main__':
	try:
		# Create ROS Node
		rospy.init_node('Virtual_Sensor', anonymous=True)
		#run the sensor
		mySensor = Sensor()
		#keep ros from exiting
		rospy.spin()
	except rospy.ROSInterruptException:
		pass