#!/usr/bin/env python
import rospy
#for random number generation
import numpy as np
#transform global to local coords
import tf_conversions
import tf2_ros
import geometry_msgs.msg
#message from gazebo/model_states
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class Sensor():
	def __init__(self):
		# get namespace
		self.ns = rospy.get_namespace()
		self.sensor()
		self.trans = None

	def noise(self,data):
		#add noise, each pose has 7 data points * number of objects in gazebo
		noise = np.random.normal(0,0.05,12*len(data.name))
		#get rid of the ground plane
		indexer = data.name.index('ground_plane')
		del data.name[indexer]
		del data.pose[indexer]
		del data.twist[indexer]
		agent = data.name.index(self.ns[1:-1])
		#multiply each state by a noise value
		if self.trans:	
			for i in range(len(data.name)):
				#check if distance is too far

				#if i != agent:
				#add transformation if not agent
				(data.pose[i]).position.x += self.trans.transform.translation.x 
				(data.pose[i]).position.y += self.trans.transform.translation.y
				(data.pose[i]).position.z += self.trans.transform.translation.z
				(data.pose[i]).orientation.x += self.trans.transform.rotation.x
				(data.pose[i]).orientation.y += self.trans.transform.rotation.y
				(data.pose[i]).orientation.z += self.trans.transform.rotation.z
				(data.pose[i]).orientation.w += self.trans.transform.rotation.w		
				rospy.loginfo("{}: {}:\n{}".format(self.ns,data.name[i],data.pose[i]))			
				(data.pose[i]).position.x += noise[i]
				(data.pose[i]).position.y += noise[i+1]
				(data.pose[i]).position.z += noise[i+2]
				(data.pose[i]).orientation.x += noise[i+3]
				(data.pose[i]).orientation.y += noise[i+4]
				(data.pose[i]).orientation.z += noise[i+5]
				(data.pose[i]).orientation.w += noise[i+6]
				#modify geometry/Twist message type
				(data.twist[i]).linear.x += noise[i+7]
				(data.twist[i]).linear.y += noise[i+8]
				(data.twist[i]).linear.z += noise[i+9]
				(data.twist[i]).angular.x += noise[i+10]
				(data.twist[i]).angular.y += noise[i+11]
				(data.twist[i]).angular.z += noise[i+12]
		#RVIZ publish
		self.publish_particles(data)
		#publish the pose data
		self.pub.publish(data)

		

	def sensor(self):
		#create a publisher to push the adjusted sensor data to
		pubstr = self.ns+'virtual_sensor'
		self.pub = rospy.Publisher(pubstr, ModelStates, queue_size=1)
		#subscribe to odom in order to create transform
		# odom_node = self.ns+'odometry/local_filtered'
		# self.tf2_sub = rospy.Subscriber(odom_node,Odometry,self.transform_handle)
		#get transformation to local coordinates
		tfbuffer = tf2_ros.Buffer()
		tf2_listener = tf2_ros.TransformListener(tfbuffer)
		#target node
		tg_node = self.ns[1:-1]+'/'+'base_link'
		#subscribe to the gazebo model_states (this is the truth data)
		self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.noise)
		#create publisher for RVIZ
		pubstr_RVIZ = self.ns+'virtual_sensor_RVIZ'
		pubstr_RVIZ2 = self.ns+'virtual_sensor_RVIZ2'
		self.particle_pub = rospy.Publisher(pubstr_RVIZ, PoseArray, queue_size=1)
		self.marker_pub = rospy.Publisher(pubstr_RVIZ2, MarkerArray, queue_size=1)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				self.trans = tfbuffer.lookup_transform(tg_node,'map',rospy.Time())
				rospy.loginfo(self.trans)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				continue
			rate.sleep()



	# def transform_handle(self, msg):
	# 	self.br = tf2_ros.TransformBroadcaster()
	# 	t = geometry_msgs.msg.TransformStamped()
	# 	t.header.stamp = rospy.Time.now()
	# 	t.header.frame_id = 'map'
	# 	t.child_frame_id = self.ns
	# 	t.transform.translation.x = msg.pose.pose.position.x
	# 	t.transform.translation.y = msg.pose.pose.position.y
	# 	t.transform.translation.z = msg.pose.pose.position.z
	# 	t.transform.rotation.x = msg.pose.pose.orientation.x
	# 	t.transform.rotation.y = msg.pose.pose.orientation.y
	# 	t.transform.rotation.z = msg.pose.pose.orientation.z
	# 	t.transform.rotation.w = msg.pose.pose.orientation.w
	# 	#create transform from map to agent
	# 	self.br.sendTransform(t)


	def publish_particles(self, data):
			particles_conv = []
			for p in data.pose:
				particles_conv.append(p)
			# actually send the message so that we can view it in rviz
			self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(), frame_id=self.ns+'base_link'), poses=particles_conv))

			marker_array = []
			for index, i in enumerate(data.pose):
				marker = Marker(header=Header(stamp=rospy.Time.now(),
											  frame_id=self.ns+'base_link'),
									  pose=i,
									  type=0,
									  scale=Vector3(x=i.orientation.w*2,y=i.orientation.w*1,z=i.orientation.w*5),
									  id=index)
				marker_array.append(marker)

			self.marker_pub.publish(MarkerArray(markers=marker_array)) 


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