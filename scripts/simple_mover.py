#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64


class LSimpleMover :

	def _getJointTopic( self, topic, jointIndx ) :

		return topic.replace( '###', str( jointIndx ) )

	def __init__( self, pNodeName, pNumJoints, pJointTopic ) :

		self.m_nodeName = pNodeName
		self.m_numJoints = pNumJoints
		self.m_pubs = []

		for i in range( self.m_numJoints ) :
			_pub = rospy.Publisher( self._getJointTopic( pJointTopic, i + 1 ), Float64, queue_size = 10 )
			self.m_pubs.append( _pub )

		self.m_startTime = 0
		self.m_rate = None

	def run( self ) :

		rospy.init_node( self.m_nodeName )
		self.m_rate = rospy.Rate( 10 )

		while not self.m_startTime :
			self.m_startTime = rospy.Time.now().to_sec()

		print 'running!'

		while not rospy.is_shutdown() :

			_elapsed = rospy.Time.now().to_sec() - self.m_startTime

			for i in range( self.m_numJoints ) :
				self.m_pubs[i].publish( math.sin( 2 * math.pi * 0.1 * _elapsed ) * ( math.pi / 2 ) )

			self.m_rate.sleep()



if __name__ == '__main__' :

	try :

		_smover = LSimpleMover( 'arm_mover', 2, '/simple_arm/joint_###_position_controller/command' )
		_smover.run()

	except rospy.ROSInterruptException :

		pass
