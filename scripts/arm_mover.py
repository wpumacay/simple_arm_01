#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from simple_arm.srv import *



class LArmMover :

	INSTANCE = None

	@staticmethod
	def handle_safe_move_request( req ) :

		if LArmMover.INSTANCE.isMoving() :
			
			print 'The arm is currently executing a command, just wait a little.'
			rospy.logwarn( 'The arm is currently moving. Go back in a minute, please' )

			return GoToPositionResponse( -1 )

		else :
			
			rospy.loginfo( 'GoToPositionRequest received - j1:%s, j2:%s', req.joint_1, req.joint_2 )
			_clamped_joints = LArmMover.INSTANCE.clamp2limits( [req.joint_1, req.joint_2] )

			_time_elapsed = LArmMover.INSTANCE.moveArm( _clamped_joints )

			return GoToPositionResponse( _time_elapsed )

	def moveArm( self, pJointAngles ) :

		print 'started request'

		_time_start = rospy.Time.now()
		_time_end = 0
		_time_elapsed = 0

		for i in range( self.m_numJoints ) :
			self.m_pubs[i].publish( pJointAngles[i] )

		self.m_isMoving = True

		while True :

			_joints_state = rospy.wait_for_message( '/simple_arm/joint_states', JointState )

			if self.isAtGoal( _joints_state.position, pJointAngles ) :

				_time_end = rospy.Time.now()
				_time_elapsed = _time_end - _time_start
				break

		print 'finished request'

		self.m_isMoving = False

		return _time_elapsed

	def clamp2limits( self, reqJointAngles ) :

		_clamped_joints = [ t for t in reqJointAngles ]

		for i in range( self.m_numJoints ) :

			_min = self.m_minJoints[i]
			_max = self.m_maxJoints[i]

			if not _min <= reqJointAngles[i] <= _max :
				_clamped_joints[i] = min( max( _clamped_joints[i], _min ), _max )
				rospy.logwarn( 'joint %s is out of bounds, valid range: ( %s, %s ), requested: %s', i + 1, _min, _max, reqJointAngles[i] )

		return _clamped_joints

	def isMoving( self ) :

		return self.m_isMoving

	def isAtGoal( self, pCurrentAngles, pReqAngles ) :

		_TOLERANCE = 0.05

		_reachedGoal = True

		for i in range( self.m_numJoints ) :

			_delta = abs( pCurrentAngles[i] - pReqAngles[i] )

			if _delta > _TOLERANCE :
				_reachedGoal = False
				break

		return _reachedGoal

	def _getJointSpecificStr( self, jointBaseStr, jointIndx ) :

		return jointBaseStr.replace( '###', str( jointIndx ) )


	def __init__ ( self, pNodeName, pJointTopicBaseName, pJointMinBaseName, pJointMaxBaseName, pNumJoints, pServiceName ) :

		LArmMover.INSTANCE = self

		self.m_nodeName = pNodeName
		self.m_serviceName = pServiceName
		self.m_jointMinName = pJointMinBaseName
		self.m_jointMaxName = pJointMaxBaseName

		self.m_numJoints = pNumJoints
		self.m_pubs = []

		self.m_isMoving = False

		rospy.init_node( self.m_nodeName )

		for i in range( self.m_numJoints ) :
			_pub = rospy.Publisher( self._getJointSpecificStr( pJointTopicBaseName, i + 1 ), Float64, queue_size = 10 )
			self.m_pubs.append( _pub )

		self.m_service = rospy.Service( self.m_serviceName, GoToPosition, LArmMover.handle_safe_move_request )

		self.m_maxJoints = []
		self.m_minJoints = []

		for i in range( self.m_numJoints ) :

			_minJointAngle = rospy.get_param( self._getJointSpecificStr( pJointMinBaseName, i + 1 ), 0.0 )
			_maxJointAngle = rospy.get_param( self._getJointSpecificStr( pJointMaxBaseName, i + 1 ), 2 * math.pi )

			self.m_minJoints.append( _minJointAngle )
			self.m_maxJoints.append( _maxJointAngle )

	def run( self ) :

		rospy.spin()


if __name__ == '__main__' :

	_amover = LArmMover( 'arm_mover', 
						 '/simple_arm/joint_###_position_controller/command',
						 '~min_joint_###_angle',
						 '~max_joint_###_angle',
						 2, '~safe_move' )

	try :

		_amover.run()

	except rospy.ROSInterruptException :

		pass