#!/usr/bin/env python

import math

import rospy
import rospkg

from sensor_msgs.msg import Image, JointState
from simple_arm.srv import *



class LookAwayNode( object ) :


    def __init__( self ) :

        rospy.init_node( 'look_away' )

        # subscribe to the camera and joints topics
        self.m_subsJoints = rospy.Subscriber( '/simple_arm/joint_states',
                                              JointState,
                                              self.onJointMsgCallback )
        self.m_subsImage = rospy.Subscriber( 'rgb_camera/image_raw',
                                             Image,
                                             self.onImageMsgCallback )

        # create a proxy to request to the safe_move service
        self.m_proxyServiceSafeMove = rospy.ServiceProxy( '/arm_mover/safe_move',
                                                          GoToPosition )

        self.m_lastPos = None
        self.m_isArmMoving = False

        rospy.spin()

    # message callbacks **************************************************

    def onJointMsgCallback( self, jointMsg ):

        # Just check if the are is moving a little bit
        if self._arePosEqual( self.m_lastPos, jointMsg.position ) :
            self.m_isArmMoving = False

        else :
            self.m_lastPos = jointMsg.position
            self.m_isArmMoving = True

    def onImageMsgCallback( self, imgMsg ) :

        # check if the are is quietly resting and looking some place boring
        if not self.m_isArmMoving and self._isImageUniform( imgMsg.data ) :

            rospy.loginfo( 'Boring view, lets turn to some place nicer' )

            try :
                # send request to look some place nice
                rospy.wait_for_service( '/arm_mover/safe_move' )

                _msgReq = GoToPositionRequest()
                # turn by these angles to see the dice on the table
                _msgReq.joint_1 = 1.57
                _msgReq.joint_2 = 1.57

                _response = self.m_proxyServiceSafeMove( _msgReq )

                if _response == -1 :
                    rospy.logwarn( 'Weird, the service says the arm is already \
                                   moving, should wait until it stops' )
                    return

                rospy.loginfo( 'Camera got bored of the ceiling and \
                               turned to someplace nicer in %s ms',
                               _response )

            except rospy.ServiceException, e :
                # something went wrong when requesting the service
                rospy.logwarn( 'Something went wrong sending the service request, %s', e )
    
    # ********************************************************************        

    # helper functions ***************************************************

    def _isImageUniform( self, imgData ) :
        return all( value == imgData[ 0 ] for value in imgData )

    def _arePosEqual( self, pos1, pos2 ) :

        if pos1 is None or pos2 is None :
            return False

        _TOLERANCE = 0.0005
        _dx = abs( pos1[ 0 ] - pos2[ 0 ] )
        _dy = abs( pos1[ 1 ] - pos2[ 1 ] )

        return ( _dx <= _TOLERANCE ) and ( _dy <= _TOLERANCE )

    # ********************************************************************


if __name__ == '__main__' :

    try :
        LookAwayNode()

    except rospy.ROSInterruptException :
        pass