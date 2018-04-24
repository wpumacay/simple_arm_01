#!/usr/bin/env python

import os
import sys
import threading
import math

import rospy
import rospkg

# binding provider is PyQt5; use its own specific API
# Check here for a good tutorial : http://zetcode.com/gui/pyqt5/
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

# messages definitions
from std_msgs.msg import Float64
from simple_arm.srv import *

class LSafeMoverUI( QWidget ) :

    def __init__( self ) :

        super( LSafeMoverUI, self ).__init__()

        # initialize ros related variables to be used
        self.m_jointsReferences = [ 0.0, 0.0 ]
        self.m_jointsReferencesBuff = [ 0.0, 0.0 ]

        self.m_proxySafeMoveService = rospy.ServiceProxy( '/arm_mover/safe_move',
                                                          GoToPosition )

        # Create threading resources to handle ui-rospy interaction
        self.m_hasSentMotionRequest = False
        self.m_workerRosService = None

        # Make UI

        self.setWindowTitle( "safe_mover_ui" )

        _ui_vbox = QVBoxLayout()

        self.m_sld_joint_1 = QSlider( Qt.Horizontal, self )
        self.m_sld_joint_1.valueChanged.connect( self.onJoint1Changed )
        self.m_sld_joint_1.setMinimum( 0 )
        self.m_sld_joint_1.setMaximum( 180 )

        self.m_sld_joint_2 = QSlider( Qt.Horizontal, self )
        self.m_sld_joint_2.valueChanged.connect( self.onJoint2Changed )
        self.m_sld_joint_2.setMinimum( 0 )
        self.m_sld_joint_2.setMaximum( 180 )

        self.m_btn_move_request = QPushButton( "move", self )
        self.m_btn_move_request.clicked.connect( self.onRequestMotion )

        _ui_vbox.addWidget( QLabel( "joint 1" ) )
        _ui_vbox.addWidget( self.m_sld_joint_1 )
        _ui_vbox.addWidget( QLabel( "joint 2" ) )
        _ui_vbox.addWidget( self.m_sld_joint_2 )
        _ui_vbox.addWidget( self.m_btn_move_request )

        self.setLayout( _ui_vbox )

    def setJointReference( self, indx, value ) :
        self.m_jointsReferences[ indx ] = value

    def onJoint1Changed( self ) :
        self.setJointReference( 0, math.pi * ( self.m_sld_joint_1.value() / 180.0 ) )

    def onJoint2Changed( self ) :
        self.setJointReference( 1, math.pi * ( self.m_sld_joint_2.value() / 180.0 ) )

    def onRequestMotion( self, cbIndx ) :

        if self.m_hasSentMotionRequest :
            QMessageBox.warning( self, "Request in progress...",
                                 "The arm is already handling a request",
                                 QMessageBox.Ok, 
                                 QMessageBox.NoButton )
            return

        # Copy data to send
        self.m_jointsReferencesBuff[ 0 ] = self.m_jointsReferences[ 0 ]
        self.m_jointsReferencesBuff[ 1 ] = self.m_jointsReferences[ 1 ]

        # Send worker for request
        self.m_workerRosService = threading.Thread( target = self.workerFcn )
        self.m_workerRosService.start()

    def workerFcn( self ) :
        print 'sending motion request!'

        # Set "in request" flag
        self.m_hasSentMotionRequest = True

        rospy.wait_for_service( '/arm_mover/safe_move' )

        # Make request
        _reqMsg = GoToPositionRequest()
        _reqMsg.joint_1 = self.m_jointsReferencesBuff[ 0 ]
        _reqMsg.joint_2 = self.m_jointsReferencesBuff[ 1 ]
        # Send request through proxy
        _response = self.m_proxySafeMoveService( _reqMsg )

        if _response == -1 :
            print 'Something went wrong, it seems the arm is still moving'
        
        print 'Requested motion took: ', _response, ' seconds'

        self.m_hasSentMotionRequest = False

        print 'finished motion request'

    def closeEvent( self, event ) :
        # # maybe clean stuff
        # self.m_workerRosService.stop()
        # self.m_timer.stop()

        event.accept()

if __name__ == "__main__" :

    rospy.init_node( "safe_mover_ui" )

    _app = QApplication( sys.argv )

    _ui = LSafeMoverUI()
    _ui.show()

    sys.exit( _app.exec_() )