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

class LSimpleMoverUI( QWidget ) :

    JOINT_1_CMD_TOPIC = "/simple_arm/joint_1_position_controller/command"
    JOINT_2_CMD_TOPIC = "/simple_arm/joint_2_position_controller/command"


    def __init__( self ) :

        super( LSimpleMoverUI, self ).__init__()

        # initialize ros related variables to be used
        self.m_jointsReferences = [ 0.0, 0.0 ]

        self.m_publishers = []
        self.m_publishers.append( rospy.Publisher( "/simple_arm/joint_1_position_controller/command", Float64, queue_size = 10 ) )
        self.m_publishers.append( rospy.Publisher( "/simple_arm/joint_2_position_controller/command", Float64, queue_size = 10 ) )

        # Create threading resources to handle ui-rospy interaction
        self.m_rate = rospy.Rate( 10 )
        self.m_lock = threading.Lock()
        
        self.m_workerRos = threading.Thread( target = self.workerFcn )

        # Make UI

        _ui_vbox = QVBoxLayout()

        self.m_sld_joint_1 = QSlider( Qt.Horizontal, self )
        self.m_sld_joint_1.valueChanged.connect( self.onJoint1Changed )
        self.m_sld_joint_1.setMinimum( 0 )
        self.m_sld_joint_1.setMaximum( 180 )

        self.m_sld_joint_2 = QSlider( Qt.Horizontal, self )
        self.m_sld_joint_2.valueChanged.connect( self.onJoint2Changed )
        self.m_sld_joint_2.setMinimum( 0 )
        self.m_sld_joint_2.setMaximum( 180 )

        _ui_vbox.addWidget( QLabel( "joint 1" ) )
        _ui_vbox.addWidget( self.m_sld_joint_1 )
        _ui_vbox.addWidget( QLabel( "joint 2" ) )
        _ui_vbox.addWidget( self.m_sld_joint_2 )

        self.setLayout( _ui_vbox )

        # Create timer and launch

        self.m_timer = QTimer()
        self.m_timer.setInterval( 100 )
        self.m_timer.timeout.connect( self.onTimerTick )
        
        # clear time and start everything
        self.m_t = 0.0
        self.m_timer.start()
        self.m_workerRos.start()

    def setJointReference( self, indx, value ) :
        self.m_lock.acquire( True )
        self.m_jointsReferences[ indx ] = value
        self.m_lock.release()

    def startMotion( self ) :
        print 'Motion started'
        self.m_timer.start()
        self.m_t = 0.0

    def stopMotion( self ) :
        print 'Motion stopped'
        self.m_timer.stop()

    def onJoint1Changed( self ) :
        self.setJointReference( 0, math.pi * ( self.m_sld_joint_1.value() / 180.0 ) )

    def onJoint2Changed( self ) :
        self.setJointReference( 1, math.pi * ( self.m_sld_joint_2.value() / 180.0 ) )

    def onSelectMotion( self, cbIndx ) :

        pass

    def onTimerTick( self ) :
        # update time and motion
        self.m_t += 0.1

    def workerFcn( self ) :
        print 'running rospy thread!'

        while not rospy.is_shutdown() :

            self.m_lock.acquire( True )

            for i in range( 2 ) :
                self.m_publishers[ i ].publish( self.m_jointsReferences[ i ] )

            self.m_lock.release()

            self.m_rate.sleep()

    def closeEvent( self, event ) :
        # # maybe clean stuff
        # self.m_workerRos.stop()
        # self.m_timer.stop()

        event.accept()

if __name__ == "__main__" :

    rospy.init_node( "simple_mover_ui" )

    _app = QApplication( sys.argv )

    _ui = LSimpleMoverUI()
    _ui.show()

    sys.exit( _app.exec_() )