#!/usr/bin/env python

import os
import sys

import rospy
import rospkg

# binding provider is PyQt5, using its own specific stuff
from python_qt_binding import QtWidgets, QtGui, QtCore

rospy.init_node( "test_qt_binding" )

app = QtWidgets.QApplication( sys.argv )

w = QtWidgets.QWidget()
w.resize( 250, 150 )
w.move( 300, 300 )
w.setWindowTitle( "test_qt_binding" )
w.show()

sys.exit( app.exec_() )

