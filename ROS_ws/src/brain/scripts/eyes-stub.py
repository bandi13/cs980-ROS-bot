#!/usr/bin/env python

from brain.srv import *
import rospy

def handle_table_setup():
    size = 4
    goal = [0,0,0,1,0,0,0,1,0,1,1,0]
    piece = [1,0,0,0,1,0,1,1,0,0,0,0]
    centers = [1,0,0,0,1,0,1,1,0,0,0,0]
    return TableSetupResponse(size, goal, piece, centers)

def table_setup_server():
    rospy.init_node('table_setup')
    s = rospy.Service('table_setup', TableSetup, handle_table_setup)
    print "sending table setup"
    rospy.spin()

if __name__ == "__main__":
    table_setup_server()
