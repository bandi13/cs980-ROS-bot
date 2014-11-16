#!/usr/bin/env python

from brain.srv import *
import rospy

def handle_grip_loc():
    print "returning grip location"
    return GripLocResponse([-1,-1,-1])

def handle_move(piece, dest):
    print "moving piece from " + str(piece) + " to " + str(goal)
    return 0

def arm_server():
    rospy.init_node('arm_server')
    rospy.Service('get_grip_loc', GripLoc, handle_grip_loc)
    rospy.Service('move_piece', Move, handle_move)
    print "ready for business"
    rospy.spin()

if __name__ == "__main__":
    arm_server()
