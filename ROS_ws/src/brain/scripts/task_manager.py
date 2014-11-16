#!/usr/bin/env python

import sys
import os
import planner
import rospy
from brain.srv import *

def set_the_table(pAlg):

    rospy.wait_for_service('table_setup')
    try:
        table_setup = rospy.ServiceProxy('table_setup', TableSetup)
        resp = table_setup()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

    rospy.wait_for_service('get_grip_loc')
    try:
        get_grip_loc = rospy.ServiceProxy('get_grip_loc', GripLoc)
        resp2 = get_grip_loc()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

    grip = []
    goal = []
    piece = []
    centers = []

    grip.append(planner.Loc(
        resp2.grip_loc[0],
        resp2.grip_loc[1],
        resp2.grip_loc[2]
    )

    for i in range(0, resp.size):
	goal.append(planner.Loc(
            resp.goals[i],
            resp.goals[i+1],
            resp.goals[i+2]
        ))
        piece.append(planner.Loc(
            resp.pieces[i],
            resp.pieces[i+1],
            resp.pieces[i+2]
        ))
        centers.append(planner.Loc(
            resp.piece_cemters[i],
            resp.piece_centers[i+1],
            resp.piece_centers[i+2]
        ))

    for i in range(0, len(piece)):
        print(piece[i].to_str())

    plnr = planner.Planner(pAlg)
    taskPlan = plnr.plan(grip, goal, piece, centers)

    if taskPlan == []:
        print('no plan found')

    for step in taskPlan:
        rospy.wait_for_service('move_piece')
        try:
            move_piece = rospy.ServiceProxy('move_piece', Move)
            res = move_piece(
                step.get_piece().raw(),
                step.get_dest().raw()
            )

            if res == -1:
                print('move failed')
                return False
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        print(step.to_str())

    for i in range(0, len(piece)):
        print(piece[i].to_str())

    return True

def usage():
    return "%s alg\n algorithm can be any of the following: nn"%sys.argv[0]

if __name__ = "__main__":
    if len(sys.argv) == 2:
        a = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
