# Patrick Merrill
# CS980
#
# This file defines the planner class which implements a variety of high level
# task planners to generate a task plan for the table setting problem.
# In order to generate a plan each planner requires a gripper start location,
# a list of goal locations, and a list of piece locations. The specific goal
# for each piece is based on the order the locations are given, as in goal1 is
# for piece1.

#!usr/bin/env python

import math

class Loc:

    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

    def equals(self, loc):
        return loc.x == self.x and loc.y == self.y and loc.z == self.z

    def distance(self, loc):
        dx = loc.x - self.x
        dy = loc.y - self.y
        dz = loc.z - self.z

        sx = math.pow(dx, 2)
        sy = math.pow(dy, 2)
        sz = math.pow(dz, 2)

        return math.sqrt(sx + sy + sz)

    def update(self, loc):
        self.x = loc.x
        self.y = loc.y
        self.z = loc.z

    def raw(self):
        return [self.x, self.y, self.z]

    def to_str(self):
        return "(" + str(self.x) + "," + str(self.y) + "," + str(self.z) + ")" 

class Move:

    def __init__(self, piece, post_loc, dest):
        self.p = piece
        self.p_loc
        self.dest = dest

    def get_piece(self):
        return self.p_loc

    def get_dest(self):
        return self.dest

    def to_str(self):
        return "Move piece " + str(self.p) + " from " + self.post_loc.to_str() + " to " + self.dest.to_str() + "\n"

class Planner:

    def __init__(self, alg):
        self.kind = "nn"

    def plan(self, grip, goals, posts, pieces, radius = 2.0):
        self.grip = list(grip)
        self.goals = list(goals)
        self.pieces = list(pieces)
        self.posts = list(posts)
        self.num = len(pieces)
        self.plan = []
        self.p_radius = radius

        if self.kind == "nn":
            return self.nearest_neighbors()
        else:
            print('unknown planning algorithm')
            return []

    def goal_reached(self):
        for i in range(0, self.num):
            if not self.pieces[i].equals(self.goals[i]):
                return False

        return True

    def on_goal(self, i):
        return self.pieces[i].equals(self.goals[i])

    def goal_covered(self, g):
        for i in range(0, self.num):
            if (self.goals[g].equals(self.pieces[i]) or
                    (self.goals[g].distance(self.pieces[i]) < p_radius) and
                    self.pieces[i].z == 0):
                if g == i:
                    continue
                return True

        return False

    def all_covered(self):
        for i in range(0, self.num):
            if not self.on_goal(i) and not self.goal_covered(i):
                return False

        return True

    def find_valid_loc(self, p, p2):
        validLoc = False
        pct = 0
        loc = Loc(0,0)
        while not validLoc:
            dx = self.p_radius
            if self.pieces[p2].x == self.pieces[p].x:
                newX = self.pieces[p2].x
            elif self.pieces[p2].x < self.pieces[p].x:
                if pct != 0:
                    dx = (self.pieces[p].x - self.pieces[p2].x) * pct
                newX = self.pieces[p2].x + dx
            else:
                if pct != 0:
                    dx = (self.pieces[p2].x - self.pieces[p].x) * pct
                newX = self.pieces[p2].x - dx

            dy = self.p_radius
            if self.pieces[p2].y == self.pieces[p].y:
                newY = self.pieces[p2].y
            elif self.pieces[p2].y < self.pieces[p].y:
                if pct != 0:
                    dy = (self.pieces[p].y - self.pieces[p2].y) * pct
                newY = self.pieces[p2].y + dy
            else:
                if pct != 0:
                    dy = (self.pieces[p2].y - self.pieces[p].y) * pct
                newY = self.pieces[p2].y - dy

            loc = Loc(newX, newY)
            post_loc = Loc(
                    newX - (self.pieces[p].x - self.posts[p].x),
                    newY - (self.pieces[p].y - self.posts[p].y)
                    )
            validLoc = True
            for i in range(0, self.num):
                if loc.distance(self.pieces[i]) < self.p_radius:
                    validLoc = False
                    
            pct = pct + .1

        return [loc, post_loc]

    def nearest_neighbors(self):
        # Builds a task plan using nearest nieghbors, where the closest
        # piece with an open goal is moved and it continues until all
        # pieces have reached their goals. If all goals are covered and
        # the goal state is not met the closest piece is moved to a
        # position closest to it's goal and the piece that belongs on
        # the goal it covered.

        while not self.goal_reached():
            p = -1
            dst = -1
            if self.all_covered():
                print('covered')

                # Find closest piece
                for i in range(0, self.num):
                    if self.on_goal(i):
                        continue

                    temp = self.pieces[i].distance(self.grip[0])
                    if temp < dst or dst == -1:
                        dst = temp
                        p = i

                # Find piece that belongs to the goal covered by the closest piece
                for i in range(0, self.num):
                    if (self.pieces[p].equals(self.goals[i]) or
                            self.pieces[p].distance(self.goals[i]) < p_radius):
                        p2 = i
                        break

                # Find location difference of the piece center to the piece post

                loc, post_loc = self.find_valid_loc(p, p2)

                self.plan.append(Move(p, post_loc, loc))
                self.grip[0].update(self.goals[p])
                self.pieces[p].update(loc)
                self.posts[p].update(post_loc)

            else:
                print('here')
                for i in range(0, self.num):
                    if self.on_goal(i):
                        print('on goal')
                        continue
                    # Handle if the piece is in the air
                    elif self.pieces[i].z != 0:
                        p2 = i
                        continue
                    elif self.goal_covered(i):
                        print('goal covered')
                        continue

                    print('there')
                    temp = self.pieces[i].distance(self.grip[0])
                    if temp < dst or dst == -1:
                        dst = temp
                        p = i

                # if the piece was in the air but its goal is covered
                # For now set it down and continue
                if self.goal_covered(i):
                    loc, post_loc = self.find_valid_loc(p2, p)

                    self.plan.append(Move(p2, post_loc, loc))
                    self.grip[0].update(loc)
                    self.pieces[p2].update(loc)
                    self.posts[p2].update(post_loc)
                    continue

                post_loc = Loc(
                    self.goals[p].x - (self.pieces[p].x - self.posts[p].x),
                    self.goals[p].y - (self.pieces[p].y - self.posts[p].y)
                    )

                self.plan.append(Move(p, post_loc, self.goals[p]))
                self.grip[0].update(self.goals[p])
                self.pieces[p].update(self.goals[p])
                self.posts[p].update(post_loc)

        print(str(self.plan))
        return self.plan
