#!/usr/bin/env python3
#
#   rrt.py
#
import rospy
import numpy as np
import random

N_MAX = 20
MAX_GROWTH_TRIES = 10
P_GOAL = 0.1

ROBOT_SIZE = 0.2
CHECKS_PER_M = 20
STEP_SIZE = 0.5

TARGET_PATH_LEN = 32


######################################################################
#
#   State Definition
#
class State:
    def __init__(self, x, y, map):
        # Remember the (x,y) position.
        self.x = x
        self.y = y
        self.map = map

    ############################################################
    # Utilities:
    # In case we want to print the state.
    def __repr__(self):
        return "<Point %2d,%2d>" % (self.x, self.y)

    ############################################################
    # RRT Functions:
    # Compute the relative distance to another state.
    def DistSquared(self, other):
        return (self.x - other.x) ** 2 + (self.y - other.y) ** 2

    # Compute/create an intermediate state.
    def Intermediate(self, other, alpha):
        return State(
            self.x + alpha * (other.x - self.x),
            self.y + alpha * (other.y - self.y),
            self.map,
        )

    def InFreespace(self):
        return (
            self.map.distancesToNearestWall(np.array([[self.x, self.y]])) > ROBOT_SIZE
        )

    # Check the local planner - whether this connects to another state.
    def ConnectsTo(self, other):
        alphas = np.linspace(
            0, 1, max(3, int(np.sqrt(self.DistSquared(other) * CHECKS_PER_M)))
        )

        pts = np.hstack(
            (
                (self.x + alphas * (other.x - self.x)).reshape(-1, 1),
                (self.y + alphas * (other.y - self.y)).reshape(-1, 1),
            )
        )
        dists = self.map.distancesToNearestWall(pts)
        return np.min(dists) > ROBOT_SIZE


######################################################################
#
#   Tree Node Definition
#
#   Define a Node class upon which to build the tree.
#
class Node:
    def __init__(self, state, parentnode):
        # Save the state matching this node.
        self.state = state

        # Link to parent for the tree structure.
        self.parent = parentnode


######################################################################
#
#   RRT


class RRT:
    def __init__(self, map) -> None:
        self.map = map

    def buildRRT(self, start_pt, goal_pt, Nmax=N_MAX):
        self.start_state = State(start_pt[0], start_pt[1], self.map)
        self.goal_state = State(goal_pt[0], goal_pt[1], self.map)
        self.tree = [Node(self.start_state, None)]

        # Loop.
        while True:
            # Determine the target state.
            target_state = None

            ct = 0
            while target_state is None:
                ct += 1
                if ct >= MAX_GROWTH_TRIES:
                    return None
                to_goal = random.uniform(0, 1)
                if to_goal < P_GOAL:
                    target_state = State(self.goal_state.x, self.goal_state.y, self.map)
                else:
                    try_state = State(
                        random.normalvariate(goal_pt[0], 10),
                        random.normalvariate(goal_pt[1], 10),
                        self.map,
                    )
                    if try_state.InFreespace():
                        target_state = try_state
            # Find the nearest node (node with state nearest the target state).
            # This is inefficient (slow for large trees), but simple.
            list = [(node.state.DistSquared(target_state), node) for node in self.tree]
            (d2, nearnode) = min(list)
            d = np.sqrt(d2)
            near_state = nearnode.state
            next_state = near_state.Intermediate(
                target_state, STEP_SIZE / np.sqrt(target_state.DistSquared(near_state))
            )

            # Check whether to attach (creating a new node).
            if near_state.ConnectsTo(next_state):
                nextnode = Node(next_state, nearnode)
                self.tree.append(nextnode)

                if next_state.ConnectsTo(self.goal_state):
                    goalnode = Node(self.goal_state, nextnode)
                    self.tree.append(goalnode)
                    return goalnode

            # Check whether we should abort (tree has gotten too large).
            if len(self.tree) >= Nmax:
                rospy.logerr("Path not found to point: " + str(goal_pt))
                return None

    def removeNextExtra(self, path):
        for i in range(len(path) - 2):
            current = path[i]
            trial = path[i + 2]
            if current.ConnectsTo(trial):
                del path[i + 1]
                return True
        return False

    def removeAllExtras(self, path):
        while self.removeNextExtra(path):
            pass

    def addMidpoints(self, path):
        for i in range(len(path) - 2, -1, -1):
            new_state = path[i].Intermediate(path[i + 1], 0.5)
            path.insert(i + 1, new_state)

    #
    #  Post Process the Path
    #
    def pathPostProcess(self, path):
        while len(path) < TARGET_PATH_LEN:
            self.addMidpoints(path)
        self.removeAllExtras(path)

    def pathRRT(self, start_pt, goal_pt):
        goalnode = self.buildRRT(start_pt, goal_pt)
        if goalnode is None:
            return np.array([])
        else:
            path = []
            n = goalnode
            while n.parent is not None:
                path.append(n.state)
                n = n.parent

            path = path[::-1]
            self.pathPostProcess(path)

            path_pts = []
            for n in path:
                path_pts.append([n.x, n.y])

            return path_pts
