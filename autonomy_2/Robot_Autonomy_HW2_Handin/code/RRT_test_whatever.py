# import numpy as np
# from RRTTree import RRTTree
# import SimpleEnvironment
# # import HerbEnvironment
#
#
# class RRTPlanner(object):
#
#     def __init__(self, plan_env, visualize):
#         self.plan_env = plan_env
#         self.visualize = visualize
#
#
#     def Plan(self, start, goal, epsilon = 3):
#
#         # Define a list that will contain the plan for the RRT Planner
#         plan = []
#         # Define the tree, that will propage through the RRT
#         tree = RRTTree(self.planning_env, start)
#         if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
#             self.planning_env.InitializePlot(goal)
#
#
#         while (True):
#
#             '''
#             When it reaches the goal, we just break the while loop
#             '''
#
#             # Generate new point
#             new_point = self.planning_env.GenerateRandomConfiguration()
#
# 	    # Find the nearest point to this one (get id and distance of a node closest.
#             (point_ID, nearest_point) = tree.GetNearestVertex(new_point)
#
# 	    # see if the closet point to new point can connect
#             can_connect = self.planning_env.Extend(nearest_point, new_point)
#
#             '''
#
#             1) Check whether there is a cycle
#             if there is no cycle:
#                     2) We have to add a new point to the verteces
#             3) 0.2 (20%) to extend towards the goal
#                 0.8 (80%) to extend towards any random configuration
#             '''
#
#             '''
#             edges are used to grow the tree passed its original size, towards the goal
#             example format:
#             vertices = [1,2,3,4,5]
#             edges = {1:2, 2:(1,3,5), 3,(2,4)}
#             '''
#
#             # if so
#             if (can_connect != None):
#
#                 # Compute the distance
#                 d = self.planning_env.ComputeDistance(can_connect, goal)
#
#
