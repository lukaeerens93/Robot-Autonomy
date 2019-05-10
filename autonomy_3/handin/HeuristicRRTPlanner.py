# import numpy
# from RRTTree import RRTTree
#
# class HeuristicRRTPlanner(object):
#
#     def __init__(self, planning_env, visualize):
#         self.planning_env = planning_env
#         self.visualize = visualize
#
#
#     def Plan(self, start_config, goal_config, epsilon = 0.001):
#
#         tree = RRTTree(self.planning_env, start_config)
#         plan = []
#         if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
#             self.planning_env.InitializePlot(goal_config)
#         # TODO: Here you will implement the rrt planner
#         #  The return path should be an array
#         #  of dimension k x n where k is the number of waypoints
#         #  and n is the dimension of the robots configuration space
#
#         plan.append(start_config)
#         plan.append(goal_config)
#
#         return plan



import numpy
from RRTTree import RRTTree
import time


class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.tree = []
        self.path = []
        self.result = []
        self.arrived = False


    def Plan(self, start_config, goal_config, epsilon = 2.0):
        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        start_time = time.time()

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # simple:
        # goal_tf = numpy.eye(4,dtype = float)
        # goal_tf[0,3] = goal_config[0]
        # goal_tf[1,3] = goal_config[1]
        #
        # start_tf = numpy.eye(4,dtype = float)
        # start_tf[0,3] = start_config[0]
        # start_tf[1,3] = start_config[1]

        self.tree.vertices = []
        # simple:
        # self.tree.AddVertex(start_tf)
        self.tree.AddVertex(start_config)
        print("tree vertex:{}".format(start_config))
        count = 0

        while (True):
            count += 1

            # Generate random point
            if(count % 10 is not 0):
                qr = self.planning_env.GenerateRandomConfiguration()
            else:
                # simple:
                # qr = goal_tf
                qr = goal_config
            # Find the nearest point to the random point
            # qi: nearest point
            #simple
            # if(numpy.shape(qr) != (4, 4)):
            if (numpy.shape(qr) != (1, 7)):
                qr = self.planning_env.GenerateRandomConfiguration()

                qi_id, qi = self.tree.GetNearestVertex(qr)

            else:
                qi_id, qi = self.tree.GetNearestVertex(qr)

            qc = self.planning_env.Extend(qi,qr)

            qc_id = self.tree.AddVertex(qc)
            self.tree.AddEdge(qi_id, qc_id)

            # when reach the goal, break the while loop
            # if(self.planning_env.ComputeDistance(qc, goal_tf) < epsilon):
            print("compute distance:{}".format(self.planning_env.ComputeDistance(qc, goal_config)))
            if(self.planning_env.ComputeDistance(qc, goal_config) < epsilon):
                print("Reach the goal is {} ".format(qc_id))
                break

        path = []
        level = 0
        visited = [False] * len(self.tree.vertices)
        # use DFS to find the path from the start_config to the goal_config
        # print("Print all paths is {} ".format(self.tree.vertices))
        self.printAllPathsUtil(0, qc_id,visited,path,level)

        # Hacky way to truncate the extra path return by the DFS
        idx = 0
        while(self.path[idx] is not qc_id):
            self.result.append(self.path[idx])
            idx += 1
        self.result.append(qc_id)

        # Look up the vertices given the vertices' id
        for i in self.result:
            v = self.tree.vertices[i]
            #simple:
            # plan.append((v[0,3],v[1,3]))
            plan.append(v)

        step_dist = self.planning_env.step
        all_step_dist = numpy.sum(len(plan) * step_dist)

        elapsed_time = time.time() - start_time
        vertices = len(self.tree.vertices)
        print("RRT: All step dist: {}, elapsed time is {} and vertices number {} ".format(all_step_dist,
                                                                                                  elapsed_time,
                                                                                                  vertices))

        # return N*Vertice_dimension ndarray path
        return plan


    def printAllPathsUtil(self, u, d, visited, path,level):
        level += 1
        visited[u] = True
        path.append(u)
        if u == d:
            print("---------------------------------------------")
            print("This is path {}".format(path))
            # self.result.append(path)
            self.path = path
        else:
            for i in self.tree.edges[u]:
                if visited[i] == False:
                    print("level %d, val %d "%(level,i))
                    self.printAllPathsUtil(i,d,visited,path,level)