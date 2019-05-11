import numpy, operator
import time
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.tree1 = []
        self.tree2 = []
        self.path1 = []
        self.path2 = []
        self.result1 = []
        self.result2 = []


    def Plan(self, start_config, goal_config, epsilon = 0.5):

        self.tree1 = RRTTree(self.planning_env, start_config)
        self.tree2 = RRTTree(self.planning_env, goal_config)
        plan = []
        start_time = time.time()

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # plan.append(start_config)
        # plan.append(goal_config)

        # simple:
        # goal_tf = numpy.eye(4,dtype = float)
        # goal_tf[0,3] = goal_config[0]
        # goal_tf[1,3] = goal_config[1]
        #
        # start_tf = numpy.eye(4,dtype = float)
        # start_tf[0,3] = start_config[0]
        # start_tf[1,3] = start_config[1]

        self.tree1.vertices = []
        #simple:
        # self.tree1.AddVertex(start_tf)
        self.tree1.AddVertex(start_config)

        self.tree2.vertices = []
        # simple:
        # self.tree2.AddVertex(goal_tf)
        self.tree2.AddVertex(goal_config)

        count = 0

        while (True):
            count += 1

            # Generate random point
            qr = self.planning_env.GenerateRandomConfiguration()

            # Find the nearest point to the random point
            # qi: nearest point
            #simple:
            # if(numpy.shape(qr) != (4, 4)):
            if (numpy.shape(qr) != (1, 7)):

                qr = self.planning_env.GenerateRandomConfiguration()
                qi_id1, qi1 = self.tree1.GetNearestVertex(qr)
                qi_id2, qi2 = self.tree2.GetNearestVertex(qr)
            else:
                qi_id1, qi1 = self.tree1.GetNearestVertex(qr)
                qi_id2, qi2 = self.tree2.GetNearestVertex(qr)

            qc1 = self.planning_env.Extend(qi1, qr)
            qc2 = self.planning_env.Extend(qi2, qr)

            qc_id1 = self.tree1.AddVertex(qc1)
            qc_id2 = self.tree2.AddVertex(qc2)

            self.tree1.AddEdge(qi_id1, qc_id1)
            self.tree2.AddEdge(qi_id2, qc_id2)
            qc1_qc2_dist = self.planning_env.ComputeDistance(qc1, qc2)
            # when reach the goal, break the while loop
            if(qc1_qc2_dist < epsilon):
                print("Reach the goal is qc_1 {} and qc_2 {} with dist {}".format(qc_id1, qc_id2 , qc1_qc2_dist))
                break

        path1 = []
        plan1 = []
        level1 = 0
        visited1 = [False] * len(self.tree1.vertices)

        path2 = []
        plan2 = []
        level2 = 0
        visited2 = [False] * len(self.tree2.vertices)

        # use DFS to find the path from the start_config to the goal_config
        # print("Print all paths1 is {} ".format(self.tree1.vertices))
        # print("Print all paths2 is {} ".format(self.tree2.vertices))

        self.printAllPathsUtil1(0, qc_id1,visited1,path1,level1)

        self.printAllPathsUtil2(0, qc_id2, visited2, path2, level2)

        # Hacky way to truncate the extra path return by the DFS
        idx1 = 0
        while(self.path1[idx1] is not qc_id1):
            self.result1.append(self.path1[idx1])
            idx1 += 1
        self.result1.append(qc_id1)

        # Look up the vertices given the vertices' id
        for i in self.result1:
            v = self.tree1.vertices[i]
            #simple
            # plan1.append((v[0,3],v[1,3]))
            plan1.append(v)


        idx2 = 0
        while(self.path2[idx2] is not qc_id2):
            self.result2.append(self.path2[idx2])
            idx2 += 1
        self.result2.append(qc_id2)

        # Look up the vertices given the vertices' id
        for i in self.result2:
            v = self.tree2.vertices[i]
            #simple:
            # plan2.append((v[0,3],v[1,3]))
            plan2.append(v)

        # given two path from two trees, plan1 and plan2, concat plan1 with plan2 (reverse) to give the final path
        reversed_plan2 = plan2[::-1]
        all_plan = numpy.concatenate((plan1,reversed_plan2),0)
        step_dist = self.planning_env.step
        all_step_dist = numpy.sum(len(all_plan)*step_dist)

        elapsed_time = time.time() - start_time
        vertices = len(numpy.concatenate((self.tree1.vertices,self.tree2.vertices),0))
        print("RRT CONNECT: All step dist: {}, elapsed time is {} and vertices number {} ".format(all_step_dist, elapsed_time, vertices))

        return all_plan

    def printAllPathsUtil1(self, u, d, visited, path, level):
        level += 1
        visited[u] = True
        path.append(u)
        if u == d:
            print("---------------------------------------------")
            print("This is path1 {}".format(path))
            # self.result.append(path)
            self.path1 = path
        else:
            for i in self.tree1.edges[u]:
                if visited[i] == False:
                    print("level %d, val %d " % (level, i))
                    self.printAllPathsUtil1(i, d, visited, path, level)

    def printAllPathsUtil2(self, u, d, visited, path, level):
        level += 1
        visited[u] = True
        path.append(u)
        if u == d:
            print("---------------------------------------------")
            print("This is path2 {}".format(path))
            # self.result.append(path)
            self.path2 = path
        else:
            for i in self.tree2.edges[u]:
                if visited[i] == False:
                    print("level %d, val %d " % (level, i))
                    self.printAllPathsUtil2(i, d, visited, path, level)
