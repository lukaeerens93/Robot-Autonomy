import heapq
import datetime
import numpy as np

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        print ("A Star Launched")


    def Plan(self, start_config, goal_config):
        start_time = datetime.datetime.now()
        def getCost(prev_g, prev_id, curr_id, goal_id):
            # Convert from node space to configuration space
            prev_config = self.planning_env.discrete_env.NodeIdToConfiguration(prev_id)
            curr_config = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
            # Transform into a 4x4 matrix
            prevNewTransform, currNewTransform = np.eye(4), np.eye(4)
            prevNewTransform[0, 2] = prev_config[0]
            prevNewTransform[1, 3] = prev_config[1]
            currNewTransform[0, 2] = curr_config[0]
            currNewTransform[1, 3] = curr_config[1]
            g = self.planning_env.ComputeDistance(prevNewTransform, currNewTransform)
            h = self.planning_env.ComputeHeuristicCost(prevNewTransform, currNewTransform)

            return prev_g + g + h

        def backtrack(node_id, graph_path):
            plan = []
            while node_id:
                plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(node_id))
                prev_id = graph_path[node_id]
                node_id = prev_id
            return plan
        plan = []

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        if self.visualize:
            self.planning_env.InitializePlot(goal_config)
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        heap = [(getCost(0, start_id, start_id, goal_id), start_id)]
        heapq.heapify(heap)
        graph_path = {start_id:None}
        hist = {start_id}
        num_nodes=1

        while heap:
            curr_cost, curr_id = heapq.heappop(heap)

            curr_config = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
            curr_1NewTransform, goal_1NewTransform = np.eye(4), np.eye(4)
            curr_1NewTransform[0, 2] = curr_config[0]
            curr_1NewTransform[1, 3] = curr_config[1]
            goal_1NewTransform[0, 2] = goal_config[0]
            goal_1NewTransform[1, 3] = goal_config[1]

            h = self.planning_env.ComputeHeuristicCost(curr_1NewTransform, goal_1NewTransform)
            g = curr_cost - h
            if curr_id == goal_id:
                plan = backtrack(curr_id, graph_path)
                break
            for succ_id in self.planning_env.GetSuccessors(curr_id):
                if not isinstance (succ_id, (list,)):
                    if succ_id in hist:
                        continue
                    if self.visualize:
                        curr_config = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
                        next_config = self.planning_env.discrete_env.NodeIdToConfiguration(succ_id)
                        self.planning_env.PlotEdge(curr_config, next_config)
                    graph_path[succ_id] = curr_id
                    heapq.heappush(heap, (getCost(g, curr_id, succ_id, goal_id), succ_id))
                    hist.add(succ_id)
                    num_nodes+=1

        plan = plan[::-1]
        end_time=datetime.datetime.now()
        print("Length of plan: ", len(plan))
        print("Plan time:", end_time-start_time)
        print("Number of nodes explored:", num_nodes)
        print(plan)
        return plan,plan
