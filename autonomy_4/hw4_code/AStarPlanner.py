import heapq
import datetime

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):
        plan = []
        if self.visualize:
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        start_time = datetime.datetime.now()
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        # heap: [(prev_path_dist_sum, prev_nodeid,cur_nodeid, goal_nodeid)]
        heap = [(self.getHeuristicCost(0, start_id, start_id, goal_id), start_id)]
        heapq.heapify(heap)
        graph_path = {start_id:None}
        graph_action = {start_id:None}
        hist = {start_id}
        num_nodes=1

        while heap:
            curr_total_cost, curr_id = heapq.heappop(heap)
            print(curr_id)
            hueristic_cost = self.planning_env.ComputeHeuristicCost(curr_id, goal_id)
            g = curr_total_cost - hueristic_cost
            if curr_id == goal_id:
                # print(NodeIdToConfiguration(goal_config))
                plan = self.backtrack(curr_id, graph_path, graph_action)
                print("Finish the plan")
                break

            for succ in self.planning_env.GetSuccessors(curr_id):
                succ_id = succ[0]
                action = succ[1]

                # if the successor has been visited, skip it
                if succ_id in hist:
                    continue

                if self.visualize:
                    curr_config = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
                    next_config = self.planning_env.discrete_env.NodeIdToConfiguration(succ_id)
                    self.planning_env.PlotEdge(curr_config, next_config)

                graph_path[succ_id] = curr_id
                graph_action[succ_id] = action
                heapq.heappush(heap, (self.getHeuristicCost(g, curr_id, succ_id, goal_id), succ_id))
                hist.add(succ_id)
                num_nodes+=1

        plan = plan[::-1]
        end_time=datetime.datetime.now()
        print(plan)
        print("Length of plan: ", len(plan))
        print("Astar Total Time:", end_time-start_time)
        print("Astar Node Number:", num_nodes)
        return plan

    def getHeuristicCost(self, prev_path_dist_sum, prev_nodeid,cur_nodeid, goal_nodeid):
        edge_dist = self.planning_env.ComputeDistance(prev_nodeid, cur_nodeid)
        heuristic_dist = self.planning_env.ComputeHeuristicCost(cur_nodeid, goal_nodeid)
        return prev_path_dist_sum + edge_dist + heuristic_dist


    def backtrack(slef, node_id, graph_path, graph_action):
        plan = []
        while node_id:
            if not graph_action[node_id]:
                break
            plan.append(graph_action[node_id])
            prev_id = graph_path[node_id]
            node_id = prev_id
        return plan