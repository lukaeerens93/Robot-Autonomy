
import numpy
from RRTTree import RRTTree
import copy


class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.tree = []
        self.path = []
        self.result = []
        self.arrived = False

    def Plan(self, start_config, goal_config, epsilon = 0.001):

        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)
        plan.append(goal_config)
        temp_plan = []
        temp_start_config = copy.deepcopy(start_config)
        temp_plan.append(temp_start_config)
        back_trace = []
        back_trace.append(0)
        while 1:
            if self.planning_env.Extend(temp_start_config, goal_config) is not None:
                break

            end_config = self.planning_env.GenerateRandomConfiguration()
            neighbor = []
            if len(temp_plan) > 1:
                for i in range(len(temp_plan)):
                    neighbor.append(self.planning_env.ComputeDistance(temp_plan[i], end_config))
                z = neighbor.index(min(neighbor))
                temp_start_config = copy.deepcopy(temp_plan[z])
                #back_trace.append(z)
                del neighbor
            if self.planning_env.Extend(temp_start_config, end_config) is not None:
                if len(temp_plan) > 1:
                    back_trace.append(z)
                temp_plan.append(end_config)
                temp_start_config = copy.deepcopy(end_config)


        y = len(temp_plan)-1
        while y != 0 :
            plan.insert(1,temp_plan[y])
            y = copy.deepcopy(back_trace[y-1])

        dist = 0
        for i in range(len(plan)-2):
            dist = dist + self.planning_env.ComputeDistance(plan[i],plan[i+1])
        print dist

        return plan
