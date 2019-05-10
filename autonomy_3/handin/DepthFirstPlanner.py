import numpy as np

class DepthFirstPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
    def Crashed(self, end_config):

        # This function is added to check if collision
        # has occured between an obstacle and the robot

        # Create a 4x4 identity matrix that will be used as a transformation matrix.
        # Define the displacement transformation according to the end configuration
        newTransform = np.eye(4)

        # np_end_config = np.asarray(end_config)
        # np_end_config = np.append(np_end_config,[0,1])

        # newTransform[3:] = np_end_config
        newTransform[0,3] = end_config[0]
        newTransform[1,3] = end_config[1]


        # Set robot to position specified by end_config

        robot = self.planning_env.robot
        robot.SetTransform(newTransform)

        # Check for collisionsCrashed
        crashed = ((robot.GetEnv().CheckCollision(robot)) or robot.CheckSelfCollision())
        if crashed:
            print "crashed!!!!!!!!!!"
            return True
        else:
            return False
    def Plan(self, start_config, goal_config):

        plan = []

        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        queue = []
        visited = []

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        queue.append(start_id)
        # visited.append(start_id)

        plan.append(start_config)

        # while queue:
        #     search_id = queue.pop(0)
        #     if search_id not in visited:
        #         visited.append(search_id)
        #         grid_coord = self.planning_env.discrete_env.NodeIdToGridCoord(search_id)
        #         np_grid_coord = np.asarray(grid_coord)
        #         plan.append(np_grid_coord)
        #         queue.append(self.planning_env.GetSuccessors(search_id))
        denv = self.planning_env.discrete_env
        if self.visualize:
            self.planning_env.InitializePlot (start_config)
        queue.append(start_id)
        D = {}
        successors = []
        visited = []
        while queue:
            #use queue to store the nodes that needs to be checked
            current_node = queue.pop()
            # print "current_node:{}".format(current_node)
            # print "visited:{}".format(visited)
            # print "Dictionary:{}".format(D)
            #if the node hasn't been cheked, enter the while loop and check it
            if not (current_node in visited):
                visited.append(current_node)
                #get the successors of the current node
                successors = self.planning_env.GetSuccessors(current_node)
                # print "successors:{}".format(successors)
                #if there are successors, append it to the Dictionary
                while successors:
                    s = successors.pop(0)
                    # print "s:{}, current_node:{}".format(s,current_node)
                    #the format of dictionary: {child: parrent}
                    if not (s in visited):
                        # print "{} is not in visited".format(s)
                        s_coor = self.planning_env.discrete_env.NodeIdToGridCoord2(s)

                        if (not self.Crashed(self.planning_env.discrete_env.NodeIdToConfiguration(s))) and (not isinstance(s, (list,))):

                            D[s] = current_node
                        # if self.Crashed(self.planning_env.discrete_env.NodeIdToGridCoord(s)):
                            if self.visualize:
                                self.planning_env.PlotEdge(denv.NodeIdToConfiguration(current_node), denv.NodeIdToConfiguration(s))
                            #if discover the goal_id, breaak the whole loop
                            if s == goal_id:
                                queue = []
                                break
                            else:
                                #if it's not the goal_id, add the successor to queue
                                queue.append(s)
                                # print "queue:{}".format(queue)





        # import IPython
        # IPython.embed()
        #extract path
        check_node = goal_id
        plan_node = []
        # print "Dictionary:{}".format(D)
        # print "check_node:{}".format(check_node)
        # print "start_id:{}".format(start_id)
        # print "goal_id:{}".format(goal_id)
        # start_id:210
        # goal_id:216
        name = raw_input("Enter to continue ")
        while not(check_node is start_id):
            # print "checknode:{}, D[check_node]:{}".format(check_node, D[check_node])
            plan_node.append(D[check_node])
            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(D[check_node]))
            check_node = D[check_node]

        plan_node.reverse()
        plan.reverse()
        # print "plan:{}, plan_node:{}".format(plan, plan_node)
        return plan[:-1], plan_node
