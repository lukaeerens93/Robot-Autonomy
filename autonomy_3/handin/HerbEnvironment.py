import numpy as np
import copy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 0.7],
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = np.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

    def Crashed(self, end_config):
        self.config = [0] * len(self.robot.GetActiveDOFIndices())
        # print("Herb environment")
        # TODO: Generate and return a random configuration
        #lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        #self.config = numpy.random.uniform(lower_limits, upper_limits)

        self.robot.SetActiveDOFValues(end_config)

        if (end_config is None or self.robot.GetEnv().CheckCollision(
                self.robot) or self.robot.CheckSelfCollision()):
            print("Collision")
            return True
        else:
            return False


    def GetSuccessors(self, node_id):
        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        grid_coord = self.discrete_env.NodeIdToGridCoord2(node_id)
        node_id_list = []
        for i in [-1, 1]:
            for j in range(self.discrete_env.dimension):
                print("center is {}".format(grid_coord))
                center_coord = copy.deepcopy(grid_coord)
                center_coord[j]= center_coord[j] + i
                near_grid_coord = center_coord
                near_grid_config = self.discrete_env.GridCoordToConfiguration(near_grid_coord)
                lower_coords = np.zeros(self.discrete_env.dimension).tolist()
                dimension = self.discrete_env.dimension
                if all(near_grid_coord[i] >= lower_coords[i] for i in range(0,dimension)):
                    if all(near_grid_coord[i] <= self.discrete_env.num_cells[i] for i in range(0,dimension)):
                        if any(n < 0 for n in near_grid_coord.tolist()):
                            print("near coord is {}".format(near_grid_coord))
                        if not self.Crashed(near_grid_config):
                            near_node_id = self.discrete_env.GridCoordToNodeId2(near_grid_coord)
                            node_id_list.append(near_node_id)

        successors = node_id_list
        return successors

    def ComputeDistance(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        dist = numpy.linalg.norm(end_config - start_config)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        dx = start_config[0, 3] - end_config[0, 3]
        dy = start_config[1, 3] - end_config[1, 3]
        dist = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        return cost

    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #

        start_time = time.time()
        while (True):

            # Randomly select 2 points from the list of points on the shortest path
            len_path = len(path)-1
            q1_, q2_ = np.random.choice(len_path, 2, replace = False)
            print("q1_:{}, q2:{}".format(q1_, q2_))
            q1, q2 = path[q1_], path[q2_]

            '''
            We now need to see if we are able to connect the points together and create a
            new sample point between the 2. Let us call this sample point qi. The formula
            for qi is given by:
            qi = (q2 - q1)*i + q1 where i is a float that lies in the range [0,1] 
            '''
            # Default condition is connect = 2,
            # If points can't be connected for shortening, connect = 0. If points can be connected for shortening, connect = 1
            connect = 2
            for i in np.linspace(0.0, 1.0, 3000):
                qi = (q2 - q1) * i + q1
                self.robot.SetActiveDOFValues(qi)
                # If a collision is imminent, then you must not consider shortening from these 2 points
                if (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(self.robot)):
                    connect = 0
                else:
                    # If no collision is imminent, go for it
                    connect = 1
            if (connect == 1):
                rng = np.arange(q1_ + 1, q2_)
                # for j in range(q1_ + 1, q2_, 1):
                #     print("q1_+1:{}, q2_:{}, size of path:{}".format(q1_+1, q2_, path.shape))
                path = np.delete(path, rng, 0)
                print("size of path".format(path.size))
            current_time = time.time()
            if (current_time - start_time >= timeout):
                print("exceed timeout")
                break
        print("shorten path: All step dist: {},and vertices number {} ".format(np.sum(self.step*len(path)), len(path)))
        return path