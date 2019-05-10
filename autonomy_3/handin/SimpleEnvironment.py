import numpy as np
import pylab as pl
import copy
from DiscreteEnvironment import DiscreteEnvironment


class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[0, 0, -1, 1.5],
                               [-1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 0, 1]])
        table.SetTransform(table_pose)
        # Create the offset, we will use this to calculate neighbors later.
        self.offset = np.array([[-1, 0], [0, -1], [1, 0], [0, 1]])
        self.config = [0] * 2
        self.p = 0.0
        self.step = 2



    def Crashed(self, end_config):
        # Create a 4x4 identity matrix that will be used as a transformation matrix.
        # Define the displacement transformation according to the end configuration
        newTransform = np.eye(4)
        newTransform[0,3] = end_config[0]
        newTransform[1,3] = end_config[1]
        robot = self.robot
        robot.SetTransform(newTransform)
        crashed = ((robot.GetEnv().CheckCollision(robot)) or robot.CheckSelfCollision())
        if crashed:
            # print "crashed!!!!!!!!!!"
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
                # print("center is {}".format(grid_coord))
                center_coord = copy.deepcopy(grid_coord)
                center_coord[j]= center_coord[j] + i
                near_grid_coord = center_coord
                near_grid_config = self.discrete_env.GridCoordToConfiguration(near_grid_coord)
                lower_coords = np.zeros(self.discrete_env.dimension).tolist()
                dimension = self.discrete_env.dimension
                if all(near_grid_coord[i] >= lower_coords[i] for i in range(0,dimension)):
                    if all(near_grid_coord[i] <= self.discrete_env.num_cells[i] for i in range(0,dimension)):
                        if any(n < 0 for n in near_grid_coord.tolist()):
                            print("near coord <0 : {}".format(near_grid_coord))
                        if self.Crashed(near_grid_config):
                            print("Crashing!!")
                        else:
                            near_node_id = self.discrete_env.GridCoordToNodeId2(near_grid_coord)
                            node_id_list.append(near_node_id)
        successors = node_id_list
        return successors

    # return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        if self.discrete_env.IdinRange(start_id) and self.discrete_env.IdinRange(end_id):
            # TODO: Here you will implement a function that
            # computes the distance between the configurations given
            # by the two node ids

            # Start coordinate
            start = self.discrete_env.NodeIdToConfiguration(start_id)
            # End coordinate
            end = self.discrete_env.NodeIdToConfiguration(end_id)

            # Calculate euclidean distance between start and end coordinates
            dist = np.linalg.norm(start_coord - end_coord)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids

        # Not sure!!!!!!!!!!!!!!!!!!!!
        cost = self.ComputeDistance(start_id, goal_id)

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')

        pl.ion()
        pl.show()

    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

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


    def SetGoalParameters(self, goal_config, p = 0.2):
        goal_tf = np.eye(4,dtype = float)
        goal_tf[0,3] = goal_config[0]
        goal_tf[1,3] = goal_config[1]
        self.goal_config = goal_tf
        self.p = p

    def GenerateRandomConfiguration(self):
        # config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        # self.config = numpy.random.uniform(lower_limits, upper_limits)
        print("Simple environment")
        config = np.eye(4, dtype = float)
        config[0,3] = random.uniform(lower_limits[0],upper_limits[1])
        config[1,3] = random.uniform(lower_limits[0],upper_limits[1])
        # print("config shape {}".format(numpy.shape(self.config)))
        self.robot.SetTransform(config)
        if not self.robot.GetEnv().CheckCollision(self.robot) and not self.robot.CheckSelfCollision():
            return config
        else:
            print("---------collision")
            self.GenerateRandomConfiguration()

        return config


    def ComputeDistance(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations

        # get x, y from the tf, find the distance and return
        dx = start_config[0,3] - end_config[0,3]
        dy = start_config[1,3] - end_config[1,3]
        dist = np.sqrt(np.power(dx,2)+np.power(dy,2))
        return dist


    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        # qi, qr, qc are 4*4 ndarray
        step_dist = self.step
        qi = start_config
        qr = end_config
        q_start = qi
        qi_qr_dist = self.ComputeDistance(qi,qr)
        step_ratio = step_dist/qi_qr_dist
        qi_qc_vec = (qr - qi) * step_ratio
        qc = qi + qi_qc_vec
        qi_qc_dist = self.ComputeDistance(qi,qc)
        qi_qc_vec_unit = qi_qc_vec/ 20
        qi_qc_vec_unit_dist = self.ComputeDistance(qi,qi+qi_qc_vec_unit)
        current_dist = self.ComputeDistance(qi,qc)
        extend_tf = np.eye(4,dtype = float)
        extend_tf[0,3] = 0
        extend_tf[1,3] = 0
        if qi_qc_dist > qi_qr_dist:
            qc = qr
        # print("*********************")
        # print("current_dist {}, qi_qc_vec_unit_dist{}: ".format(current_dist,qi_qc_vec_unit_dist))
        while(current_dist > qi_qc_vec_unit_dist):
            current_dist = self.ComputeDistance(qi,qc)
            # print("current dist {}".format(current_dist))
            qi = qi + qi_qc_vec_unit
            extend_tf[0,3] = qi[0,3]
            extend_tf[1,3] = qi[1,3]

            self.robot.SetTransform(extend_tf)
            if not self.robot.GetEnv().CheckCollision(self.robot) and not self.robot.CheckSelfCollision():
                # print("No Collision")
                pass
            else:
                print("Collision")
                extend_tf[0,3] = q_start[0,3]
                extend_tf[1,3] = q_start[1,3]
                return q_start
        extend_tf[0,3] = qi[0,3]
        extend_tf[1,3] = qi[1,3]
        return qi
