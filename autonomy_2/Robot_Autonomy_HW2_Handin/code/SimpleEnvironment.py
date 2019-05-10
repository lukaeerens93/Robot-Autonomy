import numpy
import matplotlib.pyplot as pl
import random

class SimpleEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-1., -1.], [2, 2]]
        self.step = 2

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        self.config = [0] * 2;
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        goal_tf = numpy.eye(4,dtype = float)
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
        config = numpy.eye(4, dtype = float)
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
        dist = numpy.sqrt(numpy.power(dx,2)+numpy.power(dy,2))
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
        extend_tf = numpy.eye(4,dtype = float)
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


    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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
