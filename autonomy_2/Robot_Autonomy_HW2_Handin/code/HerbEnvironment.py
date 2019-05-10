import numpy
import time

class HerbEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.config = [0] * len(self.robot.GetActiveDOFIndices())
        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

        # goal sampling probability
        self.step = 2.5
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p


    def GenerateRandomConfiguration(self):
        self.config = [0] * len(self.robot.GetActiveDOFIndices())
        print("Herb environment")
        # TODO: Generate and return a random configuration
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        self.config = numpy.random.uniform(lower_limits, upper_limits)
        self.robot.SetActiveDOFValues(self.config)

        while(self.config is None or self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()):
            self.config = self.GenerateRandomConfiguration()

        return numpy.array(self.config)

    def ComputeDistance(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        dist = numpy.linalg.norm(end_config - start_config)
        return dist

    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration

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

        if qi_qc_dist > qi_qr_dist:
            qc = qr

        while(current_dist > qi_qc_vec_unit_dist):
            current_dist = self.ComputeDistance(qi,qc)
            print("current dist {}".format(current_dist))
            qi = qi + qi_qc_vec_unit
            self.robot.SetActiveDOFValues(qi)
            if not self.robot.GetEnv().CheckCollision(self.robot) and not self.robot.CheckSelfCollision():
                print("No Collision")
                pass
            else:
                print("Collision")
                return q_start
        return qi

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
            q1_, q2_ = numpy.random.choice(len_path, 2, replace = False)
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
            for i in numpy.linspace(0.0, 1.0, 3000):
                qi = (q2 - q1) * i + q1
                self.robot.SetActiveDOFValues(qi)
                # If a collision is imminent, then you must not consider shortening from these 2 points
                if (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(self.robot)):
                    connect = 0
                else:
                    # If no collision is imminent, go for it
                    connect = 1
            if (connect == 1):
                rng = numpy.arange(q1_ + 1, q2_)
                # for j in range(q1_ + 1, q2_, 1):
                #     print("q1_+1:{}, q2_:{}, size of path:{}".format(q1_+1, q2_, path.shape))
                path = numpy.delete(path, rng, 0)
                print("size of path".format(path.size))
            current_time = time.time()
            if (current_time - start_time >= timeout):
                print("exceed timeout")
                break
        print("shorten path: All step dist: {},and vertices number {} ".format(numpy.sum(self.step*len(path)), len(path)))
        return path

# numpy.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[1,1,1,1],[2,2,2,2]])