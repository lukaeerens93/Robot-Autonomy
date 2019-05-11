import numpy
import time
import copy

class HerbEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.herb = herb
        self.config = [0] * len(self.robot.GetActiveDOFIndices())
        # add a table and move the robot into place
        # table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        # self.robot.GetEnv().Add(table)
        #
        # table_pose = numpy.array([[ 0, 0, -1, 0.6],
        #                           [-1, 0,  0, 0],
        #                           [ 0, 1,  0, 0],
        #                           [ 0, 0,  0, 1]])
        # table.SetTransform(table_pose)

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

        # TODO: Generate and return a random configuration
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        self.config = numpy.random.uniform(lower_limits, upper_limits)
        self.robot.SetActiveDOFValues(self.config)
        if not self.robot.GetEnv().CheckCollision(self.robot) and not self.robot.CheckSelfCollision():
            print("No Collision")
            return numpy.array(self.config)
        else:
            print("-------------")
            self.GenerateRandomConfiguration()
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
            time_now = 0
            while time_now < timeout:
                time_now = copy.deepcopy(time.clock() - time_now)
                g = copy.deepcopy(path[-1])
                i = 0
                while path[i+1].all() != g.all():
                    if len(path) > 3:
                        if self.Extend(path[i],path[i+2]) is not None:
                            del path[i+1]
                        i = i + 1
                    else:
                        break
                    if len(path) - i < 2:
                        break
                time_now = copy.deepcopy(time.clock() - time_now)

            return path
