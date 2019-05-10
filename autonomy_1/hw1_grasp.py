#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
from scipy.spatial import ConvexHull

np.random.seed(0)
import scipy

# OpenRAVE
import openravepy

# openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)

curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata

# This sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
    if openrave_data_path == '':
        os.environ['OPENRAVE_DATA'] = ordata_path_thispack
    else:
        datastr = str('%s:%s' % (ordata_path_thispack, openrave_data_path))
        os.environ['OPENRAVE_DATA'] = datastr

# Set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack
samples = 10
# Get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()

class RoboHandler:
    def __init__(self):
        self.openrave_init()
        self.problem_init()
        print "Start Ordering Grasps"
        # Order grasps based on your own scoring metric
        #self.order_grasps()
        print "Finished Ordering Grasps"

        # Order grasps with noise
        print "Start Ordering Noisy Grasps"
        self.order_grasps_noisy()
        print "Finished Ordering Noisy Grasps"

    # The usual initialization for openrave
    def openrave_init(self):
        self.env = openravepy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.GetViewer().SetName('HW1 Viewer')
        self.env.Load('models/%s.env.xml' % PACKAGE_NAME)
        # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
        self.robot = self.env.GetRobots()[0]
        self.manip = self.robot.GetActiveManipulator()
        self.end_effector = self.manip.GetEndEffector()

    # Problem specific initialization - load target and grasp module
    def problem_init(self):
        #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
        #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
        self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

        # Change the location so it's not under the robot
        T = self.target_kinbody.GetTransform()
        T[0:3, 3] += np.array([0.5, 0.5, 0.5])
        self.target_kinbody.SetTransform(T)
        self.env.AddKinBody(self.target_kinbody)

        # create a grasping module
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)

        # if you want to set options, e.g. friction
        options = openravepy.options
        options.friction = 0.1
        if not self.gmodel.load():
            self.gmodel.autogenerate(options)

        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps

    # Order the grasps - call eval grasp on each, set the 'performance' index, and sort
    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy()  # you should change the order of self.grasps_ordered

        # Weights for each score
        weights = np.array([.5, .5])

        # Get the Score of score and add it performance
        performance = np.zeros(shape=(np.shape(self.grasps_ordered)[0], weights.shape[0]), dtype=np.float64)

        for index, grasp in enumerate(self.grasps_ordered):
            performance[index, :] = np.array(self.eval_grasp(grasp), dtype=np.float64)

        # Normalize the performance
        performance = performance / sum(performance, 0)

        # Calculated the weighted average
        for index, grasp in enumerate(self.grasps_ordered):
            self.grasps_ordered[index][self.graspindices.get('performance')] = np.dot(weights, performance[index, :])

        # sort!
        order = np.argsort(self.grasps_ordered[:, self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

    # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp
    def order_grasps_noisy(self):
        # you should change the order of self.grasps_ordered_noisy
        self.grasps_ordered_noisy = self.grasps.copy()

        # Weights
        weights = np.array([.5, .5])

        # Get the Score of score and add it performance
        performance = np.zeros(shape=(np.shape(self.grasps_ordered_noisy)[0], weights.shape[0]), dtype=np.float64)

        # Sample random points
        for index, grasp in enumerate(self.grasps_ordered_noisy):
            eval_sum = np.zeros(shape=(samples, weights.shape[0]), dtype=np.float64)
            for i in range(samples):
                eval_sum[i, :] = self.eval_grasp(self.sample_random_grasp(grasp))
            # Update the Performance
            pass
            performance[index, :] = np.mean(eval_sum, 0)
            # if index>10:
            #     break
            # Message for debugging
            if index % 10 == 0:
                print "Grasp {}/{}".format(index, self.grasps.shape[0])

        # Normalize the performance
        performance = performance / sum(performance, 0)

        # Update the grasp performance values
        for index, grasp in enumerate(self.grasps_ordered_noisy):
            self.grasps_ordered_noisy[index][self.graspindices.get('performance')] = np.dot(weights,performance[index, :])
            # if index>10:
            #     break
        # sort!
        order = np.argsort(self.grasps_ordered_noisy[:, self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered_noisy = self.grasps_ordered_noisy[order]

    # Function to evaluate grasps
    # Returns a score, which is some metric of the grasp
    # Higher score should be a better grasp

    def eval_grasp(self, grasp):
        with self.robot:
            # Contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp=grasp, translate=True,
                                                                               forceclosure=False)

                obj_position = self.gmodel.target.GetTransform()[0:3, 3]
                # for each contact
                G = np.zeros(shape=(6, contacts.shape[0]), dtype=np.float64)  # the wrench matrix

                for index, c in enumerate(contacts):
                    pos = c[0:3] - obj_position
                    dir = -c[3:]  # this is already a unit vector

                    # Fill the G matrix
                    G[:, index] = np.hstack((dir, np.cross(pos, dir)))

                    # TODO use G to compute scrores as discussed in class

                    # Calculate the singular values of G
                    [_, S, _] = np.linalg.svd(G)

                    # M1 -- Return the minimum singular value
                    score1 = S[-1]

                    # M2 -- Volume metric
                    score2 = np.abs(np.linalg.det(np.dot(G, np.transpose(G))))
                    if np.isnan(score2):
                        print "Not defined"

                return [score1, score2]

            except openravepy.planning_error, e:
                # you get here if there is a failure in planning
                # example: if the hand is already intersecting the object at the initial position/orientation
                return 0.00  # TODO you may want to change this, ANS -- I don't want to change it because it's correct :)

                # heres an interface in case you want to manipulate things more specifically
                # NOTE for this assignment, your solutions cannot make use of graspingnoise
                #      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
                #      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
                #      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
                #      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
                #      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
                #            direction             = grasp[self.graspindices.get('igraspdir')], \
                #            roll                  = grasp[self.graspindices.get('igrasproll')], \
                #            position              = grasp[self.graspindices.get('igrasppos')], \
                #            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
                #            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
                #            target                = self.target_kinbody, \
                #            graspingnoise         = 0.0, \
                #            forceclosure          = True, \
                #            execute               = False, \
                #            outputfinal           = True, \
                #            translationstepmult   = None, \
                #            finestep              = None )

    # given grasp_in, create a new grasp which is altered randomly
    # you can see the current position and direction of the grasp by:
    # grasp[self.graspindices.get('igrasppos')]
    # grasp[self.graspindices.get('igraspdir')]

    def sample_random_grasp(self, grasp_in):
        grasp = grasp_in.copy()

        # Sample random position
        RAND_DIST_SIGMA = 0.0001  # TODO you may want to change this
        # Set a random position
        grasp[self.graspindices['igrasppos']] = np.random.normal(grasp[self.graspindices['igrasppos']], RAND_DIST_SIGMA,
                                                                 (1, 3))

        # Sample random orientation
        RAND_ANGLE_SIGMA = np.pi / 500  # TODO you may want to change this
        grasp[self.graspindices['igraspdir']] = np.random.normal(grasp[self.graspindices['igraspdir']],
                                                                 RAND_ANGLE_SIGMA, (1, 3))  # 43, 44, 45
        grasp[self.graspindices['igrasproll']] = np.random.normal(grasp[self.graspindices['igrasproll']],
                                                                  RAND_ANGLE_SIGMA, 1)  # Don't remember

        return grasp

    # displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
            with self.gmodel.GripperVisibility(self.gmodel.manip):
                time.sleep(0.1)  # let viewer update?
                try:
                    with self.env:
                        contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp=grasp, translate=True,
                                                                                       forceclosure=True)
                        # if mindist == 0:
                        #  print 'grasp is not in force closure!'
                        contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                        self.gmodel.robot.GetController().Reset(0)
                        self.gmodel.robot.SetDOFValues(finalconfig[0])
                        self.gmodel.robot.SetTransform(finalconfig[1])
                        self.env.UpdatePublishedBodies()
                        time.sleep(delay)
                except openravepy.planning_error, e:
                    print 'bad grasp!', e


if __name__ == '__main__':
    robo = RoboHandler()

    # pickle.dump(robo, open("robo.p", "wb"))

    # pass

    '''
    # Show the best 10
    print "Showing best 10 indexes"
    for grasp in robo.grasps_ordered[:10, :]:
      robo.show_grasp(grasp)
      print grasp[42]

    # Show the worst 10
    print "Showing worst 10 indexes"
    for grasp in robo.grasps_ordered[-5:-1, :]:
      robo.show_grasp(grasp)
    '''

    # Show the best 10 for noisy case
    print "Showing best 10 indexes"
    for grasp in robo.grasps_ordered_noisy[:10, :]:
        robo.show_grasp(grasp)
        print "Performance = {}".format(grasp[42])

    # Show the worst 10 for noisy case
    print "Showing worst 10 indexes"
    for grasp in robo.grasps_ordered_noisy[-10:-1, :]:
        robo.show_grasp(grasp)
        print "Performance = {}".format(grasp[42])
    pass

    # time.sleep(10000) #to keep the openrave window open
